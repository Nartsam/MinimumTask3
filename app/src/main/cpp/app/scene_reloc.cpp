#include"Basic/include/RPC.h"
#include"Basic/include/ARModule.h"
#include"Basic/include/App.h"
#include"opencv2/core.hpp"
#include"arengine.h"
#include"demos/gui.h"
#include"scene.h"
#include"demos/utils.h"
#include"demos/model.h"
#include"demos/text.h"
#include"scenegui.h"
#include"utilsmym.hpp"
#include"BFC/netcall.h"

using namespace cv;

namespace{

struct MapPose{
    glm::mat4 pose;
    int inliers;
    float inlier_ratio;
};
void decomposeRT(const Matx44f &m,cv::Matx33f &R,cv::Vec3f &tvec,bool gl2cv){
    const float *v=m.val;
    CV_Assert(fabs(v[3])<1e-3f&&fabs(v[7])<1e-3&&fabs(v[11])<1e-3&&fabs(v[15]-1.0f)<1e-3f);
    if(gl2cv){
        tvec[0]=v[12];
        tvec[1]=-v[13];
        tvec[2]=-v[14];
        R=cv::Matx33f(v[0],v[4],v[8],-v[1],-v[5],-v[9],-v[2],-v[6],-v[10]);
    }
    else{
        tvec[0]=v[12];
        tvec[1]=v[13];
        tvec[2]=v[14];
        R=cv::Matx33f(v[0],v[4],v[8],v[1],v[5],v[9],v[2],v[6],v[10]);
    }
}

static glm::mat4 GetTransMatFromR33T(const cv::Matx33f &R,const cv::Vec3d &tvec,const glm::mat4 &vmat){
    auto tmp1=FromRT(cv::Vec3f(0,0,0),cv::Vec3f(0,0,0),true);
    auto tmp_false=(tmp1*FromR33T(R,tvec,false)).t();
    glm::mat4 trans=glm::inverse(vmat)*CV_Matx44f_to_GLM_Mat4(tmp_false);
    return trans;
}


class MapDetector: public ARModule{
    std::shared_ptr<ff::NetcallServer> _netcallServer;
    bool _isStatic=false;
    int _nSum=0;
    cv::Vec3f _mrvec,_mtvec;
public:
    int Init(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        try{
//            _netcallServer=std::make_shared<ff::NetcallServer>("10.102.33.100",8001);
            _netcallServer=std::make_shared<ff::NetcallServer>("101.76.210.138",8000);
            infof("Connect to Server Success")
        }
        catch(std::exception e){
            errorf("Connect to Server Failed: %s",e.what())
        }

        infof("Connect Server Success")
        return STATE_OK;
    }
    int Update(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        if(!frameDataPtr->image.empty()){
            cv::Mat refined_img=frameDataPtr->image.front();
            auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs"));
            cv::Matx44f vmat=frame_data.cameraMat;
            const cv::Matx33f &camK=frameDataPtr->colorCameraMatrix;
            std::string cameraModel="OPENCV_FISHEYE",cameraParams="281.60213015, 281.37377039, 318.69481832, 243.690702, 0.11946399, 0.06202764, -0.28880297, 0.2142014";
//            std::string cameraModel="PINHOLE",cameraParams="232.26183,232.07349,318.1832,244.91544";
            if(_netcallServer){
                cv::Mat img;
                if(1){ //original
                    cv::cvtColor(frame_data.img,img,CV_GRAY2BGR);
                }
                else{
                    img=refined_img.clone();
                }
                ff::NetObjs objs={{"image",        ff::nct::Image(img,".jpg")},
                                  {"camera_model", cameraModel},
                                  {"camera_params",cameraParams}};
                try{
                    auto res=_netcallServer->call(objs);
                    if(res.hasError()){
                        infof("Netcall Error: %s",res["error"].get<std::string>().c_str());
                    }
                    else{
//                        auto poseData=res["pose"].get<std::string>();

                        cv::Mat poseW2C=res["pose"].getm();
                        int num_inliers=res["num_inliers"].get<int>();
                        float inlier_ratio=res["inlier_ratio"].get<float>();

                        cv::Matx33f R;
                        cv::Vec3f t;

                        //get R, t from poseW2C (3x4 matrix)
                        for(int i=0;i<3;++i){
                            for(int j=0;j<3;++j){
                                R(i,j)=poseW2C.at<float>(i,j);
                            }
                            t(i)=poseW2C.at<float>(i,3);
                        }
                        cv::Matx44f Tmw=GLM_Mat4_to_CV_Mat(GetTransMatFromR33T(R,t,CV_Matx44f_to_GLM_Mat4(vmat)));

                        if(_isStatic){
                            if(_nSum==0){_mrvec=_mtvec=cv::Vec3f(0.f,0.f,0.f);}
                            cv::Matx44f Twc=vmat;
                            Matx33f R;
                            Vec3f t,r;
                            decomposeRT(Tmw,R,t,true);
                            cv::Rodrigues(R,r);

                            _mrvec=(_mrvec*float(_nSum)+r)/(_nSum+1);
                            _mtvec=(_mtvec*float(_nSum)+t)/(_nSum+1);
                            _nSum++;
                            Tmw=FromRT(_mrvec,_mtvec,true);
                        }
                        MapPose pose;
                        glm::mat4 res=CV_Matx44f_to_GLM_Mat4(Tmw);
                        pose.pose=res;
                        pose.inliers=num_inliers;
                        pose.inlier_ratio=inlier_ratio;
                        sceneData.setData("MapPose",pose);
                        infof("MapPose: %s",GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(Tmw)).c_str());
                    }
                }
                catch(const std::exception &e){
                    infof("Netcall Exception: %s",e.what());
                }

                return STATE_OK;
            }
        }
        return STATE_OK;
    }

};

std::shared_ptr<ARApp> construct_engine(){
    std::string appName="TestApp"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）
    std::vector<ARModulePtr> modules;
    modules.push_back(createModule<ARInputs>("ARInputs"));
    modules.push_back(createModule<MapDetector>("MapDetector"));
    auto appData=std::make_shared<AppData>();
    auto sceneData=std::make_shared<SceneData>();
    appData->argc=1;
    appData->argv=nullptr;
    appData->engineDir="./AREngine/";  // for test
    appData->dataDir="./data/";        // for test
    std::shared_ptr<ARApp> app=std::make_shared<ARApp>();
    app->init(appName,appData,sceneData,modules);
    return app;
}
}


namespace{
class SceneRelocTest:public IScene {
    std::shared_ptr<ARApp> _eng;
    SceneGui gui;
    glm::mat4 mProject{},mView{};
    std::string AppDataDir="/storage/emulated/0/RokidData/";
    Model mModel{""};
    glm::vec3 mModelScale{1,1,1};
    glm::mat4 mModelRenderPose=glm::scale(glm::translate(glm::mat4(1.0f),{0,0,-2}),mModelScale);

    glm::mat4 Tobj_in_map=mModelRenderPose;

public:
    bool set_render_model(const std::string &file_path,const glm::vec3 &model_scale={1.0f,1.0f,1.0f}){
        mModel=Model::GetModelFromLocalFile("TestModel",MakeSdcardPath(file_path));
        mModelScale=model_scale;
        return true;
    }
    bool set_tobj_in_map(const glm::mat4 &tobj_in_map){
        this->Tobj_in_map=tobj_in_map;
        return true;
    }
//====================================================================

    bool initialize(const XrInstance instance,const XrSession session)override{
        _eng = construct_engine();
        _eng->start();
        SceneGui::TextItem  main_text;
        main_text.text="重定位测试";
        main_text.translate_model=glm::translate(glm::translate(glm::mat4(1.0f),glm::vec3(-0.0f,0.1f,-4.0f)),{-0.66f,-0.5f,0.f});
        main_text.scale={1.6f,1.6f,1.6f};
        gui.add_text(main_text);

        //=============== 设置显示的模型 =================
        set_render_model("/storage/emulated/0/RokidData/Model/RM/axle/axle.obj",
                         {0.159,0.159,0.159}
                         );
        return true;
    }

    void inputEvent(int leftright, const ApplicationEvent &event)override{
        gui.inputEvent(leftright,event);
        std::stringstream ss;
        ss<<"GetEvent: LR: "<<leftright<<". Event: (thumbstick xy): ";
        ss<<event.thumbstick_x<<", "<<event.thumbstick_y;
        ss<<". Trigger: "<<event.trigger<<", Sqz: "<<event.squeeze;
//        infof(ss.str().c_str());
//        infof("===========triggerEvent: %d",event.click_trigger);
    }
    void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection)override{
        gui.rayEvent(linePoint,lineDirection);
    }
    virtual void renderFrame(const XrPosef &pose,const glm::mat4 &project,const glm::mat4 &view,int32_t eye) {
        mProject=project; mView=view;
        gui.render(project,view,eye);

        if(_eng){
            auto _res=_eng->sceneData->getData("MapPose");
            if(_res.has_value()){
                auto res = std::any_cast<MapPose>(_res);
                // res.pose = Tmw: 地图坐标系 -> 眼镜世界坐标系
                // 目标点在地图坐标系中的位置
                Tobj_in_map = glm::scale(
                        glm::translate(glm::mat4(1.0f), glm::vec3(-0.884695,-0.520542,1.092438)),
                        {1,1,1}//mModelScale //把scale移动到最后做
                );
//                Tobj_in_map = glm::scale(
//                    glm::translate(glm::mat4(1.0f), glm::vec3(3.9974660696046693f, 0.10452356803454364f, -0.7892968036677749f)),
//                    mModelScale
//                );
                // 变换到眼镜世界坐标系
                mModelRenderPose = res.pose * Tobj_in_map;
                infof("Update Model Render Pose: %s",GlmMat4_to_String(mModelRenderPose).c_str())
            }
        }
        mModel.render(project,view,glm::scale(mModelRenderPose,mModelScale));
    }

    virtual void close() {
        if(_eng) _eng->stop();
    }
};
}

std::shared_ptr<IScene> _createSceneRelocTest() {
    return std::make_shared<SceneRelocTest>();
}
