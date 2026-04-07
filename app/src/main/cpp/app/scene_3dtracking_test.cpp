#include"ObjectTracking2/include/ObjectTracking2.h"
#include"Basic/include/App.h"
#include<opencv2/highgui.hpp>
#include<opencv2/calib3d.hpp>
#include"BFC/portable.h"
#include"arengine.h"
#include"scene.h"
#include"demos/utils.h"
#include"demos/model.h"
#include"config.hpp"
#include"utilsmym.hpp"
#include"scenegui.h"
#include"arucopp.h"
#include<thread>
#include"RPCServer.h"
#include"Basic/include/BasicData.h"
#include"databank.hpp"
#include"markerdetector.hpp"
#include<thread>
#include<iostream>
using namespace cv;


extern void AppCommandServerMain(int port);

glm::mat4 CameraVmat=glm::mat4(1.0f);

namespace{
struct MarkerPose {
    typedef std::tuple<int,glm::mat4,cv::Vec3d,cv::Vec3d> ItemType;
    std::vector<ItemType> markers;
};

void decomposeRT(const Matx44f &m,cv::Matx33f &R,cv::Vec3f &tvec,bool gl2cv){
    const float *v=m.val;
//    CV_Assert(fabs(v[3])<1e-3f&&fabs(v[7])<1e-3&&fabs(v[11])<1e-3&&fabs(v[15]-1.0f)<1e-3f);
    if(!(fabs(v[3])<1e-3f&&fabs(v[7])<1e-3&&fabs(v[11])<1e-3&&fabs(v[15]-1.0f)<1e-3f)){
        errorf("CV_ASSERT Failed: (fabs(v[3])<1e-3f&&fabs(v[7])<1e-3&&fabs(v[11])<1e-3&&fabs(v[15]-1.0f)<1e-3f)")
    }
    if(gl2cv){
        tvec[0]=v[12];tvec[1]=-v[13];tvec[2]=-v[14];
        R=cv::Matx33f(v[0],v[4],v[8],-v[1],-v[5],-v[9],-v[2],-v[6],-v[10]);
    }
    else{
        tvec[0]=v[12];tvec[1]=v[13];tvec[2]=v[14];
        R=cv::Matx33f(v[0],v[4],v[8],v[1],v[5],v[9],v[2],v[6],v[10]);
    }
}
class ArucoDetector: public ARModule{
    ArucoPP _detector;
    bool _refineCorners=true;
    bool _isStatic=true;
    //int _nSum=0;
    //cv::Vec3f _mrvec,_mtvec;
    std::list<std::pair<cv::Vec3f,cv::Vec3f>> _vhist;
    bool _use_opencv_detector=false; //2025-10-13 控制是使用原始的opencv检测Marker,还是用更新的检测代码(正常为false)
public:
    int Init(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        _detector.loadTemplate(Config::AppDataDir+("/templ_1.json"));
        return STATE_OK;
    }
    int Update(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        if(!frameDataPtr->image.empty()){
            cv::Mat img=frameDataPtr->image.front();
            auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs"));
            cv::Matx44f vmat=frame_data.cameraMat;
            CameraVmat=CV_Matx44f_to_GLM_Mat4(vmat);
            const cv::Matx33f &camK=frameDataPtr->colorCameraMatrix;

            cv::Vec3d rvec,tvec;
            if(_detector.detect(img,camK,rvec,tvec)){
                cv::Matx44f Tmw=GLM_Mat4_to_CV_Mat(MarkerDetector::GetTransMatFromRT(rvec,tvec,CV_Matx44f_to_GLM_Mat4(vmat)));

                if(_isStatic){
//                    if(_nSum==0){_mrvec=_mtvec=cv::Vec3f(0.f,0.f,0.f);}
                    cv::Matx44f Twc=vmat;
                    Matx33f R;
                    Vec3f t,r;
                    decomposeRT(Tmw,R,t,true);
                    cv::Rodrigues(R,r);

                    _vhist.push_back(std::make_pair(r,t));
                    if(_vhist.size()>60)
                        _vhist.pop_front();

                    cv::Vec3f mrvec=cv::Vec3f(0.f,0.f,0.f), mtvec=mrvec;
                    for(auto &v : _vhist)
                    {
                        mrvec+=v.first;
                        mtvec+=v.second;
                    }
                    float s=1.f/_vhist.size();

                    //_mrvec=(_mrvec*float(_nSum)+r)/(_nSum+1);
                    //_mtvec=(_mtvec*float(_nSum)+t)/(_nSum+1);
                    //_nSum++;
                    Tmw=FromRT(mrvec*s,mtvec*s,false);

                    //===============
                    //Tmw=FromRT(_mrvec,_mtvec,false);  //--Only For Debug--
                }
                else{
                    cv::Matx44f Twc=vmat;
                    Matx33f R;
                    Vec3f t,r;
                    decomposeRT(Tmw,R,t,true);
                    cv::Rodrigues(R,r);
                    //===============
                    Tmw=FromRT(r,t,false);  //--Only For Debug--
                }
                MarkerPose pose;
                pose.markers.emplace_back(0,CV_Matx44f_to_GLM_Mat4(Tmw),rvec,tvec);
                sceneData.setData("MarkerPose",pose);

                DataBank::Global().set("vmat",vmat);
                DataBank::Global().set("Tmw",Tmw);
            }
        }
        return STATE_OK;
    }
};

std::shared_ptr<ARApp> construct_engine(){
    std::string appName="TestApp"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）

    std::vector<ARModulePtr> modules;
    modules.push_back(createModule<ARInputs>("ARInputs"));
    //modules.push_back(createModule<ObjectTracking2>("ObjectTracking2",&ObjectTracking2::create));  //用createModule创建模块，必须指定一个模块名，并且和server上的模块名对应！！
    modules.push_back(createModule<ArucoDetector>("ArucoDetector"));

    auto appData=std::make_shared<AppData>();
    auto sceneData=std::make_shared<SceneData>();

    appData->argc=1;
    appData->argv=nullptr;
    appData->engineDir="./AREngine/";  // for test
    appData->dataDir="./data/";        // for test

    std::shared_ptr<ARApp> app=std::make_shared<ARApp>();
    app->init(appName,appData,sceneData,modules);

//    std::any cmdData=std::string(MakeSdcardPath("Download/3d/box1_new/box1.ply"));
//    app->call("ObjectTracking2",ObjectTracking2::CMD_SET_MODEL,cmdData);

    return app;
}



class Scene_3dtracking_test: public IScene{
    std::shared_ptr<ARApp> _eng;
    ARServerManager &manager=ARServerManager::instance();
    SceneGui gui;

    Model mAxle{""},mTire{""};
    glm::mat4 mTirePose{0.f},mAxlePose{0.f};
    glm::mat4 mLastSavedAxlePose{};
    glm::mat4 mAxleCustomerPose=glm::mat4(1.0f);
    glm::mat4 mAxleFixedPose=glm::mat4(0); //相对于相机的位姿，并非世界坐标系
    bool mIsInteractiveMode{false};
    glm::mat4 mProject{},mView{};
    bool mInteractiveUpdateState{true};
    bool mMarkerRefinedUpdateState{true};
//    bool mUpdatePose{true};
public:
    virtual bool initialize(const XrInstance instance,const XrSession session){
        _eng=construct_engine();
        _eng->start();

        std::thread server_thread=std::thread(AppCommandServerMain,std::stoi(Config::global().get("3DTrackingPort")));
        server_thread.detach(); //离开作用域也不会停止
        //=========================== For UI ===============================
        SceneGui::TextItem text,main_text; SceneGui::ImageItem image;

        glm::mat4 model = glm::mat4(1.0f);
        float scale = 0.5f,width=120, height=80;
        model = glm::translate(model, glm::vec3(-0.0f, -0.3f, 5.0f));
        model = glm::rotate(model, glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::scale(model, glm::vec3(scale * (width / height), scale, 1.0f));

        text.text="欢迎使用本软件, 按O键进入初始化部分";
        text.translate_model=glm::translate(glm::mat4(1.0f),{-0.3,-0.1,-2});
        text.scale=glm::vec3{0.25,0.3,0.3};
        text.use_view=false;
//        image.use_view=false;
        image.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.0f,0.1f,-4.0f));
        image.image=ConvertBGRImageToRGB(cv::imread(Config::AppDataDir+("/Image/Logo.png")));
        image.scale={0.3f,0.3f,0.3f};
//        cv::resize(image.image,image.image,{},0.6,0.6);
        main_text.text="AR机轮装配引导系统";
        main_text.translate_model=glm::translate(image.translate_model,{-0.66f,-0.5f,0.f});
        main_text.scale={1.6f,1.6f,1.6f};

        gui.add_text(text,"info");
        gui.add_image(image); gui.add_text(main_text);
        mAxle=Model::GetModelFromLocalFile("TestAxleModel","/storage/emulated/0/RokidData/Model/RM/axle/axle.obj");
        mTire=Model::GetModelFromLocalFile("TestTireModel","/storage/emulated/0/RokidData/Model/Tire/tire.ply");

        //=============== Offline Debug ==================
        cv::Matx44f obj_pose={0.99559009,0.019166477,-0.091831148,0.50098264,0,-0.97890586,-0.2043117,0.61416101,-0.093809985,0.2034107,-0.97458905,0.074030519,0,0,0,1};
//        DataBank::Global().set("obj_to_marker",obj_pose);

        //--------------------------- Load From File ----------------------------
        auto file_obj_to_marker_mat4=GlmMat4_from_String(Config::global().get("ReceivedObjectToMarker"),',',glm::mat4(0));
        if(file_obj_to_marker_mat4!=glm::mat4(0)&&file_obj_to_marker_mat4!=glm::mat4(1.0f)){
            cv::Matx44f file_obj_to_marker=GLM_Mat4_to_CV_Mat(file_obj_to_marker_mat4);
            DataBank::Global().set("obj_to_marker",file_obj_to_marker);
            infof("Load Obj2Marker From Config: \n%s",GlmMat4_to_String(file_obj_to_marker_mat4).c_str())
        }
        //============================================
        return true;
    }
    void inputEvent(int leftright, const ApplicationEvent &event)override{
        gui.inputEvent(leftright,event);
    }
    void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection)override{
        gui.rayEvent(linePoint,lineDirection);
    }
    void keypadEvent(const std::string &key_name)override{
        static std::map<std::string_view,long long> LastPressedMap; //某个按键最后一次被触发是什么时候,防止反复触发
        static long long MinGap=600; //最小触发间隔为200ms
        if(CurrentMSecsSinceEpoch()-LastPressedMap[key_name]<MinGap) return;
        LastPressedMap[key_name]=CurrentMSecsSinceEpoch();

        if(key_name=="select"){
            if(mIsInteractiveMode) mInteractiveUpdateState=!mInteractiveUpdateState;
            else mMarkerRefinedUpdateState=!mMarkerRefinedUpdateState;
            if(mIsInteractiveMode){
                if(mInteractiveUpdateState){
                    mAxleCustomerPose=mAxlePose;
                    if(mAxleFixedPose==glm::mat4(0)) mAxleFixedPose=glm::scale(glm::translate(glm::mat4(1.0f),{0,0,-2}),{0.159,0.159,0.159}); //First In
                    else mAxleFixedPose=mView*mAxleCustomerPose;
                }
                else{
                    mAxlePose=glm::inverse(mView)*mAxleFixedPose;
                }
            }
        }
        else if(key_name=="up"){
            mIsInteractiveMode=!mIsInteractiveMode;
            if(mIsInteractiveMode){
                mAxleCustomerPose=mAxlePose;
                if(mAxleFixedPose==glm::mat4(0)) mAxleFixedPose=glm::scale(glm::translate(glm::mat4(1.0f),{0,0,-2}),{0.159,0.159,0.159}); //First In
                else mAxleFixedPose=mView*mAxleCustomerPose;
            }
            else{
                mAxleCustomerPose=glm::mat4(1.0f);
                mInteractiveUpdateState=true;
            }
        }
    }

    void update_info(){
        auto info=gui.get_text_item("info");
        if(!info) return;
        if(mIsInteractiveMode){
            if(mInteractiveUpdateState) info->text="移动视角，使轮轴摆放在正确位置，按 select 键确认";
            else info->text="位置已固定，按 select 键可重新摆放";
        }
        else{
            if(mMarkerRefinedUpdateState) info->text="当前正在更新轮轴位姿";
            else info->text="轮轴位姿已固定";
        }
    }
    virtual void renderFrame(const XrPosef &pose,const glm::mat4 &project,const glm::mat4 &view,int32_t eye) {
        update_info();
        mProject=project;
        if(eye==0) mView=view;
        gui.render(project,view,eye);

        if(mIsInteractiveMode){
            if(mInteractiveUpdateState){
                mAxle.render(project,SceneGui::no_view_mat(eye),glm::scale(mAxleFixedPose,{1,1,1}));
            }
            else{
                mAxle.render(project,view,glm::scale(mAxlePose,{1,1,1}));
            }
        }
        else{
            auto obj_to_marker_ptr=DataBank::Global().get<cv::Matx44f>("obj_to_marker");
            auto vmat_ptr=DataBank::Global().get<cv::Matx44f>("vmat");
            auto Tmw_ptr=DataBank::Global().get<cv::Matx44f>("Tmw");
            auto axle_point_ptr=DataBank::Global().get<glm::vec3>("tire_average_point");

            if(Tmw_ptr&&vmat_ptr&&axle_point_ptr&&obj_to_marker_ptr){
                cv::Matx44f obj_to_marker=*obj_to_marker_ptr;
                cv::Matx44f T=*Tmw_ptr*obj_to_marker;

                Matx33f R;
                Vec3f t;
                decomposeRT(T.t(),R,t,false);
                T=FromR33T(R,t,true);
                infof("T: %s",GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(T)).c_str())

                if(mMarkerRefinedUpdateState){
                    mTirePose=CV_Matx44f_to_GLM_Mat4(T);
                    cv::Vec4f P=T*cv::Vec4f(0,0,0,1);
                    cv::Vec4f Q=T*cv::Vec4f(0,0,1,1);

                    mAxlePose=mTirePose;//GetMatrixWithoutScale(mTirePose);
                    glm::mat4 axleT=glm::scale(glm::mat4(1.0f),{0.159,0.159,0.159});
                    axleT=glm::translate(axleT,{0,-0.1*(1.0/0.159),-0.3*(1.0/0.159)});
                    mAxlePose=mAxlePose*axleT;

                    glm::mat4 tireT=glm::scale(glm::mat4(1.0f),{0.001,0.001,0.001});
                    mTirePose=tireT*mTirePose;
                }
            }
//            mTire.render(project,view,mTirePose);
            mAxle.render(project,view,mAxlePose);
        }
        if(mLastSavedAxlePose!=mAxlePose){
            DataBank::Global().set("axle_pose",mAxlePose);
            DataBank::Global().set("tire_pose",mTirePose);
            infof("Updated Axle Pose: \n%s",GlmMat4_to_String(mAxlePose).c_str())
            mLastSavedAxlePose=mAxlePose;
        }
    }
//    virtual void renderFrame(const XrPosef &pose,const glm::mat4 &project,const glm::mat4 &view,int32_t eye) {
//        mProject=project; mView=view;
//        gui.render(project,view,eye);
//
//        auto marker_pose_ptr=DataBank::Global().get<cv::Matx44f>("marker_to_world");
//        auto obj_pose_ptr=DataBank::Global().get<cv::Matx44f>("obj_to_world");
//
//        auto vmat_ptr=DataBank::Global().get<cv::Matx44f>("vmat");
//        auto Tmw_ptr=DataBank::Global().get<cv::Matx44f>("Tmw");
//        auto axle_point_ptr=DataBank::Global().get<glm::vec3>("tire_average_point");
//        if(Tmw_ptr&&vmat_ptr&&axle_point_ptr&&marker_pose_ptr&&obj_pose_ptr){
//            cv::Matx44f T;
//            cv::Matx44f marker_pose=*marker_pose_ptr;
//            cv::Matx44f obj_pose=*obj_pose_ptr;
//            cv::Matx44f T1=marker_pose;
//            cv::Matx44f T2=*Tmw_ptr;
//            T=T2*T1.inv();
//            T=T*obj_pose;
//            //infof("T1: %s",GlmMat4_to_String(T1).c_str())
//            infof("T2: %s",GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(obj_pose)).c_str())
//
//            Matx33f R;
//            Vec3f t;
//            decomposeRT(T.t(),R,t,false);
//            T=FromR33T(R,t,true);
//            infof("T: %s",GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(T)).c_str())
//
//
//            cv::Vec4f axle_points((*axle_point_ptr).x,(*axle_point_ptr).y,(*axle_point_ptr).z,1);
//            axle_points=(T2*T1.inv())*obj_pose*axle_points;
//            axle_points[0]/=axle_points[3];
//            axle_points[1]/=axle_points[3];
//            axle_points[2]/=axle_points[3];
//            axle_points[3]/=axle_points[3];
//            //
//            axle_points[2]=-axle_points[2];
//            axle_points[1]=-axle_points[1];
//
//            //-------------------------------
//            if(mUpdatePose){
//                infof("axle_points: %f,%f,%f,%f",axle_points[0],axle_points[1],axle_points[2],axle_points[3])
//                mTirePose=CV_Matx44f_to_GLM_Mat4(T);
//                mAxlePose=GetMatrixWithoutScale(mTirePose);
//                glm::mat4 axle_translation=glm::translate(glm::mat4(1.0f),{0,0,-0.1});
//                mAxlePose=axle_translation*mAxlePose;
////                mAxlePose=glm::translate(mAxlePose,{0,0,-0.1});
//                DataBank::Global().set("axle_pose",mAxlePose);
//                DataBank::Global().set("tire_pose",mTirePose);
//                infof("Axle Pose: \n%s",GlmMat4_to_String(mAxlePose).c_str())
//            }
//        }
////        mTire.render(project,view,glm::scale(mTirePose,{1,1,1}));
//        mAxle.render(project,view,glm::scale(mAxlePose,{0.159,0.159,0.159}));
//    }
    virtual void close() {
        if(_eng){
            _eng->stop();
        }
    }
};
}

std::shared_ptr<IScene> _createScene_3dtracking_test(){
    return std::make_shared<Scene_3dtracking_test>();
}

