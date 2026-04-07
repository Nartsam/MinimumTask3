#include"Basic/include/RPC.h"
#include"Basic/include/ARModule.h"
#include"Basic/include/App.h"
#include"opencv2/core.hpp"
#include"arengine.h"
#include"scene.h"
#include"demos/utils.h"
#include"demos/model.h"
#include"demos/cylinder.h"
#include"demos/text.h"
#include"markerdetector.hpp"
#include"utilsmym.hpp"
#include"config.hpp"
#include"recorder.hpp"
#include"arucopp.h"
#include"SceneGui.h"

using namespace cv;

/*
 * 用于录制相机画面和对应的vmat位姿
*/

namespace {
std::mutex CameraMutex;
cv::Mat CameraImage;
cv::Matx44f CameraVmat;

struct MarkerPose {
    typedef std::tuple<int,glm::mat4,cv::Vec3d,cv::Vec3d> ItemType;
    std::vector<ItemType> markers;
};
void decomposeRT(const Matx44f &m,cv::Matx33f &R,cv::Vec3f &tvec,bool gl2cv){
    const float *v=m.val;
    CV_Assert(fabs(v[3])<1e-3f&&fabs(v[7])<1e-3&&fabs(v[11])<1e-3&&fabs(v[15]-1.0f)<1e-3f);
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
    int _nSum=0;
    cv::Vec3f _mrvec,_mtvec;
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
            const cv::Matx33f &camK=frameDataPtr->colorCameraMatrix;
            //============================
            CameraMutex.lock();
            CameraImage=img; CameraVmat=vmat;
            CameraMutex.unlock();
            //============================
            cv::Vec3d rvec,tvec;
            if(_detector.detect(img,camK,rvec,tvec)){
                cv::Matx44f Tmw=GLM_Mat4_to_CV_Mat(MarkerDetector::GetTransMatFromRT(rvec,tvec,CV_Matx44f_to_GLM_Mat4(vmat)));
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
                MarkerPose pose;
                pose.markers.emplace_back(0,CV_Matx44f_to_GLM_Mat4(Tmw),rvec,tvec);
                sceneData.setData("MarkerPose",pose);
            }
        }
        return STATE_OK;
    }
};

std::shared_ptr<ARApp> construct_engine() {
    std::string appName = "TestApp"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）

    std::vector<ARModulePtr> modules;
    modules.push_back(createModule<ARInputs>("ARInputs"));
    modules.push_back(createModule<ArucoDetector>("ArucoDetector"));

    auto appData = std::make_shared<AppData>();
    auto sceneData = std::make_shared<SceneData>();

    appData->argc = 1;
    appData->argv = nullptr;
    appData->engineDir = "./AREngine/";  // for test
    appData->dataDir = "./data/";        // for test

    std::shared_ptr<ARApp> app = std::make_shared<ARApp>();
    app->init(appName, appData, sceneData, modules);

    return app;
}

class SceneRecorder:public IScene {
    std::shared_ptr<ARApp> _eng;
    SceneGui gui;
    long long record_start_time{0};
    bool is_recording{false};

public:
    bool initialize(const XrInstance instance,const XrSession session)override{
        _eng=construct_engine();
        _eng->start();


        //============ Init SceneGui =============
        SceneGui::TextItem InfoText;
        InfoText.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.85f,0.6f,-4.f));
        InfoText.scale=glm::vec3{1.f};
        gui.add_text(InfoText,"InfoText");
        SceneGui::ImageItem ControllerImage;
        ControllerImage.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.0f,-0.3f,-4.1f));
        ControllerImage.image=ConvertBGRImageToRGB(cv::imread(Config::AppDataDir+("/Manual/OptManual.png")));
//        gui.add_image(ControllerImage,"ControllerImage");
        SceneGui::ImageItem ManualImage;
        ManualImage.image=ConvertBGRImageToRGB(cv::imread(Config::AppDataDir+("/Manual/OptManual.png")));
//        cv::flip(ControllerImage.image,ControllerImage.image,0); cv::rotate(ControllerImage.image,ControllerImage.image,1); //1=ROTATE_180
        ManualImage.scale={0.29f,0.3f,0.3f};
        //gui.add_image(ManualImage,"ManualImage");

        return true;
    }
    void inputEvent(int leftright, const ApplicationEvent &event)override{
        gui.inputEvent(leftright,event);
        std::stringstream ss;
        ss<<"GetEvent: LR: "<<leftright<<". Event: (thumbstick xy): ";
        ss<<event.thumbstick_x<<", "<<event.thumbstick_y;
        ss<<". Trigger: "<<event.trigger<<", Sqz: "<<event.squeeze;
//        infof(ss.str().c_str());
    }
    void keypadEvent(const std::string &key_name)override{
        static std::map<std::string_view,long long> LastPressedMap; //某个按键最后一次被触发是什么时候,防止反复触发
        static long long MinGap=800; //最小触发间隔为800ms
        if(CurrentMSecsSinceEpoch()-LastPressedMap[key_name]<MinGap) return;
        LastPressedMap[key_name]=CurrentMSecsSinceEpoch();
        if(key_name=="o"){
            is_recording=!is_recording;
            if(is_recording) record_start_time=CurrentMSecsSinceEpoch();
        }
    }
    void update_gui(){
        auto InfoText=gui.get_text_item("InfoText");
        if(InfoText){
            if(!is_recording) InfoText->text="按O键开始录制";
            else InfoText->text=Format("当前已录制 %.1f 秒",((float)(CurrentMSecsSinceEpoch()-record_start_time)/1000.0f));
        }
    }
    virtual void renderFrame(const XrPosef &pose, const glm::mat4 &project, const glm::mat4 &view,int32_t eye){
        update_gui();
        gui.render(project,view,eye);
        if(_eng) {
            {
//                auto [marker_id,marker_pose,rvec,tvec]=get_pose("MarkerPose",0);
            }
        }
        //============================ Recorder ================================
        if(is_recording){ // Record Image
            CameraMutex.lock();
            static long long record_time=0;//CurrentMSecsSinceEpoch();
            static bool isfirst=true;
            static Recorder RR;
            static std::ofstream file,rfile,tfile;
            if(CurrentMSecsSinceEpoch()-record_time>3000){
                infof("--------------------------------recodering")
                if(isfirst){
                    std::string s="Download/"+CurrentDateTime("%Y-%m-%d_%H-%M-%S");
                    RR.set_recorder_save_dir(s);
                    file.open(MakeSdcardPath(s+"/position.txt"));
                    rfile.open(MakeSdcardPath(s+"/rvec.txt"));
                    tfile.open(MakeSdcardPath(s+"/tvec.txt"));
                    RR.start_recording();
                    isfirst=false;
                }
                RR.record_image(CameraImage);
                //write vmat
                std::ostringstream oss;
//                for(int i=0;i<3;++i)
//                    for(int j=0;j<3;++j) oss<<camK(i,j)<<",";
                for(int i=0;i<4;++i)
                    for(int j=0;j<4;++j) oss<<CameraVmat(i,j)<<",";
                std::string mat_str=oss.str();
                if(!mat_str.empty()) mat_str.pop_back();
                file<<mat_str<<std::endl;
                //write rvec & tvec
//                rfile<<rvec<<std::endl;
//                tfile<<tvec<<std::endl;
            }
            CameraMutex.unlock();
        }
        //================= Record Done ==========================
    }
    virtual void close() {
        if(_eng) _eng->stop();
    }
};
}

std::shared_ptr<IScene> _createSceneRecorder() {
    return std::make_shared<SceneRecorder>();
}