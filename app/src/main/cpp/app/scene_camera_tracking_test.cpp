#include"Basic/include/RPC.h"
#include"Basic/include/ARModule.h"
#include"Basic/include/App.h"
#include"opencv2/core.hpp"
#include<thread>
#include<iostream>
#include<opencv2/opencv.hpp>
// 假设 ARModule, AppData, SceneData, FrameDataPtr, STATE_OK 等定义在以下头文件中
#include "InputRealSense/include/InputOffline.h"
#include "InputRealSense/include/InputRealSense.h"
#include "CameraTracking/include/CameraTracking.h"
//#include "Rendering/include/RenderingLocal.h"
//#include "Rendering/include/RenderingClient.h"
#include "Basic/include/listener.h"
#include "Basic/include/ObjManager.h"
#include <filesystem>
#include"config.hpp"
#include"scene.h"
#include"demos/cylinder.h"
#include"markerdetector.hpp"
#include"arengine.h"

using namespace cv;
using namespace std;

std::string PROJECT_PATH=Config::AppDataDir;
#define SERVER_IP   "192.168.31.121"
//"172.25.212.88"        "192.168.31.121"
#include"demos/utils.h"
#include"app/utilsmym.hpp"
inline void infoff(const std::string &s){infof(("============================= "+s).c_str())}


// void SignalHandler(int signum) {
//     std::cout << "Interrupt signal (" << signum << ") received.\n";
//     exit(signum);
// }

namespace{
struct MarkerPose {
    typedef std::tuple<int,glm::mat4,cv::Vec3d,cv::Vec3d> ItemType;
    std::vector<ItemType> markers;
};
cv::Matx44f LatestCameraTrackingReturnMat; //仅做测试

class ArucoDetector: public ARModule {
    std::vector<int> _retFrames;
    std::mutex _retFramesMutex;
public:
    int Init(AppData &appData, SceneData &sceneData, FrameDataPtr frameDataPtr) {
        return STATE_OK;
    }
    int Update(AppData &appData, SceneData &sceneData, FrameDataPtr frameDataPtr) {
        if (!frameDataPtr->image.empty()) {
            cv::Mat img = frameDataPtr->image.front();
            //auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs")); //=====
            //cv::Matx44f vmat=frame_data.cameraMat; //=====
            MarkerDetector detector((cv::Mat)frameDataPtr->colorCameraMatrix);
//            detector.set_dist_coeffs(RokidDistCoeffs); //SceneData中获取的图像是矫正过的,这里就不用设置dist参数了 (2025-07-11)
            auto [flag,rvec,tvec]=detector.detect_aruco(img,cv::aruco::DICT_5X5_50,cv::Size(200,200),-1);
            if(flag){
                MarkerPose pose;
                //pose.markers.emplace_back(0,MarkerDetector::GetTransMatFromRT(rvec,tvec,CV_Matx44f_to_GLM_Mat4(vmat)),rvec,tvec); //=====
                sceneData.setData("MarkerPose",pose);
            }
            else infof("Detect Marker Failed");
            LatestCameraTrackingReturnMat=sceneData.getMainCamera().transform.GetMatrix();
//            std::stringstream ss;
//            ss <<flag<< " PnP: Rotation Vector: " << rvec << ", Translation Vector: " << tvec;
//            infof(ss.str().c_str());
        }
        return STATE_OK;
    }
};

std::shared_ptr<ARApp> construct_load_map_engine(){
    std::string appName="CameraTracking"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）
    // typedef std::shared_ptr<ARModule>  ModulePtr;
    std::vector<ARModulePtr> modules;
    modules.push_back(createModule<InputOffline>("InputOffline"));  //测试离线数据
//    modules.push_back(createModule<ARInputs>("ARInputs"));
    // modules.push_back(std::make_shared<NewModule>());  # NewModule need connect to server
    modules.push_back(createModule<CameraTracking>("CameraTracking"));
    modules.push_back(createModule<ArucoDetector>("ArucoDetector"));
    // modules.push_back(createModule<RenderingLocal>("Rendering"));

    // 初始化 appData 和 sceneData
    auto appData=std::make_shared<AppData>();
    auto sceneData=std::make_shared<SceneData>();
    // Map setting
    appData->isLoadMap=true;
    appData->isSaveMap= !appData->isLoadMap;
    appData->record = true;
    // App setting
    appData->argc = 1;
    appData->argv = nullptr;
    appData->rootDir=std::string(PROJECT_PATH);
    appData->engineDir=std::string(PROJECT_PATH)+"/AREngine/";  // for test
    appData->dataDir=std::string(PROJECT_PATH)+"/AREngine/CameraTracking/";        // for test
    // appData->offlineDataDir = std::string(PROJECT_PATH) + "/data/240925";
    // appData->offlineDataDir = std::string(PROJECT_PATH) + "/data/dataset-00";
    // appData->offlineDataDir = std::string(PROJECT_PATH) + "/data/dataset/dataset-1-6";
    appData->offlineDataDir=std::string(PROJECT_PATH)+"/AREngine/CameraTracking/test_switch_input_pose/data/250106_2";
    std::string camConfigFile=std::string(PROJECT_PATH)+"/AREngine/App/CameraConfigs/Camera_rsD415.json";  // load main camera config
//    std::string camConfigFile=std::string(PROJECT_PATH)+"/AREngine/App/CameraConfigs/Camera_rokid.json";  // load main camera config
    appData->sceneObjConfig=std::string(PROJECT_PATH)+"/AREngine/App/sceneObjects_T01.json";
    if(!std::filesystem::exists(appData->offlineDataDir)){
        infoff(("\033[31mError: offlineDataDir path does not exist: "+appData->offlineDataDir+"\033[0m").c_str());
//        return -1;
    }
    cv::Matx44f FirstFramePose(1,0,0,0.0,0,1,0,2.0,0,0,1,0,0,0,0,1); // object init pose in world coord
    cv::Matx44f ModelInitPose(0.001,0,0,-9.1552734e-07,0,0.001,0,-6.1035156e-08,0,0,0.001,1.5258789e-08,0,0,0,1); // for tube
    cv::Matx44f ModelInit(0.00100000,0.00000000,0.00000000,-0.97854769,0.00000000,0.00100000,0.00000000,1.92165291,0.00000000,0.00000000,0.00100000,-0.06423490,0.00000000,0.00000000,0.00000000,1.00000000);
    cv::Matx44f FirstFrame(1,0,0,0.0,0,1,0,0.0,0,0,1,0.0,0,0,0,1);
    cv::Matx44f TModelPose(0.00100000,0.00000000,0.00000000,-0.02511558,0.00000000,0.00100000,0.00000000,1.89097905,0.00000000,0.00000000,0.00100000,0.08498216,0.00000000,0.00000000,0.00000000,1.00000000);
    {
        SceneObjectPtr obj(new Camera());
        obj->filePath=camConfigFile;
        obj->initTransform=Pose();
        obj->transform=Pose();  // Camera2World
        obj->Init();
        sceneData->setObject("MainCamera",obj);
    }
//    {//加载一个虚拟物体，渲染模块需要根据这些信息进行渲染
//        SceneObjectPtr obj(new VirtualObject);
//        // obj->filePath = appData->dataDir+"norm_model/squirrel_demo_low.obj";
//        obj->filePath = appData->dataDir+"norm_model/Standtube.obj";
//        obj->initTransform.setPose(ModelInitPose);
//        obj->transform.setPose(FirstFramePose);
//        obj->Init();
//        sceneData->setObject("virtualObj1", obj);
//    }
    //===================== In Main Function ========================
//    std::thread listenThread(listenForEvent,std::ref(*appData));
//    ARApp app;
//    app.init(appName,appData,sceneData,modules);
//    infoff("Will connect Server");
//    app.connectServer(SERVER_IP,1203);
//    // app.connectServer("101.76.208.70", 1203);
//    app.run();
//    infoff("App finished Run, Will return");
//    return 0;
    //===============================================================
//    std::thread listenThread(listenForEvent, std::ref(*appData));
    std::shared_ptr<ARApp> app = std::make_shared<ARApp>();
    app->init(appName, appData, sceneData, modules);
//    app->connectServer(SERVER_IP, 1203);
    //app.run();
    return app;
}


std::shared_ptr<ARApp> construct_collect_map_engine() {
    std::string appName="CameraTracking"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）
    // typedef std::shared_ptr<ARModule>  ModulePtr;
    std::vector<ARModulePtr> modules;
//    modules.push_back(createModule<ARInputs>("ARInputs"));
    modules.push_back(createModule<InputOffline>("InputOffline"));   //测试离线数据
    // modules.push_back(std::make_shared<NewModule>());  # NewModule need connect to server
    modules.push_back(createModule<CameraTracking>("CameraTracking"));
    modules.push_back(createModule<ArucoDetector>("ArucoDetector"));
    // modules.push_back(createModule<RenderingLocal>("Rendering"));

    // 初始化 appData 和 sceneData
    auto appData=std::make_shared<AppData>();
    auto sceneData=std::make_shared<SceneData>();
    // Map setting
    appData->isLoadMap=false;
    appData->isSaveMap= !appData->isLoadMap;
    // App setting
    appData->argc = 1;
    appData->argv = nullptr;
    appData->rootDir=std::string(PROJECT_PATH);
    appData->engineDir=std::string(PROJECT_PATH)+"/AREngine/";  // for test
    appData->dataDir=std::string(PROJECT_PATH)+"/AREngine/CameraTracking/";        // for test
    // appData->offlineDataDir = std::string(PROJECT_PATH) + "/data/240925";
    // appData->offlineDataDir = std::string(PROJECT_PATH) + "/data/dataset-00";
    // appData->offlineDataDir = std::string(PROJECT_PATH) + "/data/dataset/dataset-1-6";
    appData->offlineDataDir=std::string(PROJECT_PATH)+"/AREngine/CameraTracking/test_switch_input_pose/data/250106_1";
    std::string camConfigFile=std::string(PROJECT_PATH)+"/AREngine/App/CameraConfigs/Camera_rsD415.json";  // load main camera config
//    std::string camConfigFile=std::string(PROJECT_PATH)+"/AREngine/App/CameraConfigs/Camera_rokid.json";  // load main camera config
    appData->sceneObjConfig=std::string(PROJECT_PATH)+"/AREngine/App/sceneObjects_T01.json";
    if(!std::filesystem::exists(appData->offlineDataDir)){
        infoff(("\033[31mError: offlineDataDir path does not exist: "+appData->offlineDataDir+"\033[0m").c_str());
//        return -1;
    }
    cv::Matx44f FirstFramePose(1,0,0,0.0,0,1,0,2.0,0,0,1,0,0,0,0,1); // object init pose in world coord
    cv::Matx44f ModelInitPose(0.001,0,0,-9.1552734e-07,0,0.001,0,-6.1035156e-08,0,0,0.001,1.5258789e-08,0,0,0,1); // for tube
    cv::Matx44f ModelInit(0.00100000,0.00000000,0.00000000,-0.97854769,0.00000000,0.00100000,0.00000000,1.92165291,0.00000000,0.00000000,0.00100000,-0.06423490,0.00000000,0.00000000,0.00000000,1.00000000);
    {
        SceneObjectPtr obj(new Camera());
        obj->filePath=camConfigFile;
        obj->initTransform=Pose();
        obj->transform=Pose();  // Camera2World
        obj->Init();
        sceneData->setObject("MainCamera",obj);
    }
//    {//加载一个虚拟物体，渲染模块需要根据这些信息进行渲染
//        SceneObjectPtr obj(new VirtualObject);
//        // obj->filePath = appData->dataDir+"norm_model/squirrel_demo_low.obj";
//        obj->filePath = appData->dataDir+"norm_model/Standtube.obj";
//        obj->initTransform.setPose(ModelInitPose);
//        obj->transform.setPose(FirstFramePose);
//        obj->Init();
//        sceneData->setObject("virtualObj1", obj);
//    }
    //===================== In Main Function ========================
//    std::thread listenThread(listenForEvent,std::ref(*appData));
//    ARApp app;
//    app.init(appName,appData,sceneData,modules);
//    infoff("Will connect Server");
//    app.connectServer(SERVER_IP,1203);
//    // app.connectServer("101.76.208.70", 1203);
//    app.run();
//    infoff("App finished Run, Will return");
//    return 0;
    //===============================================================
//    std::thread listenThread(listenForEvent, std::ref(*appData));
    std::shared_ptr<ARApp> app = std::make_shared<ARApp>();
    app->init(appName, appData, sceneData, modules);
//    app->connectServer(SERVER_IP, 1203);
    //app.run();
    return app;
}


class SceneCameraTrackingTest: public IScene{
    std::shared_ptr<ARApp> _eng;
    float mAxleHeight=0.2f; //轮轴高度,单位为m
    Cylinder mAxle{mAxleHeight,0.02f}; //轮轴
    glm::mat4 mAxleMarkerPose{1.0f};
    glm::mat4 mProject{},mView{};
public:
    virtual bool initialize(const XrInstance instance, const XrSession session) {
        _eng=construct_load_map_engine();   //测试load_map
//        _eng=construct_collect_map_engine();  //测试collect_map
        _eng->connectServer(SERVER_IP, 1203);//connectServer("localhost", 123);
        _eng->start();
        //初始化轮轴
        mAxle.initialize();
        return true;
    }
    virtual void close() {
        if (_eng) _eng->stop();
    }


    //@id: -1表示不要求MarkerId,返回列表中的第一个
    MarkerPose::ItemType get_pose(const std::string &name,int id=-1){
        auto _res=_eng->sceneData->getData(name);
        if(_res.has_value()){
            auto res = std::any_cast<MarkerPose>(_res);
            for(const auto &marker:res.markers){
                auto [marker_id,marker_pose,rvec,tvec]=marker;
                //也可以判断一下marker id,看看是不是设置的marker
                if(marker_id!=id&&id!=-1) continue;
                return marker;
            }
        }
        return MarkerPose::ItemType{};
    }
    void renderFrame(const XrPosef &pose, const glm::mat4 &project, const glm::mat4 &view,int32_t eye) {
        mProject=project; mView=view;
        if(_eng) {
            {
                static int __cnt=0;
                static cv::Vec3d _rvec,_tvec;
                auto [marker_id,marker_pose,rvec,tvec]=get_pose("MarkerPose",0);
                if(__cnt<100){
                    _rvec=rvec; _tvec=tvec;
                    _rvec=cv::Vec3d(0.f); _tvec=cv::Vec3d(0.f);
                }
                ++ __cnt;

                auto new_pose=MarkerDetector::GetTransMatFromRT(_rvec,_tvec,CV_Matx44f_to_GLM_Mat4((LatestCameraTrackingReturnMat)));
                mAxleMarkerPose=new_pose;//marker_pose;
                std::stringstream oss;
                oss<<"Original Pose: "<<GlmMat4_to_String(marker_pose)<<"\nNew Pose: "<<GlmMat4_to_String(new_pose)<<"\n=====================\n"<<"CameraTracking Return: "<<GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(LatestCameraTrackingReturnMat));
                infof(oss.str().c_str());
            }
        }
        static glm::mat4 final_mat;
        final_mat=mAxleMarkerPose;
//        final_mat=glm::inverse(final_mat);
        mAxle.render(mProject,mView,final_mat);
//        infof(std::string(GlmMat4_to_String(GlmMat4_from_String(GlmMat4_to_String(final_mat,',',true),','))).c_str());
    }
};

}
std::shared_ptr<IScene> _createSceneCameraTrackingTest() {
    return std::make_shared<SceneCameraTrackingTest>();
}

