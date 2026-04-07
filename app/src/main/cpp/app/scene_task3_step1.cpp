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
#include"posemotion.hpp"
#include"SceneGui.h"
#include"modelmanager.hpp"
#include"humandetector.h"
#include"databank.hpp"

using namespace cv;

/*
 * 检测 Marker，根据其位置摆放模型，并且在用户确认后进入手动调整模式
*/

namespace {
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
    bool _isStatic=false;
    int _nSum=0;
    cv::Vec3f _mrvec,_mtvec;
    bool _use_opencv_detector=false; //2025-10-13 控制是使用原始的opencv检测Marker,还是用更新的检测代码(正常为false)
public:
    int Init(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        _detector.loadTemplate(Config::AppDataDir+("/templ_1.json"));
        return STATE_OK;
    }
    int Update(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        if(_use_opencv_detector){
            if (!frameDataPtr->image.empty()) {
                cv::Mat img = frameDataPtr->image.front();
                auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs"));
                cv::Matx44f vmat=frame_data.cameraMat;
                MarkerDetector detector((cv::Mat)frameDataPtr->colorCameraMatrix);
//              detector.set_dist_coeffs(RokidDistCoeffs); //SceneData中获取的图像是矫正过的,这里就不用设置dist参数了 (2025-07-11)
                auto [flag1,rvec1,tvec1]=detector.detect_aruco(img,cv::aruco::DICT_5X5_50,cv::Size(68,68),0);
//              auto [flag2,rvec2,tvec2]=detector.detect_aruco(img,cv::aruco::DICT_5X5_50,cv::Size(180,180),17);
                if(flag1){
                    MarkerPose pose;
                    pose.markers.emplace_back(0,MarkerDetector::GetTransMatFromRT(rvec1,tvec1,CV_Matx44f_to_GLM_Mat4(vmat)),rvec1,tvec1);
                    sceneData.setData("MarkerPose",pose);
                }
                else infof("Detect Chessboard Failed");
            }
            return STATE_OK;
        }
        else{
            if(!frameDataPtr->image.empty()){
                cv::Mat img=frameDataPtr->image.front();
                auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs"));
                cv::Matx44f vmat=frame_data.cameraMat;
                const cv::Matx33f &camK=frameDataPtr->colorCameraMatrix;
                //=============== HumanDetector Test ================
                DataBank::Global().set("vmat",vmat);
                if(!HumanDetector::Global().camera_mat_valid()) HumanDetector::Global().set_camera_mat(camK);
                if(!HumanDetector::Global().server_started()) HumanDetector::Global().start_server(Config::global().get("HumanDetectorServerIp"),std::stoi(Config::global().get("HumanDetectorServerPort")));
                HumanDetector::Global().detect(img);
                //===================================================
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

class SceneTask3Step1:public IScene {
    std::shared_ptr<ARApp> _eng;
    SceneGui gui;

    glm::mat4 mMarkerPose{1.0f};
    float mAxleHeight=1.5f; //轮轴高度,单位为m
    Cylinder mAxle{mAxleHeight,0.02f}; //轮轴
    glm::mat4 mAxleAdjust=glm::mat4(1.0f);
    glm::mat4 mAxleRenderPose{};
    glm::mat4 mManualPanelPose{}; //装配说明书的显示位置
    Model mTestModel{""};

    glm::mat4 mProject{},mView{};
    int mStep=0; //当前进行到了第几步,其范围等于 @mStepDescription 这个列表的长度
    std::vector<std::string> mStepDescription{      //描述一下每一步都在干什么,该vector的size()决定了一共有多少步
        "当前正在定位轮轴\n您可以使用控制器方向键手动调整模型位置\n调整完成后，请进入下个步骤观看装配动画",
        "正在展示装配动画，可根据演示进行操作",
    };

public:

    bool initialize(const XrInstance instance,const XrSession session)override{
        _eng=construct_engine();
        //初始化模型   part1: tong, part2: gai zi, part3: luo mao
//        ModelManager::Manager().add_model_from_file("step1_MyModel",Config::AppDataDir+("/Model/Mars/mars.obj"),true);
//        ModelManager::Manager().add_model_from_file("step1_MyModel2",Config::AppDataDir+("/Model/zfmodel/part3/model.obj"),true);
//        ModelManager::Manager().add_model_from_file("step1_AniModel",Config::AppDataDir+("/Model/test_gltf/test_all.gltf"),true);
        ModelManager::Manager().set_model_adjust("step1_MyModel",glm::translate(glm::mat4(1.0f),{0.f,0.f,0.3f}));
//        ModelManager::Manager().set_model_adjust("step1_AniModel",glm::translate(glm::mat4(1.0f),{-0.4f,-0.1f,0.6f}));
        ModelManager::Manager().set_model_scale("step1_AniModel",{2.0f,2.0f,2.0f});
//        ModelManager::Manager().add_model_from_file("step1_axle",Config::AppDataDir+("/Model/Axle/axle.obj"),false);
        ModelManager::Manager().set_model_scale("step1_axle",{4.f,4.f,4.f});
        mTestModel=Model::GetModelFromLocalFile("TestModel1",Config::AppDataDir+("/Model/Mars/mars.obj"));

        //初始化轮轴
        mAxle.initialize();
        mAxle.setTexture(MatToTexture(cv::imread(Config::AppDataDir+("/Image/Marble.png"))));
        _eng->start();

        mAxleAdjust=GlmMat4_from_String(Config::global().get("AxleAdjust"),',',mAxleAdjust);
        infof(("Axle Adjust:"+GlmMat4_to_String(mAxleAdjust)).c_str());

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
        gui.add_image(ManualImage,"ManualImage");


        mStep=0; switch_step(+1); //进入第一步,应该放置在初始化完成之后
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
    void step_finished(){ //当前step结束,看看有没有什么需要处理的
        if(mStep==1){
            Config::global().set("AxleAdjust",GlmMat4_to_String(mAxleAdjust,',',true));
            Config::global().save();
        }
        else if(mStep==2){
        }
    }
    void step_start(){ //进入当前step,看看有没有什么需要处理的
        //控制部分组件的可见性
        auto controller_image=gui.get_image_item("ControllerImage"),manual_image=gui.get_image_item("ManualImage");
        if(controller_image) controller_image->visible=mStep==1;
        if(manual_image) manual_image->visible=mStep==2;
        //提示文本的修改
        auto text_item=gui.get_text_item("InfoText");
        if(text_item) text_item->text=mStepDescription[mStep-1];

        if(mStep==1){
//            mAxleAdjust=glm::mat4(1.0f);
        }
        else if(mStep==2){
            //把说明书的显示位置设置为轮轴位置的偏移
            if(manual_image) manual_image->translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(mAxleRenderPose[3])+glm::vec3(-0.3f,0.2f,-0.5f));
        }
    }
    int changeStep(int step,bool real_change)override{
        if(mStep+step>(int)mStepDescription.size()) return 1;
        if(mStep+step<1) return -1;
        if(real_change) switch_step(step);
        return 0;
    }
    bool switch_step(int step){
        static long long LastSwitchTimestamp=0;
        if(CurrentMSecsSinceEpoch()-LastSwitchTimestamp<800) return false;
        LastSwitchTimestamp=CurrentMSecsSinceEpoch();
        int new_step=this->mStep+step;
        if(new_step>(int)mStepDescription.size()||new_step<1) return false;
        step_finished();
        this->mStep=new_step;
        step_start();
        return true;
    }
    void keypadEvent(const std::string &key_name)override{
        static std::map<std::string_view,long long> LastPressedMap; //某个按键最后一次被触发是什么时候,防止反复触发
        static long long MinGap=200; //最小触发间隔为200ms
        if(CurrentMSecsSinceEpoch()-LastPressedMap[key_name]<MinGap) return;
        LastPressedMap[key_name]=CurrentMSecsSinceEpoch();

        if(mStep==1){
            float shift=0.01f;  //每一步平移的幅度
            if(key_name=="up")          model_move_shift(mAxleAdjust,{0,-shift,0});
            else if(key_name=="down")   model_move_shift(mAxleAdjust,{0,shift,0});
            else if(key_name=="left")   model_move_shift(mAxleAdjust,{-shift,0,0});
            else if(key_name=="right")  model_move_shift(mAxleAdjust,{shift,0,0});
//            else warnf(("Unknown Keypad name: "+key_name).c_str());
        }
        else if(mStep==2){
            if(key_name=="select"){ //切换动画
                auto model=ModelManager::Manager().get_model("step1_AniModel");
                if(model&&model->is_valid()){
                    auto playing_list=model->playing_animations_list();
                    int max_index=-1;
                    for(int i:playing_list) model->stop_animation(i),max_index=std::max(max_index,i);
                    model->start_animation((max_index+1)%model->animations_count());
                }
            }
        }
    }
    // @shift 的三个参数分别为在 x,y,z 轴方向上的平移幅度
    void model_move_shift(glm::mat4 &model,const glm::vec3 &shift){
        model=glm::translate(model,shift);
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
//    void setup_model_motion(){
//        static bool is_first=true;
//        if(!is_first) return;
//        is_first=false;
//        int cnt=0;
//        for(ModelItem &item:mModelList){
//            if(++cnt==1){
//                item.adjust=glm::rotate(item.adjust,glm::radians(90.0f),glm::vec3(1.f, 0.f, 0.f));
//                item.scale=glm::vec3{2.0f};
//                item.motion.set_start_pose(mAxleRenderPose);
//                item.motion.set_end_pose(glm::rotate(item.motion.start_pose(),glm::radians(180.0f),glm::vec3(0.f, 0.f, 1.f)));
//                item.motion.set_duration(3.3);
//                item.motion.set_motion_mode(PoseMotion::PingPong);
//                item.motion.start();
//            }
//            else{
//                item.scale=glm::vec3{2.0f};
//                item.motion.set_start_pose(mAxleRenderPose);
//                item.motion.set_end_pose(glm::translate(item.motion.start_pose(),{0.f, 0.f, 0.7f}));
////                item.motion.set_end_pose(glm::rotate(item.motion.start_pose(),glm::radians(-180.0f),glm::vec3(1.f, 0.f, 0.f)));
//                item.motion.set_duration(2.0);
//                item.motion.set_motion_mode(PoseMotion::Loop);
//                item.motion.start();
//            }
//        }
//    }
//    void model_render(){
//        static bool is_first=true;
//        if(is_first){
//            setup_model_motion();
//            mManualPanelPose=glm::translate(mAxleRenderPose,glm::vec3(-0.0f,-0.6f,4.3f));
//            auto manual_image=gui.get_image_item("ManualImage");
//            if(manual_image) manual_image->translate_model=mManualPanelPose;
//        }
//    }
    void model_render(){
        int cnt=0;
        auto model_name_list=ModelManager::Manager().model_name_list("step1_");
        for(const auto &model_name:model_name_list){
            auto state_code=ModelManager::Manager().get_model_state(model_name);
            if(state_code==ModelManager::Ready) ModelManager::Manager().render_model(model_name,mProject,mView,mMarkerPose);
            else if(state_code==ModelManager::Loading){ //模型仍在加载
                SceneGui::ShowToast("模型正在加载中，请稍等...",mProject,glm::mat4(1.0f),glm::scale(glm::translate(glm::mat4(1.0f),{-0.0f,-0.2f,-2.0f}),{.5f,.5f,.5f}));
            }
            else if(state_code==ModelManager::WaitSetup){
                ModelManager::Manager().make_model_valid(model_name);
                ModelManager::Manager().start_model_animation(model_name,0);
            }
            else{ //模型加载完成但加载失败了
                errorf("Load Model: %s Scene Failed with Code %d",model_name.c_str(),state_code);
            }
        }
    }
    virtual void renderFrame(const XrPosef &pose, const glm::mat4 &project, const glm::mat4 &view,int32_t eye) {
        mProject=project; mView=view;
        gui.render(project,view,eye);
        if(_eng) {
            {
                auto [marker_id,marker_pose,rvec,tvec]=get_pose("MarkerPose",0);
                if(mStep==1||mStep==2||mStep==3) mMarkerPose=marker_pose;
            }
        }
        mMarkerPose=glm::translate(glm::mat4(1.0f),{-0.3f,-0.9f,-2.0f});
        if(mStep>=1&&mStep<2){
            mAxleRenderPose=mMarkerPose*mAxleAdjust;
            mAxle.render(mProject,mView,mAxleRenderPose);
            infof(std::string(GlmMat4_to_String(GlmMat4_from_String(GlmMat4_to_String(mAxleRenderPose,',',true),','))).c_str())
        }
        mTestModel.render(mProject,mView,glm::scale(mMarkerPose,{0.01,0.01,0.01}));
        if(mStep>=1) model_render(); //2
//        HumanDetector::Global().render(mProject,mView);
    }

    virtual void close() {
        Config::global().set("AxleAdjust",GlmMat4_to_String(mAxleAdjust,',',true));
        Config::global().save();
        if(_eng) _eng->stop();
    }
};
}

std::shared_ptr<IScene> _createSceneTask3Step1() {
    return std::make_shared<SceneTask3Step1>();
}