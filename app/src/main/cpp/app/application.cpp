#include <dirent.h>
#include "pch.h"
#include "common.h"
#include "options.h"
#include "application.h"
#include "demos/controller.h"
#include "demos/hand.h"
#include <common/xr_linear.h>
#include "logger.h"
#include "demos/gui.h"
#include "demos/ray.h"
#include "demos/text.h"
#include "demos/player.h"
#include "demos/utils.h"
#include "graphicsplugin.h"
#include "demos/cube.h"
#include "app/scene.h"
#include "opencv2/opencv.hpp"
#include "utilsmym.hpp"
#include "scenegui.h"

/*
 * 双击 左/右 方向键切换 上一步/下一步
*/




class Application : public IApplication {
public:
    Application(const std::shared_ptr<struct Options>& options, const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin);
    virtual ~Application() override;
    virtual void setControllerPose(int leftright, const XrPosef& pose) override;
    virtual bool initialize(const XrInstance instance, const XrSession session) override;
    virtual void setHandJointLocation(XrHandJointLocationEXT* location) override;
    virtual void setHandJointWorldLocation(XrHandJointLocationEXT* location) override;
    virtual void inputEvent(int leftright,const ApplicationEvent& event) override;
    virtual void keypadEvent(const std::string &key_name)override;
    virtual void renderFrame(const XrPosef& pose, const glm::mat4& project, const glm::mat4& view, int32_t eye) override;
    virtual void processFrame()override{
        if(m_scene) m_scene->processFrame();
    }
    void exit()override;
    bool needExit()override;
private:
    bool setCurrentScene(const std::string &name){
        if(m_scene){
            m_scene->close();
            m_scene=nullptr;
        }
        auto ptr=createScene(name, this);
        if(ptr && ptr->initialize(m_instance, m_session)){
            m_scene=ptr;
            m_scene_created_timestamp=CurrentMSecsSinceEpoch();
            return true;
        }
        return false;
    }
    //切换scene, step = +1表示切换到sceneList中的下一个scene,返回是否切换成功
    bool switch_scene(int step,bool ignore_timestamp=false){
        int index=m_current_scene+step;
        if(index<0||index>=(int)m_scene_list.size()) return false;
        if(m_scene_list.size()==3&&m_current_scene==1&&step==-1) return false; //----- Only For Debug -----
        if(!ignore_timestamp&&CurrentMSecsSinceEpoch()-m_scene_created_timestamp<1000) return false; //切换Scene有时间限制，1s内最多切换一次
        if(this->setCurrentScene(m_scene_list[index])){
            m_current_scene=index;
            return true;
        }
        else{
            warnf("切换到scene_list中的第%d个scene失败，当前仍处于第%d个scene中",index,m_current_scene)
            return false;
        }
    }
    void step_button_clicked(bool is_next_step_button){
        int step=is_next_step_button?1:-1;
        if(!m_scene) return;
        int result=m_scene->changeStep(step);
        if(result>0) switch_scene(+1);
        if(result<0) switch_scene(-1);
    }
    bool cannot_go_next()const{
        return m_current_scene==(int)m_scene_list.size()-1&&m_scene&&m_scene->changeStep(+1,false)!=0;
    }
    bool cannot_go_prev()const{
        return m_current_scene==0&&m_scene&&m_scene->changeStep(-1,false)!=0;
    }


    void layout();
    void showDashboard(const glm::mat4& project, const glm::mat4& view);
    void showDashboardController();
    void showDeviceInformation(const glm::mat4& project, const glm::mat4& view);
    void renderHandTracking(const glm::mat4& project, const glm::mat4& view);
    // Calculate the angle between the vector v and the plane normal vector n
    float angleBetweenVectorAndPlane(const glm::vec3& vector, const glm::vec3& normal);

private:
    std::shared_ptr<IGraphicsPlugin> mGraphicsPlugin;
    std::shared_ptr<Controller> mController;
    std::shared_ptr<Hand> mHandTracker;
    std::shared_ptr<Gui> mPanel;
    std::shared_ptr<Text> mTextRender;
    std::shared_ptr<Player> mPlayer;
    glm::mat4 mControllerModel;
    XrPosef mControllerPose[HAND_COUNT];
    std::shared_ptr<CubeRender> mCubeRender;

    //openxr
    XrInstance m_instance;          //Keep the same naming as openxr_program.cpp
    XrSession m_session;
    std::vector<XrView> m_views;
    float mIpd;
    XrHandJointLocationEXT m_jointLocations[HAND_COUNT][XR_HAND_JOINT_COUNT_EXT];

    //app data
    std::string mDeviceModel;
    std::string mDeviceOS;

    bool mIsShowDashboard = false;  //不显示Dashboard

    const ApplicationEvent *mControllerEvent[HAND_COUNT];

    std::shared_ptr<IScene>  m_scene;
    std::vector<std::string> m_scene_list;
    int m_current_scene=0;
    long long m_scene_created_timestamp=0; //当前scene是什么时候创建的(unix毫秒时间戳)
    long long m_exit_key_last_pressed_timestamp=0; //退出按键最后一次被按下是什么时候(连按两次退出键才会退出程序)
    bool mExitState{false}; //true表示程序需要退出
};

std::shared_ptr<IApplication> createApplication(IOpenXrProgram *program, const std::shared_ptr<struct Options>& options, const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin) {
    auto ptr=std::make_shared<Application>(options, graphicsPlugin);
    ptr->m_program=program;
    return ptr;
}

Application::Application(const std::shared_ptr<struct Options>& options, const std::shared_ptr<IGraphicsPlugin>& graphicsPlugin) {
    mGraphicsPlugin = graphicsPlugin;
    mController = std::make_shared<Controller>();
    mHandTracker = std::make_shared<Hand>();
    mPanel = std::make_shared<Gui>("dashboard");
    mTextRender = std::make_shared<Text>();
    mPlayer = std::make_shared<Player>();
    mCubeRender = std::make_shared<CubeRender>();
}

Application::~Application() {
}

bool Application::initialize(const XrInstance instance, const XrSession session) {
    m_instance = instance;
    m_session = session;

    // get device model
    mDeviceModel = "Rokid AR Station";

    //get OS version
    //__system_property_get("ro.system.build.id", buffer); // You can also call this function, the result is the same
    mDeviceOS = "OpenXR";

    mController->initialize(mDeviceModel);
    mHandTracker->initialize(); // zhfzhf

    mPanel->initialize(600, 800);  //set resolution
    mTextRender->initialize();
    mCubeRender->initialize();

    const XrGraphicsBindingOpenGLESAndroidKHR *binding = reinterpret_cast<const XrGraphicsBindingOpenGLESAndroidKHR*>(mGraphicsPlugin->GetGraphicsBinding());
    mPlayer->initialize(binding->display);

    m_scene_list={"reloc_test","plane_detector"};
    m_current_scene=0;


    this->setCurrentScene(m_scene_list[m_current_scene]);
//====================== 给 SceneGui::Global() 添加一个不可见的图片，避免渲染文字出现乱码 ==========================
    SceneGui::ImageItem HiddenImage;
    HiddenImage.image=PureColorMat(1,1); HiddenImage.scale={0,0,0};
    SceneGui::Global().add_image(HiddenImage);
//========================= 调试用,将所有标准输出打印到文件中 ===============================
//    std::freopen((Config::AppDataDir+"/log.txt").c_str(),"w",stdout);
    return true;
}


void Application::setControllerPose(int leftright, const XrPosef& pose) {
    XrMatrix4x4f model{};
    XrVector3f scale{1.0f, 1.0f, 1.0f};
    XrMatrix4x4f_CreateTranslationRotationScale(&model, &pose.position, &pose.orientation, &scale);
    glm::mat4 m = glm::make_mat4((float*)&model);
    mController->setModel(leftright, m);
    mHandTracker->setModel(leftright, m); // zhfzhf
    mControllerPose[leftright] = pose;
}
void Application::setHandJointLocation(XrHandJointLocationEXT* location) {
    memcpy(&m_jointLocations, location, sizeof(m_jointLocations));
}
void Application::setHandJointWorldLocation(XrHandJointLocationEXT *location){
    XrHandJointLocationEXT xr_joint_locations[HAND_COUNT][XR_HAND_JOINT_COUNT_EXT];
    memcpy(&xr_joint_locations, location, sizeof(xr_joint_locations));
    for(auto hand=0;hand<HAND_COUNT;hand++){
        std::vector<glm::mat4> joints_pose(XR_HAND_JOINT_COUNT_EXT);
        for(int i=0;i<XR_HAND_JOINT_COUNT_EXT;i++){
            XrHandJointLocationEXT &jointLocation=xr_joint_locations[hand][i];
            if(!(jointLocation.locationFlags&XR_SPACE_LOCATION_POSITION_VALID_BIT&&jointLocation.locationFlags&XR_SPACE_LOCATION_POSITION_TRACKED_BIT)) continue;
            XrMatrix4x4f m{};
            XrVector3f scale{1.0f,1.0f,1.0f};
            XrMatrix4x4f_CreateTranslationRotationScale(&m,&jointLocation.pose.position,&jointLocation.pose.orientation,&scale);
            glm::mat4 model=glm::make_mat4((float *)&m);
            joints_pose[i]=model;

            if(i==0&&hand==HAND_RIGHT){
                glm::vec3 _t=model[3];
//                infof("Gesture Pos: [%f,%f,%f];",_t.x,_t.y,_t.z)
            }
        }
    }
}
void Application::inputEvent(int leftright,const ApplicationEvent& event) {
    mControllerEvent[leftright] = &event;
//    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_menu) {
//        if (event.click_menu) {
//            mIsShowDashboard = !mIsShowDashboard;
//        }
//    }
    if(leftright == HAND_LEFT) return; // *** 这里只处理了右手的手势
    if (event.controllerEventBit & CONTROLLER_EVENT_BIT_click_trigger) {
        infof("controllerEventBit:0x%02x, event.click_trigger:0x%d", event.controllerEventBit, event.click_trigger);
        mPanel->triggerEvent(event.click_trigger);
    }

    SceneGui::Global().inputEvent(leftright,event);
    static constexpr int HAND_CONTROL_INDEX=1,CONTROLLER_CONTROL_INDEX=1; //Controller Ray: 0, Hand Pose: 1 (2025-10-23: 将Controller也修改为1,因为openxr_program中将控制器视为右手)
    int CONTROL_INDEX=(SceneGui::GuiOperationTrigger()==SceneGui::Controller?CONTROLLER_CONTROL_INDEX:HAND_CONTROL_INDEX);
    const XrPosef& controllerPose = mControllerPose[CONTROL_INDEX]; //// ******** ORIGINAL is 1 **********
    glm::vec3 linePoint = glm::make_vec3((float*)&controllerPose.position);
    glm::vec3 lineDirection = mController->getRayDirection(CONTROL_INDEX);
    SceneGui::Global().rayEvent(linePoint,lineDirection);
    if(m_scene){
        m_scene->inputEvent(leftright,event);
        m_scene->rayEvent(linePoint,lineDirection);
    }
}
void Application::keypadEvent(const std::string &key_name){
    if(m_scene&&CurrentMSecsSinceEpoch()-m_scene_created_timestamp>2000) m_scene->keypadEvent(key_name); //场景创建后2000ms后才接受按键事件，否则会导致场景刚创建完成就接收到多个按键事件导致bug
    SceneGui::Global().keypadEvent(key_name);

    static std::map<std::string_view,long long> LastPressedMap; //某个按键最后一次被触发是什么时候,防止反复触发
    static long long MinGap=200; //最小触发间隔为200ms
    if(CurrentMSecsSinceEpoch()-LastPressedMap[key_name]<MinGap) return;
    LastPressedMap[key_name]=CurrentMSecsSinceEpoch();
    infof(("Pressed: "+key_name).c_str());
    if(key_name=="o"){
//        if(SceneGui::GuiOperationTrigger==SceneGui::Controller){
//            SceneGui::SetGuiOperationTrigger(SceneGui::Hand);
//            auto control_text=SceneGui::Global().get_text_item("ControlText");
//            if(control_text) control_text->text="当前控制方式为: 手势操作";
//        }
//        else{
//            SceneGui::SetGuiOperationTrigger(SceneGui::Controller);
//            auto control_text=SceneGui::Global().get_text_item("ControlText");
//            if(control_text) control_text->text="当前控制方式为: 控制器按键操作";
//        }
    }
    else if(key_name=="left"){
        static long long LastPrevStepTimestamp=0,LastLeftButtonTimestamp=0;
        long long click_gap=CurrentMSecsSinceEpoch()-LastLeftButtonTimestamp;
        LastLeftButtonTimestamp=CurrentMSecsSinceEpoch();
        if(click_gap>200&&click_gap<700){
            step_button_clicked(false);
            LastPrevStepTimestamp=CurrentMSecsSinceEpoch();
        }
    }
    else if(key_name=="right"){
        static long long LastNextStepTimestamp=0,LastRightButtonTimestamp=0;
        long long click_gap=CurrentMSecsSinceEpoch()-LastRightButtonTimestamp;
        LastRightButtonTimestamp=CurrentMSecsSinceEpoch();
        if(click_gap>200&&click_gap<700){
            step_button_clicked(true);
            LastNextStepTimestamp=CurrentMSecsSinceEpoch();
        }
    }
    else if(key_name=="x"){ //Exit
        if(CurrentMSecsSinceEpoch()-m_exit_key_last_pressed_timestamp>1000){ //退出按键第一次按下
            m_exit_key_last_pressed_timestamp=CurrentMSecsSinceEpoch();
//            SceneGui::Global().add_text()
        }
        else mExitState=true;
    }
}

void Application::layout() {
    glm::mat4 model = glm::mat4(1.0f);
    float scale = 1.0f;

    float width, height;
    mPanel->getWidthHeight(width, height);
    scale = 0.7;
    model = glm::translate(model, glm::vec3(-0.0f, -0.3f, -1.0f));
    model = glm::rotate(model, glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model, glm::vec3(scale * (width / height), scale, 1.0f));
    mPanel->setModel(model);

    model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(1.0f, -0.0f, -1.5f));
    model = glm::rotate(model, glm::radians(-20.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model, glm::vec3(scale*2, scale, 1.0f));
    mPlayer->setModel(model);
}

void Application::showDashboardController() {
#define HAND_BIT_LEFT HAND_LEFT+1
#define HAND_BIT_RIGHT HAND_RIGHT+1
#define SHOW_CONTROLLER_ROW_float(x)    ImGui::TableNextRow();\
                                        ImGui::TableNextColumn();\
                                        ImGui::Text("%s", MEMBER_NAME(ApplicationEvent, x));\
                                        ImGui::TableNextColumn();\
                                        ImGui::Text("%f", mControllerEvent[HAND_LEFT]->x);\
                                        ImGui::TableNextColumn();\
                                        ImGui::Text("%f", mControllerEvent[HAND_RIGHT]->x);

#define SHOW_CONTROLLER_ROW_bool(hand, x)   ImGui::TableNextRow();\
                                            ImGui::TableNextColumn();\
                                            ImGui::Text("%s", MEMBER_NAME(ApplicationEvent, x));\
                                            ImGui::TableNextColumn();\
                                            if (hand & HAND_BIT_LEFT && mControllerEvent[HAND_LEFT]->x) {\
                                                ImGui::Text("true");\
                                            }\
                                            ImGui::TableNextColumn();\
                                            if (hand & HAND_BIT_RIGHT && mControllerEvent[HAND_RIGHT]->x) {\
                                                ImGui::Text("true");\
                                            }

}

void Application::showDashboard(const glm::mat4& project, const glm::mat4& view) {
    PlayModel playModel = mPlayer->getPlayStyle();
    const XrPosef& controllerPose = mControllerPose[1];
    glm::vec3 linePoint = glm::make_vec3((float*)&controllerPose.position);
    glm::vec3 lineDirection = mController->getRayDirection(1);
    mPanel->isIntersectWithLine(linePoint, lineDirection);
    mPanel->begin();
    if (ImGui::CollapsingHeader("information")) {
        ImGui::BulletText("device model: %s", mDeviceModel.c_str());
        ImGui::BulletText("device OS: %s", mDeviceOS.c_str());
    }
    ImGui::BulletText("device model: %s", mDeviceModel.c_str());
    ImGui::BulletText("device OS: %s", mDeviceOS.c_str());
//    test controller
    showDashboardController();
    ImGui::Text("Please use the hand ray to pinch click.");
    mPanel->end();
    glm::mat4 uiTranslation = glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, -3.0f));
    mPanel->render(project, view,uiTranslation);
    mPlayer->setPlayStyle(playModel);
}

void Application::showDeviceInformation(const glm::mat4& project, const glm::mat4& view) {
    wchar_t text[1024] = {0};
    swprintf(text, 1024, L"model: %s, OS: %s", mDeviceModel.c_str(), mDeviceOS.c_str());

    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(0.5f, -0.6f, -1.0f));
    model = glm::rotate(model, glm::radians(-30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
    model = glm::scale(model, glm::vec3(0.5, 0.5, 1.0f));
    mTextRender->render(project, view, model, text, wcslen(text), glm::vec3(1.0, 1.0, 1.0));
}

float Application::angleBetweenVectorAndPlane(const glm::vec3& vector, const glm::vec3& normal) {
    float dotProduct = glm::dot(vector, normal);
    float lengthVector = glm::length(vector);
    float lengthNormal = glm::length(normal);
    if (lengthNormal != 1.0f) {
        lengthNormal = 1.0f;  //normalnize
    }
    float cosAngle = dotProduct / (lengthVector * lengthNormal);
    float angleRadians = std::acos(cosAngle);
    //Convert radians to degrees
    //float angleInDegrees = glm::degrees(angleRadians);
    return PI/2 - angleRadians;
}

//std::string BoneNames[2][26] {
//        {"p_l_palm", "p_l_wrist",
//            "p_l_thumb0", "p_l_thumb1", "p_l_thumb2", "p_l_thumb3",
//            "p_l_index1", "p_l_index2", "p_l_index3", "p_l_index_null", "p_l_index5",
//            "p_l_middle1", "p_l_middle2", "p_l_middle3", "p_l_middle_null", "p_l_middle5",
//            "p_l_ring1", "p_l_ring2", "p_l_ring3", "p_l_ring_null ", "p_l_ring5",
//            "p_l_pinky0", "p_l_pinky1", "p_l_pinky2", "p_l_pinky3","p_l_pinky_null",
//            },
//
//        {"p_r_palm", "p_r_wrist",
//         "p_r_thumb0", "p_r_thumb1", "p_r_thumb2", "p_r_thumb3",
//         "p_r_index1", "p_r_index2", "p_r_index3", "p_r_index_null", "p_r_index5",
//         "p_r_middle1", "p_r_middle2", "p_r_middle3", "p_r_middle_null", "p_r_middle5",
//         "p_r_ring1", "p_r_ring2", "p_r_ring3", "p_r_ring_null ", "p_r_ring5",
//         "p_r_pinky0", "p_r_pinky1", "p_r_pinky2", "p_r_pinky3","p_r_pinky_null",
//        },
//};

void Application::renderHandTracking(const glm::mat4 &project,const glm::mat4 &view){
    std::vector<CubeRender::Cube> cubes;
    for(auto hand=0;hand<HAND_COUNT;hand++){
        for(int i=0;i<XR_HAND_JOINT_COUNT_EXT;i++){
//            if(i==10) continue;
            XrHandJointLocationEXT &jointLocation=m_jointLocations[hand][i];
            if(jointLocation.locationFlags&XR_SPACE_LOCATION_POSITION_VALID_BIT&&jointLocation.locationFlags&XR_SPACE_LOCATION_POSITION_TRACKED_BIT){
                XrMatrix4x4f m{};
                XrVector3f scale{1.0f,1.0f,1.0f};
                XrMatrix4x4f_CreateTranslationRotationScale(&m,&jointLocation.pose.position,&jointLocation.pose.orientation,&scale);
                glm::mat4 model=glm::make_mat4((float *)&m);
                CubeRender::Cube cube{};
                cube.model=model;
                cube.scale=0.01f;
                cubes.push_back(cube);
//                mHandTracker->setBoneNodeMatrices(hand, getBoneNameByIndex(hand, i), model); // zhfzhf
            }
        }
//        mHandTracker->render(hand, project, view);
    }
    mCubeRender->render(project,view,cubes);
}
void Application::renderFrame(const XrPosef& pose, const glm::mat4& project, const glm::mat4& view, int32_t eye) {
    layout();
//    showDeviceInformation(project, view);
    mPlayer->render(project, view, eye);
    if (mIsShowDashboard) {
        showDashboard(project, view);
    }
    mController->render(project, view);
    renderHandTracking(project, view);

    SceneGui::Global().render(project,view,eye);
    if(m_scene) m_scene->renderFrame(pose,project,view,eye);
}
void Application::exit(){
    if(m_scene) m_scene->close();
}
bool Application::needExit(){
    return mExitState;
}

