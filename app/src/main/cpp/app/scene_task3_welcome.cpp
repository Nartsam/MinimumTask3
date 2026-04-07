#include"Basic/include/RPC.h"
#include"Basic/include/ARModule.h"
#include"Basic/include/App.h"
#include"opencv2/core.hpp"
#include"arengine.h"
#include"demos/gui.h"
#include"scene.h"
#include"demos/utils.h"
#include"glm/gtc/matrix_transform.hpp"
#include"demos/text.h"
#include"scenegui.h"
#include"utilsmym.hpp"

using namespace cv;

/*
 * 用于在软件运行时显示一个初始界面
*/

namespace {

class SceneTask3Welcome:public IScene {
    std::shared_ptr<ARApp> _eng;

    SceneGui gui;
    glm::vec3 mTextScale=glm::vec3(0.5,0.2,0.5f);

    glm::mat4 mProject{},mView{};
public:
    bool initialize(const XrInstance instance,const XrSession session)override{
//        _eng = construct_engine();
        SceneGui::TextItem   text,main_text;

        glm::mat4 model = glm::mat4(1.0f);
        float scale = 0.5f,width=120, height=80;
        model = glm::translate(model, glm::vec3(-0.0f, -0.3f, 5.0f));
        model = glm::rotate(model, glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::scale(model, glm::vec3(scale * (width / height), scale, 1.0f));

        text.text="欢迎使用本软件, 按O键进入初始化部分";
        text.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.44f,0.4f,-3.0f));
        text.scale=glm::vec3{0.9f};
        text.use_view=false;
        main_text.text="AR机轮装配引导系统";
        main_text.translate_model=glm::translate(glm::translate(glm::mat4(1.0f),glm::vec3(-0.0f,0.1f,-4.0f)),{-0.66f,-0.5f,0.f});
        main_text.scale={1.6f,1.6f,1.6f};

//        gui.add_text(text);
        gui.add_text(main_text);

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
    }

    virtual void close() {
        if(_eng) _eng->stop();
    }
};
}

std::shared_ptr<IScene> _createSceneTask3Welcome() {
    return std::make_shared<SceneTask3Welcome>();
}