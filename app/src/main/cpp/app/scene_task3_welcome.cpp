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
#include"markerdetector.hpp"
#include"scenegui.h"
#include"utilsmym.hpp"
#include"config.hpp"

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
        SceneGui::ButtonItem button;
        SceneGui::TextItem   text,main_text;
        SceneGui::ImageItem   image;

        glm::mat4 model = glm::mat4(1.0f);
        float scale = 0.5f,width=120, height=80;
        model = glm::translate(model, glm::vec3(-0.0f, -0.3f, 5.0f));
        model = glm::rotate(model, glm::radians(10.0f), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::scale(model, glm::vec3(scale * (width / height), scale, 1.0f));
        button.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.5f,-0.8f,-2.2f));//model;
        button.text="Test Button";
        button.width=width; button.height=height; button.scale=glm::vec3{0.3f};
        button.call=[=](){infof("Button Clicked, Scale: %.f",scale);};
        button.image=ConvertBGRImageToRGB(cv::imread(Config::AppDataDir+("/Manual/ImageButton.png")));
        button.width=button.image.cols; button.height=button.image.rows;
        cv::resize(button.image,button.image,{button.width,button.height});

        text.text="欢迎使用本软件, 按O键进入初始化部分";
        text.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.44f,0.4f,-3.0f));
        text.scale=glm::vec3{0.9f};
        text.use_view=false;
//        image.use_view=false;
        image.translate_model=glm::translate(glm::mat4(1.0f),glm::vec3(-0.0f,0.1f,-4.0f));
        image.image=ConvertBGRImageToRGB(cv::imread(Config::AppDataDir+("/Image/Logo.png")));
        image.scale={0.3f,0.3f,0.3f};
//        cv::resize(image.image,image.image,{},0.6,0.6);
        main_text.text="AR机轮装配引导系统";
        main_text.translate_model=glm::translate(image.translate_model,{-0.66f,-0.5f,0.f});
        main_text.scale={1.6f,1.6f,1.6f};



//        gui.add_button(button);
//        gui.add_text(text);
        gui.add_image(image);
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