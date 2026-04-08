#ifndef ROKIDOPENXRANDROIDDEMO_SCENEGUI_H
#define ROKIDOPENXRANDROIDDEMO_SCENEGUI_H

// 显示按钮、提示文字 等基本界面

#include"demos/gui.h"
#include"demos/text.h"
#include"opencv2/core.hpp"
#include"application.h"
#include<unordered_map>


class SceneGui{
private:
    struct ItemBase{ //ButtonItem,ImageItem 等等的基类
        glm::vec3 scale{1.0f};            //组件在x,y,z轴上的缩放
        glm::mat4 translate_model{1.0f};     //模型变换矩阵
        bool use_view{true};                    //为true则组件在空间中的位置固定，否则跟随视角移动
        bool visible{true};                     //组件是否可见
    };
public:
    struct ButtonItem:ItemBase{
        int width,height; //按钮尺寸
        std::string text{"Button"};  //显示在按钮上的文本
        cv::Mat image; //若image非空则显示image,否则显示text。图像大小需要提前缩放至Button大小(width,height)
        bool covered{false};
        std::function<void(void)> call;
    };
    struct TextItem:ItemBase{
        std::string text;
        glm::vec3 color{1.0f,1.0f,1.0f}; //默认为白色
        float line_gap{0.11f}; //行间距,单位为m
    };
    struct ImageItem:ItemBase{
        cv::Mat image; //需要保证image格式是RGB而不是BGR
    };
    enum OperationTrigger{ //射线的控制方式
        Hand,Controller
    };

private:
    //设置默认的GUI组件控制方式。另外,如果要改变射线的控制方式,还需在openxr_program.cpp中开启或关闭 USE_HAND_AIM 宏
    inline static OperationTrigger GuiOperationTriggerValue=OperationTrigger::Controller;
public:
    static OperationTrigger GuiOperationTrigger();
    static void SetGuiOperationTrigger(OperationTrigger trigger);
    static SceneGui& Global();
    SceneGui();
    ~SceneGui();


    void render(const glm::mat4 &p,const glm::mat4 &v,int32_t eye);
    void render_by_id(const std::string &id,const glm::mat4 &p,const glm::mat4 &v,int32_t eye);
    bool set_translate_model(const std::string &id,const glm::mat4 &m);
    glm::mat4 get_translate_model(const std::string &id);
    bool set_button_covered(const std::string &id,bool covered);


    bool add_button(const ButtonItem &item,const std::string &id={});  //往渲染列表里添加一个按钮.最后的string表示该Item的id,若不为空,则不能与其它的id重复(包括text和image)
    bool add_text(const TextItem &item,const std::string &id={});      //往渲染列表里添加一段文字.最后的string表示该Item的id,若不为空,则不能与其它的id重复(包括button和image)
    bool add_image(const ImageItem &item,const std::string &id={});    //往渲染列表里添加一张图片.最后的string表示该Item的id,若不为空,则不能与其它的id重复(包括text和button)

    void inputEvent(int leftright,const ApplicationEvent& event);
    void keypadEvent(const std::string &key_name);
    void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection);

    bool erase_gui_item(const std::string &id); //根据id删除已经添加过的 text/button/image

    TextItem*   get_text_item(const std::string &id);
    ImageItem*  get_image_item(const std::string &id);
    ButtonItem* get_button_item(const std::string &id);

    static void ShowToast(const std::string &text,const glm::mat4 &p,const glm::mat4 &v,const glm::mat4 &model);

private:
    void render_button(const glm::mat4 &p,const glm::mat4 &v,int32_t eye,const std::string &render_id={});
    void render_text(const glm::mat4 &p,const glm::mat4 &v,int32_t eye,const std::string &render_id={});
    void render_image(const glm::mat4 &p,const glm::mat4 &v,int32_t eye,const std::string &render_id={});
public:
    static glm::mat4 no_view_mat(int32_t eye);
private:
    static std::string random_unique_string(); //生成一个随机且唯一的字符串

    bool check_id_unique(const std::string &id);

    std::vector<std::tuple<ButtonItem,std::shared_ptr<Gui>,GLuint,std::string>>     mButtonList; //最后的string表示该Item的id,若不为空,则不能与其它的id重复
    std::vector<std::tuple<TextItem,Text,std::string>>                              mTextList;
    std::vector<std::tuple<ImageItem,std::shared_ptr<Gui>,GLuint,std::string>>      mImageList;  //GLuint表示图像的TextureID
    static constexpr int BUTTON_ID_INDEX=3,TEXT_ID_INDEX=2,IMAGE_ID_INDEX=3;

    glm::vec3 mLinePoint,mLineDirection; //保存射线位置,判断按钮是否被选择
    long long mLastTriggerTimestamp{0}; //最后一次点击的时间戳
    long long mLastPressedTimestamp{0}; //最后一次按下的时间戳
    std::unordered_map<std::string,long long> mControllerButtonLastPressedTimestamp;
};


/*
 *  Left <-     x      -> Right
 *  Down <-     y      -> Up
*/


#endif //ROKIDOPENXRANDROIDDEMO_SCENEGUI_H