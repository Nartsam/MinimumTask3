#pragma once

#include"app/application.h"
#include<string>

class IScene{
public:
    IOpenXrProgram *m_program=nullptr;
    IApplication *m_app=nullptr;
public:
    virtual ~IScene() = default;
    virtual bool initialize(const XrInstance instance,const XrSession session)=0;
    virtual void inputEvent(int leftright, const ApplicationEvent& event){}
    virtual void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection){}; //处理控制射线(手部射线或控制器射线)
    virtual void keypadEvent(const std::string &key_name){} //@key_name 表示哪个按键被按下了，其名称在openxr_program.cpp的KeypadCheckList中有定义。只要按键被按下就会一直不停的触发这个函数
    virtual void renderFrame(const XrPosef& pose, const glm::mat4& project, const glm::mat4& view, int32_t eye){}
    virtual void processFrame(){}
    virtual int  changeStep(int step,bool real_change=true){return step;} //需要切换到@step步之后，返回值为0表示切换成功，返回值不为0表示无法在当前场景中切换到@step步之后，返回值的正负性表示需要切换到下一个还是上一个scene中
    virtual void close(){} //退出Scene
};


std::shared_ptr<IScene> createScene(const std::string &name, IApplication *app);

