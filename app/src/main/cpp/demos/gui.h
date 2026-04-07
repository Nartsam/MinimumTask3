#pragma once
#include "guiBase.h"

class Gui {
public:
    Gui(std::string name);
    ~Gui();
    bool initialize(int32_t width, int32_t height);
    void render(const glm::mat4& p, const glm::mat4& v);
    void render(const glm::mat4& p, const glm::mat4& v,const glm::mat4& fix_on_view);

    void setModel(const glm::mat4& m);
    void getWidthHeight(float& width, float& height);
    //2025-09-08 添加@update_event参数，可以控制是否要调用updateMousePosition()将事件发送给ImGui
    bool isIntersectWithLine(const glm::vec3& linePoint, const glm::vec3& lineDirection,bool update_event=true);
    void active();
    void begin();
    void end();
    void triggerEvent(bool down);
    void setGuiFlags(ImGuiWindowFlags flags);

private:
    bool initShader();
    void updateMousePosition(float x, float y);

private:
    static Shader mShader;
    std::string mName;

    GLuint mFramebuffer;
    GLuint mTextureColorbuffer;
    GLuint mVAO;
    GLuint mVBO;

    int32_t mWidth;
    int32_t mHeight;

    glm::mat4 mModel;
    glm::vec3 mIntersectionPoint;

    ImGuiWindowFlags mGuiFlags=0; // Add on 2025-08-13
};