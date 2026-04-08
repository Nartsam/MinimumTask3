#pragma once
#include "shader.h"
#include "glm/glm.hpp"
#include "common/gfxwrapper_opengl.h"

// 用于绘制一个水平半透明平面，位置在地面（眼镜高度减去用户身高处）
class PlaneRender {
public:
    PlaneRender();
    ~PlaneRender();

    void initialize();
    // 渲染平面；若 hasIntersection 为 true，则在 intersectionPoint 处显示白色圆点
    void render(const glm::mat4& p, const glm::mat4& v, const glm::mat4& m,
                bool hasIntersection = false,
                const glm::vec3& intersectionPoint = glm::vec3(1e6f, 1e6f, 1e6f));

    void setColor(const glm::vec3& color);
    void setAlpha(float alpha);
    void setHalfExtent(float halfExtent); // 设置平面半边长（米）

    float getHalfExtent() const { return mHalfExtent; }

private:
    bool initShader();

private:
    static Shader mShader; // 平面 shader（含相交点圆点绘制）

    GLuint mVAO = 0;

    glm::vec3 mColor{0.2f, 0.6f, 1.0f}; // 平面颜色（蓝色调）
    float mAlpha = 0.4f;                  // 透明度
    float mHalfExtent = 3.0f;             // 平面半边长，单位：米
};
