#ifndef ROKIDOPENXRANDROIDDEMO_CYLINDER_H
#define ROKIDOPENXRANDROIDDEMO_CYLINDER_H

#pragma once

#include <vector>
#include <glm/glm.hpp>
#include "Shader.h"


// 用于绘制一个柱状物体，这不是 RokidOpenXRDemo 中自带的部分
class Cylinder {
public:
    explicit Cylinder(float height=0.1f,float radius=0.05f);
    ~Cylinder();

    void initialize();
    void render(const glm::mat4& p, const glm::mat4& v, const glm::mat4& m);

    void setHeight(float height);      // <--- 动态设置高度
    void setTexture(GLuint texID);     // <--- 设置外部加载好的纹理ID

    glm::vec3 getDirectionVector(const glm::mat4& m);
    void setColor(const glm::vec3& color);
    void setColor(float x, float y, float z);

    std::vector<glm::vec3> getPoints();

private:
    static Shader mShader;
    bool initShader();

    GLuint mVAO = 0;
    GLuint mTexture = 0;               // <--- 纹理ID
    std::vector<float> mVertices;      // 每个顶点: x y z u v
    std::vector<GLuint> mIndices;
    glm::vec3 mColor{};

    float mRadius = 0.05f;
    float mHeight = 0.1f;
    int mSegments = 36;  // 横切面细分段数
    glm::vec3 mBottomCenter{}, mTopCenter{};
};

#endif //ROKIDOPENXRANDROIDDEMO_CYLINDER_H
