#include "plane.h"
#include "utils.h"
#include "glm/gtc/matrix_transform.hpp"

Shader PlaneRender::mShader;

PlaneRender::PlaneRender() {
}

PlaneRender::~PlaneRender() {
}

bool PlaneRender::initShader() {
    static bool init = false;
    if (init) {
        return true;
    }

    // 顶点着色器：将平面顶点变换到裁剪空间，传出世界坐标用于相交点圆点判断
    const GLchar* vertex_shader_glsl = R"_(
        #version 320 es
        precision highp float;
        layout (location = 0) in vec3 position;
        uniform mat4 projection;
        uniform mat4 view;
        uniform mat4 model;
        out vec3 FragPos;
        void main()
        {
            FragPos = vec3(model * vec4(position, 1.0));
            gl_Position = projection * view * vec4(FragPos, 1.0);
        }
    )_";

    // 片元着色器：半透明平面，在相交点附近绘制白色不透明圆点
    const GLchar* fragment_shader_glsl = R"_(
        #version 320 es
        precision mediump float;
        in vec3 FragPos;
        out vec4 FragColor;
        uniform vec4 planeColor;        // rgba，a 为透明度
        uniform vec3 intersectionPoint; // 相交点（世界坐标），无相交时传远处值
        uniform float dotRadius;        // 相交点圆点半径（米）
        void main()
        {
            float dx = FragPos.x - intersectionPoint.x;
            float dy = FragPos.y - intersectionPoint.y;
            float dz = FragPos.z - intersectionPoint.z;
            float dist2 = dx*dx + dy*dy + dz*dz;
            if (dist2 < dotRadius * dotRadius) {
                FragColor = vec4(1.0, 1.0, 1.0, 1.0); // 相交点：白色圆点
            } else {
                FragColor = planeColor;
            }
        }
    )_";

    if (!mShader.loadShader(vertex_shader_glsl, fragment_shader_glsl)) {
        return false;
    }
    init = true;
    return true;
}

void PlaneRender::initialize() {
    if (!initShader()) {
        return;
    }

    // 平面顶点在局部空间以 [-1,1] 为范围（XZ 平面，Y=0 朝上），
    // 通过外部传入的 model 矩阵进行缩放和位移，实现可调大小和位置。
    const float vertices[] = {
        -1.0f, 0.0f, -1.0f,
         1.0f, 0.0f, -1.0f,
         1.0f, 0.0f,  1.0f,
        -1.0f, 0.0f,  1.0f,
    };
    const GLuint indices[] = {
        0, 1, 2,
        0, 2, 3,
    };

    GLuint VBO = 0, EBO = 0;
    GL_CALL(glGenVertexArrays(1, &mVAO));
    GL_CALL(glGenBuffers(1, &VBO));
    GL_CALL(glGenBuffers(1, &EBO));

    GL_CALL(glBindVertexArray(mVAO));
    GL_CALL(glBindBuffer(GL_ARRAY_BUFFER, VBO));
    GL_CALL(glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW));
    GL_CALL(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO));
    GL_CALL(glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW));

    GL_CALL(glEnableVertexAttribArray(0));
    GL_CALL(glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0));

    GL_CALL(glBindVertexArray(0));
}

void PlaneRender::render(const glm::mat4& p, const glm::mat4& v, const glm::mat4& m,
                         bool hasIntersection, const glm::vec3& intersectionPoint) {
    mShader.use();
    mShader.setUniformMat4("projection", p);
    mShader.setUniformMat4("view", v);
    mShader.setUniformMat4("model", m);
    mShader.setUniformVec4("planeColor", glm::vec4(mColor, mAlpha));
    // 无相交时将相交点设置到极远处，确保圆点不可见
    mShader.setUniformVec3("intersectionPoint",
        hasIntersection ? intersectionPoint : glm::vec3(1e6f, 1e6f, 1e6f));
    mShader.setUniformFloat("dotRadius", 0.05f); // 圆点半径 5cm

    GL_CALL(glDisable(GL_CULL_FACE));   // 双面渲染，从任意角度都能看到平面
    GL_CALL(glEnable(GL_BLEND));
    GL_CALL(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    GL_CALL(glEnable(GL_DEPTH_TEST));

    GL_CALL(glBindVertexArray(mVAO));
    GL_CALL(glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0));
    GL_CALL(glBindVertexArray(0));
}

void PlaneRender::setColor(const glm::vec3& color) {
    mColor = color;
}

void PlaneRender::setAlpha(float alpha) {
    mAlpha = alpha;
}

void PlaneRender::setHalfExtent(float halfExtent) {
    mHalfExtent = halfExtent;
}
