#pragma once

#include<string>
#include<map>
#include<vector>
#include<memory>
#include<unordered_map>
#include"mesh.h"
#include"assimp/Importer.hpp"
#include"assimp/scene.h"


class AniMesh{
public:
    AniMesh(std::vector<Vertex> vertices,std::vector<unsigned int> indices,std::vector<Texture> textures);
    void draw(Shader &shader);
    bool activeTexture(const std::string &textureName);
    void setupMesh();
//private:
    glm::mat4 mModelMatrix{1.0f}; // 新增: 每个mesh的模型矩阵
    std::vector<Vertex> mVertices;
    std::vector<unsigned int> mIndices;
    std::vector<Texture> mTextures;
    unsigned int mFramebuffer;
    unsigned int mVAO;
    unsigned int mVBO;
    unsigned int mEBO;
};


/*
 * 不支持无参构造和拷贝，在需要拷贝的场景可以使用指针代替
*/

class AniModel{
public:
    AniModel()=delete;
    AniModel(const std::string &name,bool hasBoneInfo=false);
    ~AniModel();
    std::string &name();

    //为了支持在另一个线程中加载，添加了@external_scene参数,若不为空则认为其数据就是@modelFileName文件的数据，不再从本地读取
    bool loadLocalModel(const std::string &modelFileName,const aiScene *external_scene=nullptr);
    bool initialize(){return false;};
    bool bindMeshTexture(const std::string &meshName,const std::string &textureName);
    bool activeMeshTexture(const std::string &meshName,const std::string &textureName);
    bool render(const glm::mat4 &p,const glm::mat4 &v,const glm::mat4 &m);
    int getBoneNodeIndexByName(const std::string &name) const;
    void setBoneNodeMatrices(const std::string &bone,const glm::mat4 &m);

    //=========================================================================
private:
    bool _valid{false};
    struct NodeAnimation{
        std::string nodeName;
        std::vector<std::pair<double,glm::vec3>> positionKeys;
        std::vector<std::pair<double,glm::quat>> rotationKeys;
        std::vector<std::pair<double,glm::vec3>> scaleKeys;
    };
    struct Animation{
        std::string name;
        double duration;    // in ticks
        double ticksPerSecond;
        std::map<std::string,NodeAnimation> channels; // key: nodeName
        std::unordered_map<std::string,glm::mat4> nodeTransforms; //每个动画维护独立姿态树,可以同时处理多段动画
        float weight=1.0f; // 默认权重
        double animation_time{0};
        long long last_update_time{0}; //unix msec timestamp
    };
    std::vector<Animation> mAnimations;
//    Animation mAnimation;
//    double mAnimationTime=0;
    bool mHasAnimation=false;

    glm::mat4 interpolateNodeTransform(const NodeAnimation &nodeAnim,double timeInTicks);
    void updateStaticHierarchy(const aiNode* node, const glm::mat4& parentTransform);
    void updateNodeHierarchy(const aiNode *node,const glm::mat4 &parentTransform,int animation_index,double timeInTicks);

public:
    void update_animations();
    void compose_animations();
    bool start_animation(int animation_index);
    bool stop_animation(int animation_index);
    bool control_all_animations(bool start);
    int animations_count()const;
    std::vector<int> playing_animations_list()const;
    void update_animation(float delta_time);
    bool is_valid()const;
    //=========================================================================

private:
    void initShader();
    std::vector<Texture> loadMaterialTextures(aiMaterial *mat,aiTextureType type,std::string typeName);
    std::vector<Texture> loadMaterialTextures_force(aiMaterial *mat,aiTextureType type,std::string typeName,std::string file);
    void processNode(aiNode *node,const aiScene *scene);
    AniMesh processMesh(aiMesh *mesh,const aiScene *scene);
    void processMeshBone(aiMesh *mesh,std::vector<Vertex> &vertices);
    void initializeBoneNode();
    void draw();

private:
    Assimp::Importer mImporter; // 成员变量
    const aiScene *mScene=nullptr; // 保存aiScene指针
    glm::mat4 mLastRenderModel{1.0f}; //最近一次render()时的model参数

    std::string mName;
    std::map<std::string,AniMesh> mMeshes;
    bool mHasBoneInfo;
    struct boneInfo{
        int id; /*id is index in finalBoneMatrices*/
        explicit boneInfo(int count):id(count){};
    };

    std::map<std::string,std::shared_ptr<boneInfo>> mBoneInfoMap;
    bool mIsGammaCorrection;
    std::vector<Texture> mTexturesLoaded;
    std::string mDirectory;
    std::map<std::string,std::vector<std::string>> mMeshTexturesMap;

    static Shader mShader;
};
