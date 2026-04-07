#pragma once
#include <string>
#include <map>
#include <vector>
#include <memory>
#include "mesh.h"
#include "shader.h"
#include "assimp/Importer.hpp"
#include "assimp/scene.h"
#include "assimp/postprocess.h"
#include "shader.h"


class Model {
public:
    Model() = delete;
    Model(const std::string& name, bool hasBoneInfo = false);
    ~Model();

    std::string& name();

    bool loadModel(const std::string& modelFileName);
    //为了支持在另一个线程中加载，添加了@external_scene参数,若不为空则认为其数据就是@modelFileName文件的数据，不再从本地读取. @normalize: 是否对模型进行标准化，防止过大或过小
    bool loadLocalModel(const std::string &modelFileName,bool normalize=false,const aiScene* external_scene=nullptr);
    bool initialize() { return false; };
    bool bindMeshTexture(const std::string& meshName, const std::string& textureName);
    bool activeMeshTexture(const std::string& meshName, const std::string& textureName);
    int getBoneNodeIndexByName(const std::string& name) const;
    void setBoneNodeMatrices(const std::string& bone, const glm::mat4& m);
    bool render(const glm::mat4& p, const glm::mat4& v, const glm::mat4& m);
    //=========================================================================
    bool is_valid()const;
    //@file_path: 必须是可直接访问的 Sdcard 路径
    static Model GetModelFromLocalFile(const std::string &model_name,const std::string &file_path,bool normalize=false);
private:
    bool _valid{false};
    //=========================================================================

private:
    void initShader();
    std::vector<Texture> loadMaterialTextures(aiMaterial* mat, aiTextureType type, std::string typeName);
    std::vector<Texture> loadMaterialTextures_force(aiMaterial* mat, aiTextureType type, std::string typeName, std::string file);
    void processNode(aiNode* node, const aiScene* scene);
    Mesh processMesh(aiMesh* mesh, const aiScene* scene);
    void processMeshBone(aiMesh* mesh, std::vector<Vertex>& vertices);
    void initializeBoneNode();
    void draw();

private:
    std::string mName;
    std::map<std::string, Mesh> mMeshes;
    bool mHasBoneInfo;
    
    struct boneInfo {
        /*id is index in finalBoneMatrices*/
        int id;
        /*offset matrix transforms vertex from model space to bone space*/
        glm::mat4 offset{};
        boneInfo(int count) : id(count) {};
    };
    std::map<std::string, std::shared_ptr<boneInfo>> mBoneInfoMap;

    bool mIsGammaCorrection;

    std::vector<Texture> mTexturesLoaded;
    std::string mDirectory;

    std::map<std::string, std::vector<std::string>> mMeshTexturesMap;

    static Shader mShader;
};
