#include"AniModel.h"
#include"utils.h"
#include"logger.h"
#define GLM_ENABLE_EXPERIMENTAL
#include"glm/gtx/quaternion.hpp"
#include<glm/gtx/matrix_decompose.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include"assimp/postprocess.h"
#include<iostream>
#include<fstream>
#include<algorithm>
#include<queue>
#include"app/utilsmym.hpp"
#include<thread>

Shader AniModel::mShader;

void AniModel::initShader(){
    static bool init=false;
    if(init){
        return;
    }
    else{
        const char *vertexShaderCode=R"_(
             #version 320 es
            layout(location = 0) in vec3 aPos;
            layout(location = 1) in vec3 aNormal;
            layout(location = 2) in vec2 aTexCoords;
            layout(location = 3) in vec3 tangent;
            layout(location = 4) in vec3 bitangent;
            layout(location = 5) in ivec4 boneIds;
            layout(location = 6) in vec4 weights;
            uniform mat4 model;
            uniform mat4 view;
            uniform mat4 projection;
            const int MAX_BONE_NODES = 100;
            const int MAX_BONE_INFLUENCE = 4;
            uniform mat4 finalBoneNodesMatrices[MAX_BONE_NODES];
            out vec2 TexCoords;
            void main(){
                vec4 total_position = vec4(0.0f);
                bool has_bone = false;
                for (int i = 0; i < MAX_BONE_INFLUENCE; i++) {
                    if (boneIds[i] < 0) {
                        continue;
                    }
                    if (boneIds[i] >= MAX_BONE_NODES) {
                        total_position = vec4(aPos, 1.0f);
                        break;
                    }
                    vec4 local_position = finalBoneNodesMatrices[boneIds[i]] * vec4(aPos, 1.0f);
                    total_position += local_position * weights[i];
                    has_bone = true;
                }
                if (has_bone == false) {
                    total_position = vec4(aPos, 1.0f);
                }
                gl_Position = projection * view * model * total_position;
                TexCoords = aTexCoords;
            }
        )_";

        const char *fragmentShaderCode=R"_(
            #version 320 es
            precision mediump float;
            out vec4 FragColor;
            in vec2 TexCoords;
            uniform sampler2D texture_diffuse1;
            void main()
            {
                FragColor = texture(texture_diffuse1, TexCoords);
            }
        )_";
        mShader.loadShader(vertexShaderCode,fragmentShaderCode);
        init=true;
    }
}

AniModel::AniModel(const std::string &name,bool hasBoneInfo):mName(name),mHasBoneInfo(hasBoneInfo){
//    initShader(); // ************* 调试用，正常没有这行代码(2025-09-09) *************
    mBoneInfoMap.clear();
}

AniModel::~AniModel(){
    mBoneInfoMap.clear();
}

std::string &AniModel::name(){
    return mName;
}

bool AniModel::bindMeshTexture(const std::string &meshName,const std::string &textureName){
    auto it=mMeshTexturesMap.find(meshName);
    if(it==mMeshTexturesMap.end()){
        std::vector<std::string> textureNames(1,textureName);
        mMeshTexturesMap[meshName]=textureNames;
    }
    else{
        it->second.push_back(textureName);
    }
    return true;
}

bool AniModel::activeMeshTexture(const std::string &meshName,const std::string &textureName){
    auto it=mMeshes.find(meshName);
    if(it!=mMeshes.end()){
        return it->second.activeTexture(textureName);
    }
    return false;
}

std::vector<Texture> AniModel::loadMaterialTextures(aiMaterial *mat,aiTextureType type,std::string typeName){
    std::vector<Texture> textures;
    for(uint32_t i=0;i<mat->GetTextureCount(type);i++){
        aiString str;
        mat->GetTexture(type,i,&str);
        // check if Texture was loaded before and if so, continue to next iteration: skip loading a new texture
        bool skip=false;
        for(uint32_t j=0;j<mTexturesLoaded.size();j++){
            if(std::strcmp(mTexturesLoaded[j].path.data(),str.C_Str())==0){
                textures.push_back(mTexturesLoaded[j]);
                skip=true; // a Texture with the same filepath has already been loaded, continue to next one. (optimization)
                break;
            }
        }
        if(!skip){
            Texture texture;
            texture.id=TextureFromFileAssets(str.C_Str(),mDirectory);
            texture.type=typeName;
            texture.path=str.C_Str();
            texture.active=false;
            textures.push_back(texture);
            mTexturesLoaded.push_back(texture);  // store it as Texture loaded for entire model, to ensure we won't unnecessary load duplicate textures.
        }
    }
    return textures;
}

std::vector<Texture> AniModel::loadMaterialTextures_force(aiMaterial *mat,aiTextureType type,std::string typeName,std::string file){
    std::vector<Texture> textures;
    bool skip=false;
    for(uint32_t j=0;j<mTexturesLoaded.size();j++){
        if(std::strcmp(mTexturesLoaded[j].path.data(),file.c_str())==0){
            skip=true; // a Texture with the same filepath has already been loaded, continue to next one. (optimization)
            break;
        }
    }
    if(!skip){
        Texture texture;
        if((file.rfind("/storage/emulated/0/",0)==0)) texture.id=TextureFromFile(file.c_str(),""); //start with
        else texture.id=TextureFromFileAssets(file.c_str(),"");
        texture.type=typeName;
        texture.path=file.c_str();
        texture.active=false;
        textures.push_back(texture);
        mTexturesLoaded.push_back(texture);  // store it as Texture loaded for entire model, to ensure we won't unnecessary load duplicate textures.
    }
    return textures;
}

void AniModel::processMeshBone(aiMesh *mesh,std::vector<Vertex> &vertices){
    int boneIndex=0;
//    infof("processMeshBone mesh name:%s, vertices:%d, total bone:%d", mesh->mName.C_Str(), vertices.size(), mesh->mNumBones);
    for(uint32_t i=0;i<mesh->mNumBones;i++){
//        infof("i:%02d, bone: %-16s, %02d, total weights:%d", i, mesh->mBones[i]->mName.C_Str(), boneIndex, mesh->mBones[i]->mNumWeights);
        std::string name=mesh->mBones[i]->mName.C_Str();
        std::shared_ptr<boneInfo> boneInformation(new boneInfo(boneIndex));
        auto it=mBoneInfoMap.find(name);
        if(it==mBoneInfoMap.end()){
            mBoneInfoMap[name]=boneInformation;
        }
        else{
            errorf("already has boneNode %s",name.c_str());
        }

        for(int weightIndex=0;weightIndex<mesh->mBones[i]->mNumWeights;weightIndex++){
            int vertexIndex=mesh->mBones[i]->mWeights[weightIndex].mVertexId;
            float weight=mesh->mBones[i]->mWeights[weightIndex].mWeight;
            if(weightIndex<vertices.size()){
                Vertex &v=vertices[vertexIndex];
                int k=0;
                for(k=0;k<MAX_BONE_INFLUENCE;k++){
                    if(v.BoneIDs[k]<0){
                        v.Weights[k]=weight;
                        v.BoneIDs[k]=boneIndex;
                        break;
                    }
                }
                if(k>=4){
                    //errorf("k >= 4, weightIndex:%d", weightIndex);
                }
            }
            else{
                errorf("weightIndex %d > vertices.size() %d",weightIndex,vertices.size());
            }

        }
        boneIndex++;
    }
}

AniMesh AniModel::processMesh(aiMesh *mesh,const aiScene *scene){
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    std::vector<Texture> textures;

//    infof("mesh vertex count: %d, face count:%d, bone:%d", mesh->mNumVertices, mesh->mNumFaces, mesh->mNumBones);
    for(uint32_t i=0;i<mesh->mNumVertices;i++){
        Vertex vertex{};
        glm::vec3 vector{};
        vector.x=mesh->mVertices[i].x;
        vector.y=mesh->mVertices[i].y;
        vector.z=mesh->mVertices[i].z;
        vertex.Position=vector;

        if(mesh->HasNormals()){
            vector.x=mesh->mNormals[i].x;
            vector.y=mesh->mNormals[i].y;
            vector.z=mesh->mNormals[i].z;
            vertex.Normal=vector;
        }

        if(mesh->mTextureCoords[0]){
            glm::vec2 vec;
            vec.x=mesh->mTextureCoords[0][i].x;
            vec.y=mesh->mTextureCoords[0][i].y;
            vertex.TexCoords=vec;
            // tangent
            vector.x=mesh->mTangents[i].x;
            vector.y=mesh->mTangents[i].y;
            vector.z=mesh->mTangents[i].z;
            vertex.Tangent=vector;
            // bitangent
            vector.x=mesh->mBitangents[i].x;
            vector.y=mesh->mBitangents[i].y;
            vector.z=mesh->mBitangents[i].z;
            vertex.Bitangent=vector;
        }
        else{
            vertex.TexCoords=glm::vec2(0.0f,0.0f);
        }
        vertices.push_back(vertex);
    }

    for(uint32_t i=0;i<mesh->mNumFaces;i++){
        aiFace face=mesh->mFaces[i];
        // retrieve all indices of the face and store them in the indices vector
        for(uint32_t j=0;j<face.mNumIndices;j++){
            indices.push_back(face.mIndices[j]);
        }
    }
    if(mHasBoneInfo){
        processMeshBone(mesh,vertices);
    }
    aiMaterial *material=scene->mMaterials[mesh->mMaterialIndex];

    /*
    // 1. diffuse maps
    std::vector<Texture> diffuseMaps = loadMaterialTextures(material, aiTextureType_DIFFUSE, "texture_diffuse");
    textures.insert(textures.end(), diffuseMaps.begin(), diffuseMaps.end());
    // 2. specular maps
    std::vector<Texture> specularMaps = loadMaterialTextures(material, aiTextureType_SPECULAR, "texture_specular");
    textures.insert(textures.end(), specularMaps.begin(), specularMaps.end());
    // 3. normal maps
    std::vector<Texture> normalMaps = loadMaterialTextures(material, aiTextureType_HEIGHT, "texture_normal");
    textures.insert(textures.end(), normalMaps.begin(), normalMaps.end());
    // 4. height maps
    std::vector<Texture> heightMaps = loadMaterialTextures(material, aiTextureType_AMBIENT, "texture_height");
    textures.insert(textures.end(), heightMaps.begin(), heightMaps.end());
    */

//    infof((std::string("MName: ")+mesh->mName.C_Str()).c_str());
    auto it=mMeshTexturesMap.find(mesh->mName.C_Str());
    if(it!=mMeshTexturesMap.end()){
        for(auto &i:it->second){
            std::vector<Texture> texture=loadMaterialTextures_force(material,aiTextureType_DIFFUSE,"texture_diffuse",i);
            textures.insert(textures.end(),texture.begin(),texture.end());
        }
    }

    return AniMesh(vertices,indices,textures);
}

void AniModel::processNode(aiNode *node,const aiScene *scene){
    static std::string indent="";
//    infof("%snode:%s, children:%d", indent.c_str(), node->mName.C_Str(), node->mNumChildren); ** My Changes : Comment **
    for(uint32_t i=0;i<node->mNumMeshes;i++){
        aiMesh *mesh=scene->mMeshes[node->mMeshes[i]];
//        infof("%smesh: %s", indent.c_str(), mesh->mName.C_Str()); ** My Changes : Comment **
        mMeshes.insert(std::pair<std::string,AniMesh>(mesh->mName.C_Str(),processMesh(mesh,scene)));
    }
    for(uint32_t i=0;i<node->mNumChildren;i++){
        std::string tmp=indent;
        indent+="  ";
        processNode(node->mChildren[i],scene);
        indent=tmp;
    }
}

void AniModel::draw(){
    GL_CALL(glFrontFace(GL_CCW));
    GL_CALL(glCullFace(GL_BACK));
    GL_CALL(glEnable(GL_CULL_FACE));
    GL_CALL(glEnable(GL_DEPTH_TEST));
    GL_CALL(glEnable(GL_BLEND));
    GL_CALL(glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA));
    for(auto &it:mMeshes){
        it.second.draw(mShader);
    }
}

void AniModel::update_animation(float delta_time){
    if(!mHasAnimation){
        updateStaticHierarchy(mScene->mRootNode,mLastRenderModel);
        return;
    }
    for(int i=0;i<(int)mAnimations.size();++i){
        Animation &ani=mAnimations[i];
        if(ani.last_update_time<=0) continue; //not playing
        ani.animation_time+=delta_time*ani.ticksPerSecond;
        ani.last_update_time=CurrentMSecsSinceEpoch();
        double timeInTicks=fmod(ani.animation_time,ani.duration);
        updateNodeHierarchy(mScene->mRootNode,mLastRenderModel/*glm::mat4(1.0f)*/,i,timeInTicks);
    }
}
bool AniModel::render(const glm::mat4 &p,const glm::mat4 &v,const glm::mat4 &m){
    mLastRenderModel=m;
    mShader.use();
    mShader.setUniformMat4("projection",p);
    mShader.setUniformMat4("view",v);
    mShader.setUniformMat4("model",m);
    draw();
    glUseProgram(0);
    return true;
}
void AniModel::initializeBoneNode(){
    mShader.use();
    glm::mat4 m=glm::mat4(1.0f);
    for(auto it:mBoneInfoMap){
        std::string name="finalBoneNodesMatrices["+std::to_string(it.second->id)+"]";
        mShader.setUniformMat4(name,m);
    }
}
int AniModel::getBoneNodeIndexByName(const std::string &name) const{
    auto it=mBoneInfoMap.find(name);
    if(it!=mBoneInfoMap.end()){
        return it->second->id;
    }
    errorf("not found bone %s",name.c_str());
    return -1;
}

void AniModel::setBoneNodeMatrices(const std::string &bone,const glm::mat4 &m){
    int index=getBoneNodeIndexByName(bone);
    if(index<0){
        return;
    }
    std::string name="finalBoneNodesMatrices["+std::to_string(index)+"]";
    mShader.use();
    mShader.setUniformMat4(name,m);
}

bool AniModel::loadLocalModel(const std::string &modelFileName,const aiScene *external_scene){
    initShader();
    mScene=external_scene;
    if(mScene==nullptr){
        mScene=mImporter.ReadFile(modelFileName,aiProcess_Triangulate|aiProcess_GenSmoothNormals|aiProcess_FlipUVs|aiProcess_CalcTangentSpace);
        if(mScene==nullptr||mScene->mFlags&AI_SCENE_FLAGS_INCOMPLETE||mScene->mRootNode==nullptr){
            Log::Write(Log::Level::Error,Fmt("assimp read file error %s",mImporter.GetErrorString()));
            return false;
        }
    }
    // ---------------------- Get Texture Filename ---------------------------
    std::vector<std::string> texture_filename_list;
    for(unsigned int m=0;m<mScene->mNumMaterials;m++){
        aiMaterial *mtl=mScene->mMaterials[m];
        int nTex;
        if((nTex=mtl->GetTextureCount(aiTextureType_DIFFUSE))>0){
            aiString path;    // filename
            for(int i=0;i<nTex;++i){
                if(mtl->GetTexture(aiTextureType_DIFFUSE,i,&path)==AI_SUCCESS){
//                    infof(("AIString: "+std::string(path.C_Str())).c_str());
                    texture_filename_list.emplace_back(path.C_Str());
                }
            }
        }
    }
    // --------------------- Get Mesh Name -----------------------------
    std::vector<std::string> mesh_name_list;
    std::queue<aiNode *> q; //遍历所有Node用
    q.push(mScene->mRootNode);
    while(!q.empty()){
        auto node=q.front();
        q.pop();
        for(uint32_t i=0;i<node->mNumMeshes;i++){
            aiMesh *mesh=mScene->mMeshes[node->mMeshes[i]];
            mesh_name_list.emplace_back(mesh->mName.C_Str());
//            infof((std::string("MName: ")+mesh->mName.C_Str()).c_str());
        }
        for(uint32_t i=0;i<node->mNumChildren;i++) q.push(node->mChildren[i]);
    }
    // 需要注意，如果mesh和texture的数量大于1.它们之间的对应关系暂时还无法确定，可能会有匹配错误的情况出现
    if(mesh_name_list.size()>1||texture_filename_list.size()>1){
        warnf("Mesh Num: %d and Texture Num: %d Upper than 1.",(int)mesh_name_list.size(),(int)texture_filename_list.size())
    }
    mDirectory=modelFileName.substr(0,modelFileName.find_last_of('/'));
    for(int i=0;i<(int)std::min(mesh_name_list.size(),texture_filename_list.size());++i){
        bindMeshTexture(mesh_name_list[i],mDirectory+'/'+texture_filename_list[i]);
    }
    infof("model:%s, scene:%s, mNumMeshes:%d, mNumMaterials:%d, mNumAnimations:%d, mNumTextures:%d",modelFileName.c_str(),mScene->mName.C_Str(),mScene->mNumMeshes,mScene->mNumMaterials,mScene->mNumAnimations,mScene->mNumTextures);
    processNode(mScene->mRootNode,mScene);
    //=========================== 把模型标准化，防止过大或过小 ================================
    if(0){
        double sum_x=0,sum_y=0,sum_z=0;
        int count=0;
        float vmax=0;
        for(auto &mesh:mMeshes){
            for(auto &v:mesh.second.mVertices){
                auto pos=v.Position;
                sum_x+=pos.x; sum_y+=pos.y; sum_z+=pos.z;
                ++count;
            }
        }
        infof("Average: %f, %f, %f. Count: %d",sum_x/count,sum_y/count,sum_z/count,count)
        for(auto &mesh:mMeshes){
            for(auto &v:mesh.second.mVertices){
                auto &pos=v.Position;
                pos.x-=(float)(sum_x/count);
                pos.y-=(float)(sum_y/count);
                pos.z-=(float)(sum_z/count);
                vmax=std::max(abs(pos.x),vmax);
                vmax=std::max(abs(pos.y),vmax);
                vmax=std::max(abs(pos.z),vmax);
            }
        }
        for(auto &mesh:mMeshes){
            for(auto &v:mesh.second.mVertices){
                auto &pos=v.Position;
                pos/=vmax;
                pos/=20;  //这行代码是干啥的？ (注释掉模型会变得巨大无比)
            }
            mesh.second.setupMesh();
        }
    }
    //====================================================================
    if(mScene->mNumAnimations>0){
        for(unsigned int _ani_index=0;_ani_index<mScene->mNumAnimations;++_ani_index){
            aiAnimation *anim=mScene->mAnimations[_ani_index];
            mAnimations.emplace_back();
            auto &animation=mAnimations[mAnimations.size()-1];
            animation.name=anim->mName.C_Str();
            animation.duration=anim->mDuration;
            animation.ticksPerSecond=anim->mTicksPerSecond!=0?anim->mTicksPerSecond:25.0;
            for(unsigned int i=0;i<anim->mNumChannels;i++){
                aiNodeAnim *channel=anim->mChannels[i];
                NodeAnimation nodeAnim;
                nodeAnim.nodeName=channel->mNodeName.C_Str();
                for(unsigned int j=0;j<channel->mNumPositionKeys;j++){
                    aiVectorKey key=channel->mPositionKeys[j];
                    nodeAnim.positionKeys.emplace_back(key.mTime,glm::vec3(key.mValue.x,key.mValue.y,key.mValue.z));
                }
                for(unsigned int j=0;j<channel->mNumRotationKeys;j++){
                    aiQuatKey key=channel->mRotationKeys[j];
                    nodeAnim.rotationKeys.emplace_back(key.mTime,glm::quat(key.mValue.w,key.mValue.x,key.mValue.y,key.mValue.z));
                }
                for(unsigned int j=0;j<channel->mNumScalingKeys;j++){
                    aiVectorKey key=channel->mScalingKeys[j];
                    nodeAnim.scaleKeys.emplace_back(key.mTime,glm::vec3(key.mValue.x,key.mValue.y,key.mValue.z));
                }
                animation.channels[nodeAnim.nodeName]=nodeAnim;
            }
        }
        mHasAnimation=true;
    }
    //==================================================================================================
    initializeBoneNode();
    for(int i=0;i<(int)std::min(mesh_name_list.size(),texture_filename_list.size());++i){
        activeMeshTexture(mesh_name_list[i],mDirectory+'/'+texture_filename_list[i]);
    }
    _valid=true;
    return true;
}


//=================================================================================================
bool AniModel::is_valid() const{
    return _valid;
}
inline glm::mat4 convertAIMatrixToGLMTransposed(const aiMatrix4x4 &from){
    glm::mat4 to;
    to[0][0]=from.a1;to[0][1]=from.a2;to[0][2]=from.a3;to[0][3]=from.a4;to[1][0]=from.b1;to[1][1]=from.b2;to[1][2]=from.b3;to[1][3]=from.b4;
    to[2][0]=from.c1;to[2][1]=from.c2;to[2][2]=from.c3;to[2][3]=from.c4;to[3][0]=from.d1;to[3][1]=from.d2;to[3][2]=from.d3;to[3][3]=from.d4;
    return to;
}
void AniModel::updateNodeHierarchy(const aiNode *node,const glm::mat4 &parentTransform,int animation_index,double timeInTicks){
    if(node==nullptr) return;
    std::string nodeName(node->mName.C_Str());
    glm::mat4 nodeTransform=convertAIMatrixToGLMTransposed(node->mTransformation);
    // 查找该节点是否被当前动画控制
    auto &ani=mAnimations[animation_index];
    auto it_channel=ani.channels.find(nodeName);
    if(it_channel!=ani.channels.end()){
        nodeTransform=interpolateNodeTransform(it_channel->second,timeInTicks);
    }
    glm::mat4 globalTransform=parentTransform*nodeTransform;
    // ✅ 仅记录节点的全局变换，不直接作用到 mesh
    ani.nodeTransforms[nodeName]=globalTransform;
    for(unsigned int i=0;i<node->mNumChildren;i++){
        updateNodeHierarchy(node->mChildren[i],globalTransform,animation_index,timeInTicks);
    }
}
void AniModel::updateStaticHierarchy(const aiNode *node,const glm::mat4 &parentTransform){
    if(node==nullptr) return;
    std::string nodeName(node->mName.C_Str());
    glm::mat4 nodeTransform=convertAIMatrixToGLMTransposed(node->mTransformation);
    glm::mat4 globalTransform=parentTransform*nodeTransform;
    // 如果这个 node 有 mesh，就应用这个 globalTransform
    for(unsigned int i=0;i<node->mNumMeshes;i++){
        std::string meshName=node->mName.C_Str();
        auto it=mMeshes.find(meshName);
        if(it!=mMeshes.end()) it->second.mModelMatrix=(globalTransform);
    }
    // 递归更新子节点
    for(unsigned int i=0;i<node->mNumChildren;i++) updateStaticHierarchy(node->mChildren[i],globalTransform);
}

glm::mat4 AniModel::interpolateNodeTransform(const AniModel::NodeAnimation &nodeAnim,double timeInTicks){
    // --- Interpolate translation ---
    glm::vec3 translation(0.0f);
    if(!nodeAnim.positionKeys.empty()){
        if(nodeAnim.positionKeys.size()==1) translation=nodeAnim.positionKeys[0].second;
        else{
            for(size_t i=0;i<nodeAnim.positionKeys.size()-1;i++){
                if(timeInTicks<nodeAnim.positionKeys[i+1].first){
                    double t1=nodeAnim.positionKeys[i].first;
                    double t2=nodeAnim.positionKeys[i+1].first;
                    float factor=float((timeInTicks-t1)/(t2-t1));
                    translation=glm::mix(nodeAnim.positionKeys[i].second,nodeAnim.positionKeys[i+1].second,factor);
                    break;
                }
            }
        }
    }
    // --- Interpolate rotation ---
    glm::quat rotation(1,0,0,0);
    if(!nodeAnim.rotationKeys.empty()){
        if(nodeAnim.rotationKeys.size()==1) rotation=nodeAnim.rotationKeys[0].second;
        else{
            for(size_t i=0;i<nodeAnim.rotationKeys.size()-1;i++){
                if(timeInTicks<nodeAnim.rotationKeys[i+1].first){
                    double t1=nodeAnim.rotationKeys[i].first;
                    double t2=nodeAnim.rotationKeys[i+1].first;
                    float factor=float((timeInTicks-t1)/(t2-t1));
                    rotation=glm::slerp(nodeAnim.rotationKeys[i].second,nodeAnim.rotationKeys[i+1].second,factor);
                    break;
                }
            }
        }
    }
    // --- Interpolate scale ---
    glm::vec3 scale(1.0f);
    if(!nodeAnim.scaleKeys.empty()){
        if(nodeAnim.scaleKeys.size()==1) scale=nodeAnim.scaleKeys[0].second;
        else{
            for(size_t i=0;i<nodeAnim.scaleKeys.size()-1;i++){
                if(timeInTicks<nodeAnim.scaleKeys[i+1].first){
                    double t1=nodeAnim.scaleKeys[i].first;
                    double t2=nodeAnim.scaleKeys[i+1].first;
                    float factor=float((timeInTicks-t1)/(t2-t1));
                    scale=glm::mix(nodeAnim.scaleKeys[i].second,nodeAnim.scaleKeys[i+1].second,factor);
                    break;
                }
            }
        }
    }
    glm::mat4 T=glm::translate(glm::mat4(1.0f),translation);
    glm::mat4 R=glm::toMat4(rotation);
    glm::mat4 S=glm::scale(glm::mat4(1.0f),scale);
    return T*R*S;
}
bool AniModel::start_animation(int animation_index){
    if(animation_index<0||animation_index>=(int)mAnimations.size()) return false;
    if(mAnimations[animation_index].last_update_time<=0) mAnimations[animation_index].last_update_time=CurrentMSecsSinceEpoch();
    return true;
}
bool AniModel::stop_animation(int animation_index){
    if(animation_index<0||animation_index>=(int)mAnimations.size()) return false;
    if(mAnimations[animation_index].last_update_time>0) mAnimations[animation_index].last_update_time=0;
    return true;
}
int AniModel::animations_count() const{
    return (int)mAnimations.size();
}
std::vector<int> AniModel::playing_animations_list()const{
    std::vector<int> res;
    for(int i=0;i<(int)mAnimations.size();++i) if(mAnimations[i].last_update_time>0) res.push_back(i);
    return res;
}
bool AniModel::control_all_animations(bool start){
    bool flag=true;
    for(int i=0;i<(int)mAnimations.size();++i) flag&=(start?start_animation(i):stop_animation(i));
    return flag;
}
void AniModel::update_animations(){
    if(!mHasAnimation){
        updateStaticHierarchy(mScene->mRootNode,mLastRenderModel);
        return;
    }
    long long cur_msec=CurrentMSecsSinceEpoch();
    for(int i=0;i<(int)mAnimations.size();++i){
        Animation &ani=mAnimations[i];
        if(ani.last_update_time<=0) continue; //not playing
        auto delta_time=float((double)(cur_msec-ani.last_update_time)/1000.0);
        ani.animation_time+=delta_time*ani.ticksPerSecond;
        double timeInTicks=fmod(ani.animation_time,ani.duration);
        ani.last_update_time=cur_msec;
        // ✅ 更新每个动画自己的节点变换缓存
        ani.nodeTransforms.clear();
//        updateNodeHierarchy(mScene->mRootNode,glm::mat4(1.0f),i,timeInTicks);
        updateNodeHierarchy(mScene->mRootNode,mLastRenderModel/*glm::mat4(1.0f)*/,i,timeInTicks);
    }
}
inline void decomposeTRS(const glm::mat4 &mat,
                            glm::vec3 &translation,
                            glm::quat &rotation,
                            glm::vec3 &scale)
{
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::vec3 dummy;
    glm::decompose(mat, scale, rotation, translation, skew, perspective);
}
void AniModel::compose_animations(){
    // 记录每个节点的混合状态
    struct TRS {
        glm::vec3 translation{0.0f};
        glm::quat rotation{1, 0, 0, 0};
        glm::vec3 scale{0.0f};
        float totalWeight = 0.0f;
        bool first = true;
    };
    std::unordered_map<std::string, TRS> blendedNodes;
    // 遍历所有正在播放的动画
    for (auto &ani : mAnimations) {
        if (ani.last_update_time <= 0 || ani.weight <= 0.0f)
            continue;

        for (auto &kv : ani.nodeTransforms) {
            const std::string &nodeName = kv.first;
            const glm::mat4 &mat = kv.second;

            glm::vec3 T, S;
            glm::quat R;
            decomposeTRS(mat, T, R, S); // ✅ 我们要写一个decomposeTRS函数

            auto &bn = blendedNodes[nodeName];
            float w = ani.weight;
            bn.totalWeight += w;

            if (bn.first) {
                bn.translation = T * w;
                bn.rotation = glm::normalize(R);
                bn.scale = S * w;
                bn.first = false;
            } else {
                bn.translation += T * w;
                bn.rotation = glm::slerp(bn.rotation, R, w / bn.totalWeight); // 平滑插值旋转
                bn.scale += S * w;
            }
        }
    }

    // ✅ 合成最终矩阵并应用到 mesh
    for (auto &it : blendedNodes) {
        const std::string &nodeName = it.first;
        auto &bn = it.second;
        if (bn.totalWeight < 1e-6f) continue;

        glm::vec3 T = bn.translation / bn.totalWeight;
        glm::vec3 S = bn.scale / bn.totalWeight;
        glm::quat R = glm::normalize(bn.rotation);

        glm::mat4 finalMat = glm::translate(glm::mat4(1.0f), T)
                             * glm::toMat4(R)
                             * glm::scale(glm::mat4(1.0f), S);

        auto meshIt = mMeshes.find(nodeName);
        if (meshIt != mMeshes.end()) {
            meshIt->second.mModelMatrix = finalMat;
        }
    }
}

AniMesh::AniMesh(std::vector<Vertex> vertices,std::vector<unsigned int> indices,std::vector<Texture> textures):mVertices(vertices),mIndices(indices),mTextures(textures){
    setupMesh();
}
void AniMesh::setupMesh(){
    // create buffers/arrays
    //glGenFramebuffers(1, &mFramebuffer);
    //glBindFramebuffer(GL_FRAMEBUFFER, mFramebuffer);
    glGenVertexArrays(1,&mVAO);
    glGenBuffers(1,&mVBO);
    glGenBuffers(1,&mEBO);

    glBindVertexArray(mVAO);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER,mVBO);
    // A great thing about structs is that their memory layout is sequential for all its items.
    // The effect is that we can simply pass a pointer to the struct and it translates perfectly to a glm::vec3/2 array which
    // again translates to 3/2 floats which translates to a byte array.
    glBufferData(GL_ARRAY_BUFFER,mVertices.size()*sizeof(Vertex),&mVertices[0],GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,mEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,mIndices.size()*sizeof(unsigned int),&mIndices[0],GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void *)0);
    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void *)offsetof(Vertex,Normal));
    // vertex Texture coords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2,2,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void *)offsetof(Vertex,TexCoords));
    // vertex tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void *)offsetof(Vertex,Tangent));
    // vertex bitangent
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4,3,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void *)offsetof(Vertex,Bitangent));
    // ids
    glEnableVertexAttribArray(5);
    glVertexAttribIPointer(5,4,GL_INT,sizeof(Vertex),(void *)offsetof(Vertex,BoneIDs));
    // weights
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6,4,GL_FLOAT,GL_FALSE,sizeof(Vertex),(void *)offsetof(Vertex,Weights));
    glBindVertexArray(0);
}

bool AniMesh::activeTexture(const std::string &textureName){
    for(auto &it:mTextures){
        if(textureName==it.path) it.active=true;
        else it.active=false;
    }
    return true;
}
void AniMesh::draw(Shader &shader){
    // 在绘制前上传模型矩阵
    shader.setUniformMat4("model",mModelMatrix);  // ✅ 新增
    // bind appropriate textures
    unsigned int diffuseNr=1;
    unsigned int specularNr=1;
    unsigned int normalNr=1;
    unsigned int heightNr=1;
    for(unsigned int i=0;i<mTextures.size();i++){
        if(mTextures[i].active==false){
            continue;
        }
        glActiveTexture(GL_TEXTURE0+i); // active proper Texture unit before binding
        // retrieve Texture number (the N in diffuse_textureN)
        std::string number;
        std::string name=mTextures[i].type;
        if(name=="texture_diffuse"){
            number=std::to_string(diffuseNr++);
        }
        else if(name=="texture_specular"){
            number=std::to_string(specularNr++); // transfer unsigned int to string
        }
        else if(name=="texture_normal"){
            number=std::to_string(normalNr++); // transfer unsigned int to string
        }
        else if(name=="texture_height"){
            number=std::to_string(heightNr++); // transfer unsigned int to string
        }
        // now set the sampler to the correct Texture unit
        glUniform1i(glGetUniformLocation(shader.id(),(name+number).c_str()),i);
        // and finally bind the texture
        glBindTexture(GL_TEXTURE_2D,mTextures[i].id);
    }
    // draw mesh
    glBindVertexArray(mVAO);
    glDrawElements(GL_TRIANGLES,static_cast<unsigned int>(mIndices.size()),GL_UNSIGNED_INT,0);
    glBindVertexArray(0);
    // always good practice to set everything back to defaults once configured.
    glActiveTexture(GL_TEXTURE0);
}