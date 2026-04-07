#ifndef ROKIDOPENXRANDROIDDEMO_MODELMANAGER_HPP
#define ROKIDOPENXRANDROIDDEMO_MODELMANAGER_HPP

#include"demos/animodel.h"
#include"demos/model.h"
#include"posemotion.hpp"
#include<thread>
#include<map>
#include"demos/utils.h"

class ModelManager{
public:
    enum ModelState{
        NotExists=0,    //模型不存在
        Ready=1,        //模型全部加载完成,可进行渲染
        Loading=2,      //模型正在加载中
        WaitSetup=3,    //模型加载完成,但还没有创建OpenGL上下文
        LoadFailed=4,   //模型加载失败
    };
    std::shared_ptr<AniModel> get_model(const std::string &model_name){
        if(!model_exists(model_name)) return nullptr;
        return mModelsMap[model_name].model;
    }
    bool set_model_adjust(const std::string &model_name,const glm::mat4 &adjust_mat){
        if(!model_exists(model_name)) return false;
        mModelsMap[model_name].adjust=adjust_mat;
        return true;
    }
    bool set_model_scale(const std::string &model_name,const glm::vec3 &scale){
        if(!model_exists(model_name)) return false;
        mModelsMap[model_name].scale=scale;
        return true;
    }
    bool model_exists(const std::string &model_name)const{
        return mModelsMap.find(model_name)!=mModelsMap.end();
    }
    //检查@model_name的scene是不是已经读取完成，如果还未读取完成就等待读取完成；如果读取完成但还未创建OpenGL上下文就加载一下保证模型有效，可重复调用
    void make_model_valid(const std::string &model_name){
        if(!model_exists(model_name)) return;
        auto &item=mModelsMap[model_name];
        while(item.state.load()==0);
        if(item.state.load()!=2) return;
        auto s_t=CurrentMSecsSinceEpoch();
        if(!item.model->loadLocalModel(item.model_path,item.model_scene)){
            errorf("Init Model '%s' with External Scene Failed!",item.model->name().c_str());
            item.state.store(3);
        }
        else item.state.store(1);
        infof("============================ Load Model '%s' Cost: %lld ms, State: %d",item.model->name().c_str(),CurrentMSecsSinceEpoch()-s_t,item.state.load());
    }
    bool render_model(const std::string &model_name,const glm::mat4& p,const glm::mat4& v,const glm::mat4& m,bool update_animation=true){
        if(!model_exists(model_name)) return false;
        auto& item=mModelsMap[model_name];
        int state_code=item.state.load();
        if(state_code!=1) return false;
        if(update_animation){
            item.model->update_animations();
            item.model->compose_animations();
        }
        item.model->render(p,v,glm::scale(m*item.adjust,item.scale));
        return true;
    }
    bool start_model_animation(const std::string &model_name,int animation_index){
        if(!model_exists(model_name)) return false;
        auto &item=mModelsMap[model_name];
        if(item.state.load()!=1) return false;
        return item.model->start_animation(animation_index);
    }
    bool stop_model_animation(const std::string &model_name,int animation_index){
        if(!model_exists(model_name)) return false;
        auto &item=mModelsMap[model_name];
        if(item.state.load()!=1) return false;
        return item.model->stop_animation(animation_index);
    }
    bool control_model_all_animations(const std::string &model_name,bool start){ //控制模型全部动画的打开/关闭
        if(!model_exists(model_name)) return false;
        auto &item=mModelsMap[model_name];
        if(item.state.load()!=1) return false;
        return item.model->control_all_animations(start);
    }
    ModelState get_model_state(const std::string &model_name){
        if(!model_exists(model_name)) return ModelState::NotExists;
        int load_state=mModelsMap[model_name].state.load();
        if(load_state==1) return ModelState::Ready;
        else if(load_state==0) return ModelState::Loading;
        else if(load_state==2) return ModelState::WaitSetup;
        else return ModelState::LoadFailed;
    }
    //@local_path 必须是可直接读取的本地路径，@model_name为空或重复会添加失败
    bool add_model_from_file(const std::string &model_name,const std::string &local_path,bool detach=true){
        if(model_exists(model_name)){
            warnf("Can't Add Model '%s' Because Model Name Duplicate",model_name.c_str()) return false;
        }
        auto& item=mModelsMap[model_name];
        item.name=model_name; item.model_path=local_path;
        item.model=std::make_shared<AniModel>(model_name);
        if(detach) item.load_assimp_async(item.model_path);
        else{
            if(item.model->loadLocalModel(item.model_path)) item.state.store(1);
            else item.state.store(3);
        }
        return true;
    }
    std::vector<std::string> model_name_list(const std::string &prefix={}){
        std::vector<std::string> res;
        for(const auto &[model_name,_]:mModelsMap){
            if(string_starts_with(model_name,prefix)) res.push_back(model_name);
        }
        return res;
    }
    static ModelManager& Manager(){
        if(!GlobalManager) GlobalManager=std::make_unique<ModelManager>();
        return *GlobalManager;
    }

private:
    struct ModelItem{
        std::string name{};
        std::shared_ptr<AniModel> model;
        std::atomic<int> state{0}; //表示是否读取完成. 0:未完成; 1: 模型读取完成,创建OpenGL上下文完成; 2: 模型scene已经读取到内存中, 但还未创建; 3:文件读取失败
        glm::mat4   adjust=glm::mat4(1.0f);
        glm::vec3   scale{1.f,1.f,1.f};

        std::string                         model_path{};
        const aiScene*                      model_scene{nullptr};
        std::unique_ptr<Assimp::Importer>   model_importer{}; // importer 必须和 scene 同生命周期
        void load_assimp_async(const std::string &file_path,unsigned int flags=aiProcess_Triangulate|aiProcess_GenSmoothNormals|aiProcess_FlipUVs|aiProcess_CalcTangentSpace){
            this->state.store(0); model_path=file_path;
            std::thread([this,file_path,flags](){
                auto s_t=CurrentMSecsSinceEpoch();
                this->model_importer=std::make_unique<Assimp::Importer>();
                this->model_scene=this->model_importer->ReadFile(file_path,flags);
                if(!this->model_scene||this->model_scene->mFlags&AI_SCENE_FLAGS_INCOMPLETE||!this->model_scene->mRootNode){
                    errorf("Load Local Model(%s) Async Failed: %s",file_path.c_str(),this->model_importer->GetErrorString());
                    this->model_importer.reset();
                    this->model_scene=nullptr;
                    this->state.store(3);
                }
                else this->state.store(2);
                infof("=======================Setup Assimp Cost %lld ms, State: %d, ModelPath: %s",CurrentMSecsSinceEpoch()-s_t,this->state.load(),file_path.c_str());
            }).detach();
        }
    };
    inline static bool string_starts_with(const std::string &str,const std::string &starts_with){
        if((str.rfind(starts_with,0)==0)) return true; //start with
        return false;
    }
    std::map<std::string,ModelItem> mModelsMap;
    std::map<std::string,std::string> mFilePathToId; //某个模型文件路径第一次被添加时被赋予的ID
    inline static std::unique_ptr<ModelManager> GlobalManager;

    inline static constexpr bool SHARED_MODEL_RESOURCE=true;
};


#endif //ROKIDOPENXRANDROIDDEMO_MODELMANAGER_HPP
