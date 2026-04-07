#include"Basic/include/RPC.h"
#include"Basic/include/ARModule.h"
#include"Basic/include/App.h"
#include"opencv2/core.hpp"
#include"nlohmann/json.hpp"
#include"arucopp.h"
#include"arengine.h"
#include"demos/gui.h"
#include"scene.h"
#include"demos/utils.h"
#include"demos/model.h"
#include"demos/text.h"
#include"markerdetector.hpp"
#include"scenegui.h"
#include"utilsmym.hpp"
#include"modelmanager.hpp"
#include"config.hpp"
#include"databank.hpp"
#include"humandetector.h"

using namespace cv;
using json=nlohmann::json;

/*
 * 用
*/

namespace{
struct MarkerPose {
    typedef std::tuple<int,glm::mat4,cv::Vec3d,cv::Vec3d> ItemType; //<id,translation_model,rvec,tvec>
    std::vector<ItemType> markers;
};
void decomposeRT(const Matx44f &m,cv::Matx33f &R,cv::Vec3f &tvec,bool gl2cv){
    const float *v=m.val;
    CV_Assert(fabs(v[3])<1e-3f&&fabs(v[7])<1e-3&&fabs(v[11])<1e-3&&fabs(v[15]-1.0f)<1e-3f);
    if(gl2cv){
        tvec[0]=v[12];tvec[1]=-v[13];tvec[2]=-v[14];
        R=cv::Matx33f(v[0],v[4],v[8],-v[1],-v[5],-v[9],-v[2],-v[6],-v[10]);
    }
    else{
        tvec[0]=v[12];tvec[1]=v[13];tvec[2]=v[14];
        R=cv::Matx33f(v[0],v[4],v[8],v[1],v[5],v[9],v[2],v[6],v[10]);
    }
}
struct DetectType{
    std::string marker_name="MarkerPose";   //指定检测结果的名称，以便后续根据此名称从sceneData中取出
    bool use_optimized_detector=true; //使用优化后的检测器。 如果为false,那么下面直到另一个 [bool]use_... 变量前的所有的选项都不生效

    bool use_opencv_aruco_detector=false;  //使用原始的opencv检测器。 如果为false,那么下面直到另一个 [bool]use_... 变量前的所有的选项都不生效
    cv::aruco::PredefinedDictionaryType aruco_type=cv::aruco::DICT_5X5_50;  //要检测的aruco类型
    cv::Size aruco_size={50,50};                               //要检测的aruco尺寸，单位为mm
    int aruco_id=-1;                                                        //要检测的aruco编号，-1为无要求
};

std::vector<DetectType> ArucoDetectList;
std::mutex ArucoDetectListMutex;
std::string ArucoDetectorOptimizedTemplateFilePath=Config::AppDataDir+("/templ_1.json"); //优化后的aruco检测器需要一个.json文件，这里指定.json文件的路径
glm::mat4 ModelAdjust{1.0f};

class ArucoDetector: public ARModule{
    ArucoPP _detector;
    bool _refineCorners=true;
    bool _isStatic=true;
    int _nSum=0;
    cv::Vec3f _mrvec,_mtvec;
public:
    int Init(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        _detector.loadTemplate(ArucoDetectorOptimizedTemplateFilePath);
        return STATE_OK;
    }
    int Update(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        if(!frameDataPtr->image.empty()){
            cv::Mat img=frameDataPtr->image.front();
            auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs"));
            cv::Matx44f vmat=frame_data.cameraMat;
            //====================== 根据设置的不同检测器进行检测 ========================
            if(!ArucoDetectListMutex.try_lock()) return STATE_OK;
            for(const auto &_dt:ArucoDetectList){
                if(_dt.use_optimized_detector){
                    const cv::Matx33f &camK=frameDataPtr->colorCameraMatrix;
                    cv::Vec3d rvec,tvec;
                    if(_detector.detect(img,camK,rvec,tvec)){
                        cv::Matx44f Tmw=GLM_Mat4_to_CV_Mat(MarkerDetector::GetTransMatFromRT(rvec,tvec,CV_Matx44f_to_GLM_Mat4(vmat)));
                        if(_isStatic){
                            if(_nSum==0){_mrvec=_mtvec=cv::Vec3f(0.f,0.f,0.f);}
                            cv::Matx44f Twc=vmat;
                            Matx33f R;
                            Vec3f t,r;
                            decomposeRT(Tmw,R,t,true);
                            cv::Rodrigues(R,r);

                            _mrvec=(_mrvec*float(_nSum)+r)/(_nSum+1);
                            _mtvec=(_mtvec*float(_nSum)+t)/(_nSum+1);
                            _nSum++;
                            Tmw=FromRT(_mrvec,_mtvec,true);
                        }
                        MarkerPose pose;
                        glm::mat4 res=CV_Matx44f_to_GLM_Mat4(Tmw);
//                        res=glm::scale(res,{-1,-1,-1});
                        pose.markers.emplace_back(0,res,rvec,tvec);
                        sceneData.setData(_dt.marker_name,pose);
                        infof("detector[%s]: %s",_dt.marker_name.c_str(),GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(Tmw)).c_str())
                    }
                    //=============== HumanDetector Test ================
//                    DataBank::Global().set("vmat",vmat);
//                    if(!HumanDetector::Global().camera_mat_valid()) HumanDetector::Global().set_camera_mat(camK);
//                    if(!HumanDetector::Global().server_started()) HumanDetector::Global().start_server(Config::global().get("HumanDetectorServerIp"),std::stoi(Config::global().get("HumanDetectorServerPort")));
//                    HumanDetector::Global().detect(img);
                    //===================================================
                }
                else if(_dt.use_opencv_aruco_detector){
                    MarkerDetector detector((cv::Mat)frameDataPtr->colorCameraMatrix);
                    auto [flag1,rvec1,tvec1]=detector.detect_aruco(img,_dt.aruco_type,_dt.aruco_size,_dt.aruco_id);
                    if(flag1){
                        MarkerPose pose;
                        pose.markers.emplace_back(_dt.aruco_id,MarkerDetector::GetTransMatFromRT(rvec1,tvec1,CV_Matx44f_to_GLM_Mat4(vmat)),rvec1,tvec1);
                        sceneData.setData(_dt.marker_name,pose);
                    }
                    else infof("Detect Aruco Failed");
                }
            }
            ArucoDetectListMutex.unlock();
        }
        return STATE_OK;
    }
};
std::shared_ptr<ARApp> construct_engine(){
    std::string appName="TestApp"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）
    std::vector<ARModulePtr> modules;
    modules.push_back(createModule<ARInputs>("ARInputs"));
    modules.push_back(createModule<ArucoDetector>("ArucoDetector"));
    auto appData=std::make_shared<AppData>();
    auto sceneData=std::make_shared<SceneData>();
    appData->argc=1;
    appData->argv=nullptr;
    appData->engineDir="./AREngine/";  // for test
    appData->dataDir="./data/";        // for test
    std::shared_ptr<ARApp> app=std::make_shared<ARApp>();
    app->init(appName,appData,sceneData,modules);
    return app;
}
}
//================================================================================================================
// JSON解析辅助函数
namespace nlohmann{
// glm::vec3 的 JSON 序列化/反序列化
template<>
struct adl_serializer<glm::vec3>{
    static void to_json(json &j,const glm::vec3 &vec){
        j=json::array({vec.x,vec.y,vec.z});
    }
    static void from_json(const json &j,glm::vec3 &vec){
        if(j.is_array()&&j.size()==3){
            vec.x=j[0].get<float>(); vec.y=j[1].get<float>(); vec.z=j[2].get<float>();
        }
        else vec=glm::vec3(1.0f,1.0f,1.0f);
    }
};
// cv::Size 的 JSON 序列化/反序列化
template<>
struct adl_serializer<cv::Size>{
    static void to_json(json &j,const cv::Size &vec){
        j=json::array({vec.width,vec.height});
    }
    static void from_json(const json &j,cv::Size &vec){
        if(j.is_array()&&j.size()==2){
            vec.width=j[0].get<int>(); vec.height=j[1].get<int>();
        }
        else vec=cv::Size(0.f,0.f);
    }
};
}
//================================================================================================================
namespace{
enum SceneItemType{BaseItem,ModelItem,TextItem,ImageItem};
struct SceneItemBase{
    std::string id{};                     //该组件的名称，唯一
    glm::vec3   scale{1.f};         //该组件在x,y,z轴上的缩放
    glm::vec3   translate{0};       //该组件在x,y,z轴上的平移
    float       rotate_angle{0};          //组件的旋转角度(不是弧度)
    glm::vec3   rotate_axis{1.f};   //组件的旋转轴
    std::string relative_id{};   //若不为空，则组件的平移/旋转等参数是相对于该id所对应的组件的. 需要保证不会出现循环依赖等关系，并且该id所对应的组件存在

    glm::mat4 translate_model{1.0f}; //组件的变换矩阵，根据平移/旋转等参数计算，不是在json文件中构造的，避免重复计算
    virtual SceneItemType type()const{return SceneItemType::BaseItem;}
    virtual bool init(){return true;}
    virtual void render(){};
    virtual ~SceneItemBase()=default;
};
struct SceneModel:SceneItemBase{ //Scene中的模型部分
    std::string file_path{}; //模型在本地可直接访问的路径
    int animation_index{0}; //要播放哪条动画,-1表示不播放,0表示全部
    bool is_static_model{false};
    SceneItemType type()const override{return SceneItemType::ModelItem;}

    bool init()override{
        return true;
    }
};
struct SceneImage:SceneItemBase{
    std::string file_path{}; //图片在本地可直接访问的路径
    cv::Mat image{};         //加载的图片

    SceneItemType type()const override{return SceneItemType::ImageItem;}
    bool init()override{
        return true;
    }
};
struct SceneText:SceneItemBase{
    std::string text{};      //要显示的文本
    glm::vec3 color{1.f};       //文本的颜色
    float line_gap{0.11f}; //行间距，单位为m

    SceneItemType type()const override{return SceneItemType::TextItem;}
};

// 从JSON解析SceneItemBase的基础字段
void from_json_base(const json &j,std::shared_ptr<SceneItemBase> &item){
    if(j.contains("id")) item->id=j["id"].get<std::string>();
    if(j.contains("scale")) item->scale=j["scale"].get<glm::vec3>();
    if(j.contains("translate")) item->translate=j["translate"].get<glm::vec3>();
    if(j.contains("rotate_angle")) item->rotate_angle=j["rotate_angle"].get<float>();
    if(j.contains("rotate_axis")) item->rotate_axis=j["rotate_axis"].get<glm::vec3>();
    if(j.contains("relative_id")) item->relative_id=j["relative_id"].get<std::string>();
    item->translate_model=glm::translate(glm::mat4(1.0f),item->translate);
    if(item->rotate_angle!=0) item->translate_model=glm::rotate(item->translate_model,glm::radians(item->rotate_angle),item->rotate_axis);
//    item->translate_model=glm::scale(item->translate_model,item->scale); //scale渲染时单独计算，否则有传染性
}
// SceneModel 的 JSON 反序列化
void from_json(const json &j,std::shared_ptr<SceneModel> &model){
    if(!model) model=std::make_shared<SceneModel>();
    auto base_ptr=std::static_pointer_cast<SceneItemBase>(model);
    from_json_base(j,base_ptr);
    if(j.contains("file_path")) model->file_path=j["file_path"].get<std::string>();
    if(j.contains("animation_index")) model->animation_index=j["animation_index"].get<int>();
    if(j.contains("is_static_model")) model->is_static_model=j["is_static_model"].get<bool>();
}
// SceneImage 的 JSON 反序列化
void from_json(const json &j,std::shared_ptr<SceneImage> &image){
    if(!image) image=std::make_shared<SceneImage>();
    auto base_ptr=std::static_pointer_cast<SceneItemBase>(image);
    from_json_base(j,base_ptr);
    if(j.contains("file_path")) image->file_path=j["file_path"].get<std::string>();
}
// SceneText 的 JSON 反序列化
void from_json(const json &j,std::shared_ptr<SceneText> &text){
    if(!text) text=std::make_shared<SceneText>();
    auto base_ptr=std::static_pointer_cast<SceneItemBase>(text);
    from_json_base(j,base_ptr);
    if(j.contains("text")) text->text=j["text"].get<std::string>();
    if(j.contains("color")) text->color=j["color"].get<glm::vec3>();
    if(j.contains("line_gap")) text->line_gap=j["line_gap"].get<float>();
}
// DetectType 的 JSON 反序列化
void from_json(const json &j,DetectType &dt){
    if(j.contains("id")) dt.marker_name=j["id"].get<std::string>();
    int dt_type=0; //0 is opencv detector, 1 is optimized_detector
    if(j.contains("type")){
        auto s=StringTrimmed(j["type"].get<std::string>());
        std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return std::tolower(c);});
        if(s=="optimized_detector") dt_type=1;
        else{
            dt_type=0;
            if(s=="dict_5x5_50") dt.aruco_type=cv::aruco::DICT_5X5_50;
        }
    }
    if(j.contains("size")) dt.aruco_size=j["size"].get<cv::Size>();
    if(j.contains("aruco_id")) dt.aruco_id=j["aruco_id"].get<int>();
    if(dt_type==0){
        dt.use_opencv_aruco_detector=true; dt.use_optimized_detector=false;
    }
    else if(dt_type==1){
        dt.use_opencv_aruco_detector=false; dt.use_optimized_detector=true;
    }
}

class AScene{  //读取 scene_list.txt 中的 Scene 列表，每项作为一个 SceneItem
private:
    std::string scene_name;  //Scene的名称，在scene_list.txt中指定
    std::map<std::string,std::shared_ptr<SceneItemBase>>    render_list; //要渲染的组件放在这里,key是组件的id
    std::vector<DetectType>                                 detect_list; //待检测的marker列表
    std::unordered_map<std::string,glm::mat4>               pose_map;    //保存<组件id-组件最新渲染/检测位姿>的映射
    std::vector<std::string>  render_order_list;  //顺序是组件的一个合法渲染顺序，更新render_list后要重新
    bool scene_initialized{false};
    SceneGui gui; //scene中的图像等
    std::unordered_map<std::string,Model> static_model_list; //不需要动画效果，但需要光照的模型
public:
    static void PrintScenePtr(const std::unique_ptr<AScene> &ptr){ //调试用
        infof("-------------------------------------------")
        if(!ptr){
            infof("ScenePtr 是空指针") return;
        }
        try{
            int count=0;
            for(const auto &i:ptr->render_list){
                std::shared_ptr<SceneItemBase> p=i.second;
                std::stringstream ss;
                std::string type_str="BaseItem";
                ss<<"id: "<<p->id<<std::endl;
                ss<<"scale: "<<Format("%f,%f,%f",p->scale.x,p->scale.y,p->scale.z)<<std::endl;
                ss<<"translate: "<<Format("%f,%f,%f",p->translate.x,p->translate.y,p->translate.z)<<std::endl;
                ss<<"rotation(axis,angle): "<<Format("[%f,%f,%f] %f",p->rotate_axis.x,p->rotate_axis.y,p->rotate_axis.z,p->rotate_angle)<<std::endl;
                ss<<"relative_id: "<<p->relative_id<<std::endl;
                if(p->type()==SceneItemType::ModelItem){
                    type_str="Model";
                    auto cp=std::static_pointer_cast<SceneModel>(p);
                    ss<<"model_file_path: "<<cp->file_path<<std::endl;
                }
                else if(p->type()==SceneItemType::ImageItem){
                    type_str="Image";
                    auto cp=std::static_pointer_cast<SceneImage>(p);
                    ss<<"image_file_path: "<<cp->file_path<<std::endl;
                }
                else if(p->type()==SceneItemType::TextItem){
                    type_str="Text";
                    auto cp=std::static_pointer_cast<SceneText>(p);
                    ss<<"text: "<<cp->text<<std::endl;
                    ss<<"text_color: "<<Format("(%f,%f,%f)",cp->color.x,cp->color.y,cp->color.z)<<std::endl;
                }
                infof("\n渲染列表第 %d 项, 类型: %s, 内容如下:\n%s",++count,type_str.c_str(),ss.str().c_str())
            }
            for(int i=0;i<(int)ptr->detect_list.size();++i){
                std::stringstream ss;
                auto dt=ptr->detect_list[i];
                ss<<"UseOptimized: "<<dt.use_optimized_detector<<std::endl;
                ss<<"UseOpenCV: "<<dt.use_opencv_aruco_detector<<std::endl;
                ss<<"ArucoInfo: "<<Format("Size: (%d,%d), ID: %d, Type: %d",dt.aruco_size.width,dt.aruco_size.height,dt.aruco_id,dt.aruco_type)<<std::endl;
                infof("\n检测列表第 %d 项, 名称: %s, 信息如下:\n%s",i+1,dt.marker_name.c_str(),ss.str().c_str())
            }
            std::string order_str;
            for(int i=0;i<(int)ptr->render_order_list.size();++i) order_str+=(i?", "+ptr->render_order_list[i]:ptr->render_order_list[i]);
            infof("\n渲染顺序: %s",order_str.c_str());
        }
        catch(...){infof("打印ScenePtr时出现异常")}
    }
    static std::unique_ptr<AScene> CreateScene(const std::string &name,const std::string &scene_dir){
        if(name.find_first_of("/\\")!=std::string::npos){
            errorf("无法从本地文件中构造Scene: %s(目录: %s), 因为名称中含有非法字符",name.c_str(),scene_dir.c_str())
            return nullptr;
        }
        auto json_path=scene_dir+"/scene.json";
        std::ifstream file(json_path);
        if(!file.is_open()){
            errorf("无法加载json文件,路径不存在: %s",json_path.c_str())
            return nullptr;
        }
        auto ptr=std::make_unique<AScene>();
        try{
            json j;
            file>>j;
            if(j.contains("models")){
                auto v=j["models"].get<std::vector<std::shared_ptr<SceneModel>>>();
                for(const auto &i:v) ptr->render_list.emplace(i->id,i);
            }
            if(j.contains("images")){
                auto v=j["images"].get<std::vector<std::shared_ptr<SceneImage>>>();
                for(const auto &i:v) ptr->render_list.emplace(i->id,i);
            }
            if(j.contains("texts")){
                auto v=j["texts"].get<std::vector<std::shared_ptr<SceneText>>>();
                for(const auto &i:v) ptr->render_list.emplace(i->id,i);
            }
            if(j.contains("markers")){
                ptr->detect_list=j["markers"].get<std::vector<DetectType>>();
            }
            ptr->calc_render_order();
            ptr->init_scene();
        }
        catch(...){
            errorf("无法从本地文件中构造Scene: %s(目录: %s), 捕获到异常",name.c_str(),scene_dir.c_str())
            ptr.reset();
            return nullptr;
        }
        PrintScenePtr(ptr);
        return ptr;
    }
private:
    bool calc_render_order(){ //根据拓扑排序计算组件的渲染顺序
        std::unordered_map<std::string,std::vector<std::string>> G;
        std::unordered_map<std::string,int> ind; //存储每个id的入度
        std::queue<std::string> q;
        for(const auto &i:this->render_list) ind[i.second->id]=0;
        for(const auto &i:this->render_list){
            if(id_in_render_list(i.second->relative_id)){
                ++ind[i.second->id];
                G[i.second->relative_id].push_back(i.second->id);
            }
        }
        for(const auto &i:ind) if(i.second==0&&id_in_render_list(i.first)) q.push(i.first);
        bool flag=true;
        std::vector<std::string> res(render_list.size());
        int index=0;
        while(!q.empty()&&flag){
            auto u=q.front(); q.pop();
            if(index>=(int)res.size()){
                flag=false; break;
            }
            res[index++]=u;
            for(const std::string &v:G[u]){
                if(--ind[v]==0) q.push(v);
                if(ind[v]<0){
                    flag=false; break;
                }
            }
        }
        if(flag) render_order_list=res;
        else errorf("计算渲染顺序失败")
        return flag;
    }
public:
    void init_scene(){ //只在构造时调用一次
        if(scene_initialized) return;
        for(const auto &i:this->render_list){
            std::shared_ptr<SceneItemBase> p=i.second;
            if(p->type()==SceneItemType::ModelItem){
                auto cp=std::static_pointer_cast<SceneModel>(p);
                if(!cp->is_static_model) ModelManager::Manager().add_model_from_file(cp->id,cp->file_path);
                else{
                    auto _model=Model::GetModelFromLocalFile(cp->id,cp->file_path);
                    _model.initialize();
                    static_model_list.emplace(cp->id,_model);
                }
            }
            else if(p->type()==SceneItemType::ImageItem){
                auto cp=std::static_pointer_cast<SceneImage>(p);
                SceneGui::ImageItem item;
                item.image=ConvertBGRImageToRGB(cv::imread(cp->file_path));
                item.scale=cp->scale; item.translate_model=cp->translate_model;
                gui.add_image(item,cp->id);
            }
            else if(p->type()==SceneItemType::TextItem){
                auto cp=std::static_pointer_cast<SceneText>(p);
                SceneGui::TextItem item;
                item.text=cp->text; item.color=cp->color;
                item.scale=cp->scale; item.translate_model=cp->translate_model;
                gui.add_text(item,cp->id);
            }
        }
        //初始化pose_map
        for(const auto &i:this->render_list) this->pose_map[i.second->id]=glm::mat4(1.0f);
        for(const auto &i:this->detect_list) this->pose_map[i.marker_name]=glm::mat4(1.0f);
        scene_initialized=true;
    }
    bool init_detector(){ //将ARModule的检测列表设置为当前Scene需要检测的Aruco列表，在进入当前Scene时执行一次
        ArucoDetectListMutex.lock();
        ArucoDetectList.clear();
        for(const auto &i:this->detect_list){
            ArucoDetectList.push_back(i);
        }
        ArucoDetectListMutex.unlock();
        return true;
    }
    void inputEvent(int leftright,const ApplicationEvent &event){

    }
    void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection){

    }
    void render(const XrPosef &pose,const glm::mat4 &project,const glm::mat4 &view,int32_t eye){
        for(const auto &name:render_order_list){
            auto _it=render_list.find(name);
            if(_it==render_list.end()) continue;
            std::shared_ptr<SceneItemBase> p=_it->second;
            glm::mat4 render_pose=(p->relative_id.empty()?glm::mat4(1.0f):pose_map[p->relative_id]);
            render_pose=render_pose*p->translate_model;
//            render_pose=p->translate_model*render_pose;
            pose_map[p->id]=render_pose; //更新pose_map
            glm::mat4 render_m=glm::scale(render_pose,p->scale);

            if(p->id=="axle_pose"){
                auto refined_ptr=DataBank::Global().get<glm::mat4>("axle_pose");
                if(refined_ptr){
                    render_m=*refined_ptr;
                    pose_map[p->id]=glm::translate(glm::mat4(1.0f),glm::vec3(render_m[3]));
                    infof("refined: %s",GlmMat4_to_String(pose_map[p->id]).c_str())
                }
            }
            if(p->type()==SceneItemType::ModelItem){
                auto cp=std::static_pointer_cast<SceneModel>(p);
                if(!cp->is_static_model){
                    ModelManager::Manager().make_model_valid(cp->id);
                    ModelManager::Manager().render_model(cp->id,project,view,render_m);
                    if(cp->animation_index==0) ModelManager::Manager().control_model_all_animations(cp->id,true);
                    else if(cp->animation_index>0) ModelManager::Manager().start_model_animation(cp->id,cp->animation_index-1);
                    else if(cp->animation_index==-1){
                        ModelManager::Manager().start_model_animation(cp->id,0);
                        ModelManager::Manager().stop_model_animation(cp->id,0);
                    }
                }
                else{
                    auto _it_m=static_model_list.find(cp->id);
                    if(_it_m!=static_model_list.end()) _it_m->second.render(project,view,render_m);
                }
            }
            else if(p->type()==SceneItemType::ImageItem){
                auto cp=std::static_pointer_cast<SceneImage>(p);
                gui.set_translate_model(cp->id,render_m);
                gui.render_by_id(cp->id,project,view,eye);
            }
            else if(p->type()==SceneItemType::TextItem){
                auto cp=std::static_pointer_cast<SceneText>(p);
                if(cp->id=="title"||cp->id=="desc"){
                    if(!cp->text.empty()){
                        gui.set_translate_model(cp->id,render_m*ModelAdjust);
                        gui.render_by_id(cp->id,project,view,eye);
                    }
                }
                else{ //正常的渲染流程
                    if(!cp->text.empty()){ //空文本不渲染,提高效率
                        gui.set_translate_model(cp->id,render_m);
                        gui.render_by_id(cp->id,project,view,eye);
                    }
                }
            }
        }
    }
    bool id_in_render_list(const std::string &id){
        return render_list.find(id)!=render_list.end();
    }
    bool set_detect_result(const std::string &id,const glm::mat4 &model){
        auto it=pose_map.find(id);
        if(it==pose_map.end()) return false;
        glm::vec3 model_translation=glm::vec3(model[3]);
        //glm::mat3 rotationMat3=glm::mat3(model); // 提取旋转（移除平移部分）注意：如果矩阵有非均匀缩放，这会包含缩放信息
        it->second=glm::translate(glm::mat4(1.0f),model_translation);
        return true;
    }
    std::vector<std::string> get_detect_id_list()const{
        std::vector<std::string> res;
        for(const auto &i:detect_list) res.push_back(i.marker_name);
        return res;
    }
};


class SceneModelEdit:public IScene{
    std::shared_ptr<ARApp> _eng;

    std::string mSceneListFilePath=Config::AppDataDir+"/Scene/scene_list.txt"; //scene_list.txt的可直接访问的路径
    inline static std::vector<std::unique_ptr<AScene>> mSceneList;
    inline static bool mSceneLoaded{false};
    int mCurrentScene{-1};

    glm::mat4 mProject{},mView{};

    SceneGui gui; //显示文本

    int mEditType{0}; //是否启用了手势控制,以及控制的是什么。0:未启用; 1、2、3: 平移、旋转、缩放

    glm::vec3 mAdjustTranslate{0};
    glm::vec3 mAdjustScale{1,1,1};
    glm::vec3 mAdjustRotate{0,0,0}; //在x,y,z轴上的旋转幅度

    bool mAllowGesture{true}; //当前是否响应手势控制

public:
    bool initialize(const XrInstance instance,const XrSession session)override{
        _eng=construct_engine();
        _eng->start();
        if(!mSceneLoaded){
            mSceneLoaded=load_scene_list(mSceneListFilePath);
        }
        SceneGui::TextItem info_text;
        info_text.translate_model=glm::translate(glm::mat4(1.0f),{-0.3,-0.1,-2});
        info_text.scale={0.25,0.3,0.3};
        info_text.use_view=false;
        gui.add_text(info_text,"info");
        return true;
    }
    bool load_scene_list(const std::string &scene_list_file_path){
        std::ifstream file;
        file.open(scene_list_file_path);
        if(!file.is_open()){
            return false;
        }
        std::string scene_name;
        int count=0; //只加载前5个
        while(std::getline(file,scene_name)){
            scene_name=StringTrimmed(scene_name);
            if(scene_name.empty()) continue; //空行跳过
            if(++count>5) break;
            //把scene_name当作文件夹名替换scene_list_file_path中的文件路径
            auto last_pos=scene_list_file_path.find_last_of("/\\");
            std::string scene_dir=(last_pos==std::string::npos?scene_name:scene_list_file_path.substr(0,last_pos+1)+scene_name);
            auto ptr=AScene::CreateScene(scene_name,scene_dir);
            if(ptr){
                mSceneList.push_back(std::move(ptr));
                infof("成功从本地文件中构造Scene: %s, 目录: %s",scene_name.c_str(),scene_dir.c_str())
            }
            else errorf("从本地文件中构造Scene失败, name: %s, dir: %s",scene_name.c_str(),scene_dir.c_str())
        }
        // 关闭文件
        file.close();
        if(!mSceneList.empty()){
            mCurrentScene=0;
            scene_setup();
        }
        return true;
    }
    void reset_adjust(){
        mAdjustTranslate={0,0,0}; mAdjustScale={1,1,1}; mAdjustRotate={};
    }
    void inputEvent(int leftright, const ApplicationEvent &event)override{
        if(mSceneList.empty()) return;
        mSceneList[mCurrentScene]->inputEvent(leftright,event);
    }
    void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection)override{
        if(mSceneList.empty()) return;
        mSceneList[mCurrentScene]->rayEvent(linePoint,lineDirection);
    }
    void keypadEvent(const std::string &key_name)override{
        static std::map<std::string_view,long long> LastPressedMap; //某个按键最后一次被触发是什么时候,防止反复触发
        static long long MinGap=600; //最小触发间隔为200ms
        if(CurrentMSecsSinceEpoch()-LastPressedMap[key_name]<MinGap) return;
        LastPressedMap[key_name]=CurrentMSecsSinceEpoch();

        if(key_name=="o"){  //开启或关闭手势控制
            if(mEditType!=0) mEditType=0;
            else mEditType=1; //从平移开始
        }
        else if(key_name=="right"){ //切换控制的是 平移/旋转/缩放
            if(mEditType!=0){
                mEditType=(mEditType)%3+1;
            }
        }
        else if(key_name=="left"){
            mAllowGesture=!mAllowGesture;
//            if(mEditType!=0){
//                mEditType=(mEditType-2)%3+1;
//            }
        }
        else if(key_name=="select") reset_adjust();
    }
    void handleGesture(){
        auto info_text=gui.get_text_item("info");
        if(info_text){
            if(mEditType==0) info_text->text="按O键可以进入手势编辑模式";
            else{
                std::string s="当前处于编辑模式，通过手势可控制模型的 ";
                if(mEditType==1) s+="平移";
                else if(mEditType==2) s+="旋转";
                else if(mEditType==3) s+="缩放";
                info_text->text=s;
            }
        }
        static glm::vec3 LastPos;
        auto data_ptr=DataBank::Global().get<glm::vec3>("wrist_position");
        if(!data_ptr) return;
        auto wrist_pos=*data_ptr;
        infof("gesture position: %f, %f, %f",wrist_pos.x,wrist_pos.y,wrist_pos.z)
        float dx=wrist_pos.x-LastPos.x,dy=wrist_pos.y-LastPos.y,dz=wrist_pos.z-LastPos.z;
        if(std::max(std::abs(dx),std::max(std::abs(dy),std::abs(dz)))>0.02){ //一帧移动超过2cm认为异常数据，不作处理
            LastPos=wrist_pos; return;
        }
        if(dx==0&&dy==0&&dz==0) return;
        if(mEditType==0) return;
        if(!mAllowGesture) return;
        if(mEditType==1){  //平移
            float TranslationLimit=3.f;
            mAdjustTranslate.x=std::clamp(mAdjustTranslate.x+dx*0.2f,-TranslationLimit,TranslationLimit);
            mAdjustTranslate.y=std::clamp(mAdjustTranslate.y+dy*0.2f,-TranslationLimit,TranslationLimit);
            mAdjustTranslate.z=std::clamp(mAdjustTranslate.z+dz*0.2f,-TranslationLimit,TranslationLimit);
        }
        else if(mEditType==2){  //旋转
            float RotationLimit=90.f;
            mAdjustRotate.x=std::clamp(mAdjustRotate.x+dx*20,-RotationLimit,RotationLimit);
//            mAdjustRotate.y=std::clamp(mAdjustRotate.y+dy*20,-RotationLimit,RotationLimit);
            //mAdjustRotate.z+=dz;
        }
        else if(mEditType==3){  //缩放
            float ScaleUpLimit=5.f,ScaleDownLimit=0.1f;
            mAdjustScale+=dy;
            if(mAdjustScale.x<ScaleDownLimit) mAdjustScale={ScaleDownLimit,ScaleDownLimit,ScaleDownLimit};
            if(mAdjustScale.x>ScaleUpLimit) mAdjustScale={ScaleUpLimit,ScaleUpLimit,ScaleUpLimit};
        }
    }
    glm::mat4 get_adjust(){
        glm::mat4 res=glm::translate(glm::mat4(1.0f),mAdjustTranslate);
        glm::mat4 rotation_part=res; rotation_part[3]=glm::vec4(0.0f,0.0f,0.0f,1.0f); //从res提取旋转
        // 创建局部旋转矩阵
        glm::mat4 local_rotation=glm::rotate(glm::mat4(1.0f),glm::radians(mAdjustRotate.x),{0,0,1});
        local_rotation=glm::rotate(local_rotation,glm::radians(mAdjustRotate.y),{0,1,0});
        // 应用旋转：先局部旋转，再应用之前的旋转
        glm::mat4 new_rotation=rotation_part*local_rotation;
        // 提取位置 恢复位置
        new_rotation[3]=res[3];
        res=new_rotation;
        return glm::scale(res,mAdjustScale);
    }
    void scene_finished(){ //对mCurrentScene执行退出操作
        if(mCurrentScene<0||mCurrentScene>=(int)mSceneList.size()) return;
    }
    void scene_setup(){ //对mCurrentScene执行初始化操作
        if(mCurrentScene<0||mCurrentScene>=(int)mSceneList.size()) return;
        mSceneList[mCurrentScene]->init_detector();
    }
    int changeStep(int step,bool real_change)override{
        if(mCurrentScene==-1) return step;
        if(mCurrentScene+step>=(int)mSceneList.size()) return 1;
        if(mCurrentScene+step<0) return -1;
        if(real_change){
            scene_finished();
            mCurrentScene+=step;
            scene_setup();
        }
        return 0;
    }
    //@id: -1表示不要求MarkerId,返回列表中的第一个
    MarkerPose::ItemType get_pose(const std::string &name,int id=-1){
        auto _res=_eng->sceneData->getData(name);
        if(_res.has_value()){
            auto res = std::any_cast<MarkerPose>(_res);
            for(const auto &marker:res.markers){
                auto [marker_id,marker_pose,rvec,tvec]=marker;
                //也可以判断一下marker id,看看是不是设置的marker
                if(marker_id!=id&&id!=-1) continue;
                return marker;
            }
        }
        return MarkerPose::ItemType{};
    }
    virtual void renderFrame(const XrPosef &pose,const glm::mat4 &project,const glm::mat4 &view,int32_t eye){
        if(mSceneList.empty()) return;
        mProject=project; mView=view;
        handleGesture();
        gui.render(project,view,eye);
        ModelAdjust=get_adjust();
        if(_eng){
            auto id_list=mSceneList[mCurrentScene]->get_detect_id_list();
            for(const auto &id:id_list){
                auto [marker_id,marker_pose,rvec,tvec]=get_pose(id,-1);
                if(marker_pose==glm::mat4(0)) continue;
                bool flag=mSceneList[mCurrentScene]->set_detect_result(id,marker_pose);
                infof("Set Detect Result %s.[%s]: %s",(flag?"Success":"Fail"),id.c_str(),GlmMat4_to_String(marker_pose).c_str())
            }
        }
        mSceneList[mCurrentScene]->render(pose,project,view,eye);
    }
    virtual void close() {
        if(_eng) _eng->stop();
    }
};
}

std::shared_ptr<IScene> _createSceneModelEditTest(){
    return std::make_shared<SceneModelEdit>();
}