#include"Basic/include/RPC.h"
#include"Basic/include/ARModule.h"
#include"Basic/include/App.h"
#include"opencv2/core.hpp"
#include"arengine.h"
#include"scene.h"
#include"demos/utils.h"
#include"demos/model.h"
#include"demos/cylinder.h"
#include"demos/text.h"
#include"demos/plane.h"
#include"utilsmym.hpp"
#include"arucopp.h"
#include"SceneGui.h"
#include<glm/gtc/type_ptr.hpp>   // glm::make_mat4
#include<glm/gtc/matrix_transform.hpp>
#include<set>

using namespace cv;

namespace{
// 在 PlaneDetector::Update() 中传递给渲染线程的数据
struct PlaneDetectorData{
    float eyeHeight; // 眼镜在世界坐标系中的 Y 高度（米）
};

class PlaneDetector: public ARModule{
public:
    int Init(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        return STATE_OK;
    }
    int Update(AppData &appData,SceneData &sceneData,FrameDataPtr frameDataPtr){
        if(!frameDataPtr->image.empty()){
            cv::Mat img=frameDataPtr->image.front();
            auto frame_data=std::any_cast<ARInputSources::FrameData>(sceneData.getData("ARInputs"));
            cv::Matx44f vmat=frame_data.cameraMat;
            const cv::Matx33f &camK=frameDataPtr->colorCameraMatrix;

            // 从相机位姿矩阵提取眼镜当前的 Y 轴高度（世界坐标系，单位：米）
            // cameraMat 为相机到世界的变换矩阵，第 3 列（index 7）即 Y 方向平移
            float eyeHeight=vmat(1,3);

            // 将眼镜高度写入 sceneData，供渲染线程（ScenePlaneDetector::renderFrame）读取
            PlaneDetectorData data;
            data.eyeHeight=eyeHeight;
            sceneData.setData("PlaneDetectorData",data);
        }
        return STATE_OK;
    }
};

std::shared_ptr<ARApp> construct_engine(){
    std::string appName="TestApp"; //APP名称，必须和服务器注册的App名称对应（由服务器上appDir中文件夹的名称确定）

    std::vector<ARModulePtr> modules;
    modules.push_back(createModule<ARInputs>("ARInputs"));
    modules.push_back(createModule<PlaneDetector>("PlaneDetector"));

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

class ScenePlaneDetector: public IScene{
    std::shared_ptr<ARApp> _eng;
    SceneGui gui;
    float user_height=2.2; //在低于眼镜高度的位置渲染一个平面
    float height_step=0.08; //按 上/下 键的时候 抬高/降低 平面的高度
    std::mutex mModelMutex;
    Model mModel{""}; //要在平面上渲染的模型
    glm::vec3 mModelScale{1,1,1};
    glm::mat4 mModelAdjust=glm::mat4(1.0f);

    PlaneRender mPlane;           // 地面平面渲染器
    bool mPlaneInitialized=false; // 延迟初始化标记（需要在 GL 线程中初始化）

    glm::vec3 mRayIntersection{1e6f,1e6f,1e6f}; // 当前帧射线与平面的交点（无交点时为远处）
    bool mHasIntersection=false;                  // 当前帧射线是否与平面相交
    std::mutex mRayMutex;

    bool mModelVisible=false;        // 是否在交点处渲染 mModel
    glm::vec3 mModelPosition{0,0,0}; // mModel 放置的世界坐标位置

    // 当前地面 Y 坐标（眼镜高度 - 用户身高），由 renderFrame 每帧更新
    float mGroundY=0.0f;

public:
    bool set_render_model(const std::string &file_path,const glm::vec3 &model_scale={1.0f,1.0f,1.0f},const glm::mat4 &model_adjust=glm::mat4(1.0f)){
        std::lock_guard<std::mutex> locker(mModelMutex);
        mModel=Model::GetModelFromLocalFile("TestModel",MakeSdcardPath(file_path));
        mModelScale=model_scale;
        mModelAdjust=model_adjust;
        return true;
    }
//====================================================================

    bool initialize(const XrInstance instance,const XrSession session) override{
        _eng=construct_engine();
        _eng->start();
        //============ Init SceneGui =============
        SceneGui::TextItem InfoText;
        InfoText.translate_model=glm::translate(glm::translate(glm::mat4(1.0f),glm::vec3(-0.0f,0.1f,-4.0f)),{-0.66f,-0.5f,0.f});
        InfoText.scale={1.6f,1.6f,1.6f};
        InfoText.use_view=true;
        InfoText.text="平面检测测试";
        gui.add_text(InfoText,"InfoText");
        //================= 加载模型 ===================
        set_render_model("/storage/emulated/0/RokidData/Model/RM/axle/axle.obj",
                         {0.159,0.159,0.159},
                         glm::rotate(mModelAdjust,glm::radians(-90.0f),glm::vec3(1,0,0)));
        return true;
    }
    void inputEvent(int leftright,const ApplicationEvent &event) override{
        gui.inputEvent(leftright,event);
    }
    void keypadEvent(const std::string &key_name) override{
        static std::map<std::string_view,long long> LastPressedMap; //某个按键最后一次被触发是什么时候,防止反复触发
        static long long MinGap=500; //最小触发间隔为800ms
        if(CurrentMSecsSinceEpoch()-LastPressedMap[key_name]<MinGap) return;
        LastPressedMap[key_name]=CurrentMSecsSinceEpoch();
        if(key_name=="o"){
        }
        else if(key_name=="up"){ //上键：升高平面（减小用户身高偏移）
            user_height-=height_step;
        }
        else if(key_name=="down"){ //下键：降低平面（增大用户身高偏移）
            user_height+=height_step;
        }
        else if(key_name=="select"){ // 显示 或 隐藏 mModel
            std::lock_guard<std::mutex> locker(mRayMutex);
            if(mHasIntersection){
                // 仅当射线与平面相交时，切换模型可见状态，并记录放置位置
                mModelVisible=!mModelVisible;
                if(mModelVisible){
                    mModelPosition=mRayIntersection;
                }
            }
        }
    }
    void rayEvent(glm::vec3 linePoint,glm::vec3 lineDirection) override{
        // 射线与水平平面（Y = mGroundY）求交
        // 平面方程：Y = mGroundY，法向量 N = (0,1,0)
        // 直线参数方程：P = linePoint + t * lineDirection
        // 求解 t：(linePoint.y + t * lineDirection.y) = mGroundY
        //         t = (mGroundY - linePoint.y) / lineDirection.y
        std::lock_guard<std::mutex> locker(mRayMutex);
        mHasIntersection=false;
        if(fabs(lineDirection.y)<1e-6f){
            // 射线平行于地面，无交点
            return;
        }
        float t=(mGroundY-linePoint.y)/lineDirection.y;
        if(t<=0.0f){
            // 交点在射线反方向，不计入
            return;
        }
        glm::vec3 point=linePoint+t*lineDirection;

        // 判断交点是否在平面范围内（[-halfExtent, halfExtent] × [-halfExtent, halfExtent]）
        float halfExtent=mPlane.getHalfExtent();
        // 平面中心为 (0, mGroundY, 0)
        if(fabs(point.x)<=halfExtent&&fabs(point.z)<=halfExtent){
            mHasIntersection=true;
            mRayIntersection=point;
        }
    }
    void update_gui(){
        auto InfoText=gui.get_text_item("InfoText");
        if(!InfoText) return;
    }
    virtual void renderFrame(const XrPosef &pose,const glm::mat4 &project,const glm::mat4 &view,int32_t eye){
        update_gui();
        gui.render(project,view,eye);

        // 延迟初始化平面（必须在 GL 渲染线程中调用）
        if(!mPlaneInitialized){
            mPlane.initialize();
            mPlaneInitialized=true;
        }

        float eyeHeight=0.0f;
        if(_eng){
            // 从 sceneData 取出 PlaneDetector 设置的眼镜高度
            auto _res=_eng->sceneData->getData("PlaneDetectorData");
            if(_res.has_value()){
                auto data=std::any_cast<PlaneDetectorData>(_res);
                eyeHeight=data.eyeHeight;
            }
        }

        // 地面 Y 坐标 = 眼镜高度 - 用户身高
        mGroundY=eyeHeight-user_height;

        // 构造平面的 model 矩阵：
        //   1. 先缩放至 halfExtent 大小（局部顶点范围 [-1,1]）
        //   2. 再平移到地面高度 (0, mGroundY, 0)
        float halfExtent=mPlane.getHalfExtent();
        glm::mat4 planeModel=glm::mat4(1.0f);
        planeModel=glm::translate(planeModel,glm::vec3(0.0f,mGroundY,0.0f));
        planeModel=glm::scale(planeModel,glm::vec3(halfExtent,1.0f,halfExtent));

        // 读取射线交点（加锁保证线程安全）
        glm::vec3 intersection;
        bool hasIntersection;
        {
            std::lock_guard<std::mutex> locker(mRayMutex);
            intersection=mRayIntersection;
            hasIntersection=mHasIntersection;
        }

        // 渲染半透明地面平面，并在交点处显示白色圆点
        mPlane.render(project,view,planeModel,hasIntersection,intersection);

        //渲染模型
        mModelMutex.lock();
        if(mModelVisible){
            // 将模型放置在交点处（平面上），Y 坐标取地面高度保证贴地
            glm::mat4 modelMat=glm::mat4(1.0f);
            modelMat=glm::translate(modelMat,glm::vec3(mModelPosition.x,mGroundY,mModelPosition.z));
            modelMat=glm::scale(modelMat*mModelAdjust,mModelScale);
            mModel.render(project,view,modelMat);
        }
        mModelMutex.unlock();
    }
    virtual void close(){
        if(_eng) _eng->stop();
    }
};
}

std::shared_ptr<IScene> _createScenePlaneDetector(){
    return std::make_shared<ScenePlaneDetector>();
}
