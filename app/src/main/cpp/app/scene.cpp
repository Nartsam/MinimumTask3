#include "scene.h"


std::shared_ptr<IScene> createScene(const std::string &name, IApplication *app){
    std::shared_ptr<IScene> _createScene_empty();
    std::shared_ptr<IScene> _createScene_marker_test_rpc();
    std::shared_ptr<IScene> _createScene_3dtracking_test();
    std::shared_ptr<IScene> _createSceneTask3Step1();
    std::shared_ptr<IScene> _createSceneTask3Welcome();
    std::shared_ptr<IScene> _createSceneCameraTrackingTest();
    std::shared_ptr<IScene> _createSceneCustomer();
    std::shared_ptr<IScene> _createSceneModelEditTest();
    std::shared_ptr<IScene> _createSceneRecorder();

    struct DFunc{
        std::string name;
        typedef std::shared_ptr<IScene> (*createFuncT)();
        createFuncT  create;
    };

    DFunc funcs[]= {
            {"empty", _createScene_empty},
            {"marker_test_rpc", _createScene_marker_test_rpc},
            {"3dtracking_test",_createScene_3dtracking_test},
            {"task3_step1",_createSceneTask3Step1},
            {"task3_welcome",_createSceneTask3Welcome},
            {"scene_customer",_createSceneCustomer},
            {"camera_tracking_test",_createSceneCameraTrackingTest},
            {"model_edit_test",_createSceneModelEditTest},
            {"scene_recorder",_createSceneRecorder},
            //list other scenes
    };

    std::shared_ptr<IScene> ptr=nullptr;
    for(auto &f : funcs){
        if(f.name==name) {
            ptr = f.create();
            break;
        }
    }
    if(ptr){
        ptr->m_app=app;
        ptr->m_program=app? app->m_program : nullptr;
    }

    return ptr; //原来是return nullptr,可能是写错了
}


class SceneEmpty
        :public IScene
{
public:
    virtual bool initialize(const XrInstance instance, const XrSession session) {
        return true;
    }
};

std::shared_ptr<IScene> _createScene_empty()
{
    return std::make_shared<SceneEmpty>();
}

