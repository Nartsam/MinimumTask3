#include<thread>
#include"RPCServer.h"
//#include"ObjectTracking/include/ObjectTrackingServer.h"
#include"Basic/include/BasicData.h"
#include<iostream>


#include"demos/utils.h"
#include"config.hpp"
#include<dirent.h>
#include"databank.hpp"
#include"utilsmym.hpp"
using namespace cv;





class TestPro1Server:public ARModuleServer{
public:
    /*
    * 服务器同时管理多个应用，每个应用可有多个连接（多个端设备同时运行一个应用）。
    * 有新的连接时，服务器会调用create创建一个Server模块的新的对象，处理该连接的事件。
    */
    virtual ARModuleServerPtr create()
    {
        return std::make_shared<TestPro1Server>();
    }

    //在create之后会调用init初始化server对象
    virtual int init(RPCServerConnection& con)
    {
        printf("%s. TestPro1Server: init...\n", con.name.c_str());

        return STATE_OK;
    }

    //处理对应连接的消息
    virtual int call(RemoteProcPtr proc, FrameDataPtr frameDataPtr, RPCServerConnection& con) override
    {
        auto& send = proc->send;
        auto cmd = send.getd<std::string>("cmd");

        printf("%s.TestPro1Server: call cmd=%s, frameID=%d\n", con.name.c_str(), cmd.c_str(), frameDataPtr->frameID);

        Mat img = frameDataPtr->image.front().clone();
        CV_Assert(img.size()==Size(640,480) && img.at<uchar>(0, 0) == uchar(frameDataPtr->frameID % 256)); //验证输入图像大小和像素值
        CV_Assert(proc->frameDataPtr->frameID == frameDataPtr->frameID);

        if (cmd == "set")
        {
            int val = send.getd<int>("val", 0);
            img = uchar(val % 256);

            //设置返回结果，随后可以在ARModule::ProRemoteReturn中接收并进行处理
            proc->ret = {
                    {"result",Image(img,".png")}
            };
        }

        if (cmd == "add")
        {
            int val = send.getd<int>("val", 0);

            img = uchar((img.at<uchar>(0,0)+val) % 256);

            //设置返回结果，随后可以在ARModule::ProRemoteReturn中接收并进行处理
            proc->ret = {
                    {"result",Image(img,".png")}
            };

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return STATE_OK;
    }
};

class AppCommandServer:public ARModuleServer{
public:
    virtual ARModuleServerPtr create()
    {
        return std::make_shared<AppCommandServer>();
    }

    virtual int init(RPCServerConnection& con)
    {
        printf("%s. AppCommandServer: init...\n", con.name.c_str());

        return STATE_OK;
    }
    virtual int call(RemoteProcPtr proc, FrameDataPtr frameDataPtr, RPCServerConnection& con) override
    {
        auto& send = proc->send;
        auto cmd = send.getd<std::string>("cmd");

        printf("%s.AppCommandServer: call cmd=%s\n", con.name.c_str(), cmd.c_str());

        if (cmd == "set_align")
        {
            Matx34f obj_to_marker_34=send.get<Matx34f>("obj_to_marker");
            cv::Matx44f obj_to_marker;
            for(int i=0;i<3;++i)
                for(int j=0;j<4;++j)
                    obj_to_marker(i,j)=obj_to_marker_34(i,j);
            obj_to_marker(3,0)=0.0f; obj_to_marker(3,1)=0.0f; obj_to_marker(3,2)=0.0f; obj_to_marker(3,3)=1.0f;
            DataBank::Global().set("obj_to_marker",obj_to_marker);
            Config::global().set("ReceivedObjectToMarker",GlmMat4_to_String(CV_Matx44f_to_GLM_Mat4(obj_to_marker),',',true));
            Config::global().save();

            std::stringstream _ss; _ss<<std::endl<<obj_to_marker<<std::endl;
            infof("App Command Server Mat:%s",_ss.str().c_str())
            proc->ret = {
                    {"msg",std::string("ok")}
            };
        }
        return STATE_OK;
    }
};




void AppCommandServerMain(int port){
    try{
        ARServerManager &manager=ARServerManager::instance();
        // appDir中的每个子文件夹对应一个app，文件夹的名称就是app名称（必须和客户端的appName一致）
        manager.start(Config::AppDataDir+"/AREngine/App");
        //manager.registerModule(createModule<ObjectTrackingServer>("ObjectTracking"));

        manager.registerModule(createModule<TestPro1Server>("TestPro1")); //和客户端的模块名称一致，注意是"TestPro1"，不是"TestPro1Server"
        manager.registerModule(createModule<AppCommandServer>("AppCommand"));

        runRPCServer(port);
    }
    catch(...){
        errorf("AppCommandServerMain Error")
    }
}