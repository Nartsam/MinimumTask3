#ifndef ROKIDOPENXRANDROIDDEMO_HUMANDETECTOR_H
#define ROKIDOPENXRANDROIDDEMO_HUMANDETECTOR_H

#pragma once

#include<opencv2/opencv.hpp>
#include<string>
#include<thread>
#include<mutex>
#include"glm/glm.hpp"

class CubeRender;
class HumanDetector{
public:
    void set_camera_mat(const cv::Matx33f &camera_mat);
    bool camera_mat_valid()const;
    bool start_server(const std::string &ip,int port);
    bool server_started()const;
    bool detect(const cv::Mat &image);

    void render(const glm::mat4 &project,const glm::mat4 &view);


    static HumanDetector& Global();
private:
    HumanDetector();
    ~HumanDetector();

    void thread_main(const std::string &ip,int port);


    cv::Matx33f mCameraMat{};
    std::atomic<bool> mCameraMatValid{false};
    std::atomic<bool> mServerStarted{false};
    std::atomic<bool> mServerIsStarting{false};

    std::unique_ptr<CubeRender> mCubeRender;
    std::vector<glm::mat4> mJointsPose;
    std::mutex mResultMutex;

    cv::Mat mImage;
    std::mutex mImageMutex;
    std::atomic<bool> mImageProcessed{true}; //mImage是否已经被发送到服务器进行推理

    std::thread mWorker;
    std::atomic<bool> mThreadStop{false};
};


#endif //ROKIDOPENXRANDROIDDEMO_HUMANDETECTOR_H
