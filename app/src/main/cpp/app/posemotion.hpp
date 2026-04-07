#pragma once
#ifndef ROKIDOPENXRANDROIDDEMO_POSEMOTION_HPP
#define ROKIDOPENXRANDROIDDEMO_POSEMOTION_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtx/matrix_decompose.hpp>
#include<glm/gtc/quaternion.hpp>
#include<glm/gtx/quaternion.hpp>
#include"utilsmym.hpp"


/*
 * 设定 @start_pose 和 @end_pose, 定义动画持续时间 @duration (s), 调用 start() 启动动画
 * 在任意时间调用 get_pose() 即可得到当前的位姿
*/


class PoseMotion{
public:
    enum MotionMode{  //动画模式
        Clamp,      // 超时后保持结束位姿
        Loop,       // 超时后从头循环
        PingPong,   // 超时后往返循环
    };
    PoseMotion()=default;
    PoseMotion(const glm::mat4 &start_pose,const glm::mat4 &end_pose,double duration_sec,MotionMode mode=Clamp):mDuration(duration_sec),mMode(mode){
        set_start_pose(start_pose);
        set_end_pose(end_pose);
    }
    glm::mat4 get_pose()const{
        if(!mPlaying) return mStartPose;
        if(mDuration<=0) return mEndPose;
        double t=double(CurrentMSecsSinceEpoch()-mStartTimestamp)/1000.0;
        double alpha=computeAlpha(t);
        // 平移插值
        glm::vec3 trans=glm::mix(mStartTrans,mEndTrans,alpha);
        // 旋转插值
        glm::quat rot=glm::slerp(mStartRot,mEndRot,(float)alpha);
        // 缩放插值
        glm::vec3 scale=glm::mix(mStartScale,mEndScale,alpha);
        // 组合矩阵
        return glm::translate(glm::mat4(1.0f),trans)*glm::toMat4(rot)*glm::scale(glm::mat4(1.0f),scale);
    }
    glm::mat4 get_relative_pose()const{
        glm::mat4 current=get_pose();
        return glm::inverse(mStartPose)*current;
    }
    bool set_start_pose(const glm::mat4 &pose){
        if(mPlaying) return false;
        mStartPose=pose;
        decomposeTransform(mStartPose,mStartTrans,mStartRot,mStartScale);
        return true;
    }
    glm::mat4 start_pose()const{return mStartPose;}
    glm::mat4 end_pose()const{return mEndPose;}
    bool set_end_pose(const glm::mat4 &pose){
        if(mPlaying) return false;
        mEndPose=pose;
        decomposeTransform(mEndPose,mEndTrans,mEndRot,mEndScale);
        return true;
    }
    bool set_duration(double sec){
        if(mPlaying) return false;
        mDuration=sec; return true;
    }
    bool set_motion_mode(MotionMode mode){
        if(mPlaying) return false;
        mMode=mode; return true;
    }
    bool start(){
        if(mPlaying) return false;
        mPlaying=true;
        mStartTimestamp=CurrentMSecsSinceEpoch();
        return true;
    }
    bool stop(){
        if(!mPlaying) return false;
        mPlaying=false; return true;
    }

private:
    glm::mat4 mStartPose=DefaultPose,mEndPose=DefaultPose;
    glm::vec3 mStartTrans,mEndTrans;
    glm::quat mStartRot,mEndRot;
    glm::vec3 mStartScale,mEndScale;

    double mDuration{0};
    MotionMode mMode{Clamp};
    bool mPlaying{false};
    long long mStartTimestamp{0};

    double computeAlpha(double t)const{
        double alpha=t/mDuration;
        switch(mMode){
            case Clamp:
                return glm::clamp(alpha,0.0,1.0);
            case Loop:{
                double wrapped=fmod(alpha,1.0);
                if(wrapped<0.0f) wrapped+=1.0; // 支持负时间
                return wrapped;
            }
            case PingPong:{
                double cycle=floor(alpha),frac=alpha-cycle;
                if(frac<0.0) frac+=1.0; // 支持负时间
                bool reverse=(static_cast<int>(cycle)%2==1);
                return reverse?(1.0-frac):frac;
            }
        }
        return 1.0; // 默认返回结束位姿
    }
    static void decomposeTransform(const glm::mat4 &mat,glm::vec3 &translation,glm::quat &rotation,glm::vec3 &scale){
        glm::vec3 skew;
        glm::vec4 perspective;
        glm::decompose(mat,scale,rotation,translation,skew,perspective);
        rotation=glm::normalize(rotation);
    }
    inline static const glm::mat4 DefaultPose={0.997388, -0.037953, 0.061457, 0.000000,
                                        -0.035476, -0.998533, -0.040900, 0.000000,
                                        0.062920, 0.038613, -0.997271, 0.000000,
                                        0, 0, 0, 1.000000};
};


#endif //ROKIDOPENXRANDROIDDEMO_POSEMOTION_HPP
