#ifndef ROKIDOPENXRANDROIDDEMO_GESTURE_HPP
#define ROKIDOPENXRANDROIDDEMO_GESTURE_HPP

#pragma once

#include"utilsmym.hpp"
#include<mutex>

class Gesture{
public:
    inline static constexpr int HAND_JOINTS_COUNT=26;  //in openxr.h
    inline static constexpr int HISTORY_SIZE=15;
    inline static constexpr int PALM_INDEX=0,WRIST_INDEX=1,FOREFINGER_TIP_INDEX=10;


    enum HandCode{
        LEFT_HAND,RIGHT_HAND,HANDS_COUNT
    };

    enum GestureResult{
        SwipeLeft,SwipeRight,None
    };

    Gesture() = default;
    ~Gesture() = default;

    /* ================= Joint Update ================= */

    void set_joints_pose(HandCode hand,const std::vector<glm::mat4> &pose_list){
        for(int i=0;i<std::min(HAND_JOINTS_COUNT,(int)pose_list.size());++i) mHandJoints[hand][i]=pose_list[i];
        mHandUpdateTimestamp[hand]=CurrentMSecsSinceEpoch();
    }
    glm::mat4 get_joint_pose(int joint_index,HandCode hand=RIGHT_HAND){
        if(joint_index<0||joint_index>=HAND_JOINTS_COUNT) return {};
        return mHandJoints[hand][joint_index];
    }
    glm::vec3 get_joint_position(int joint_index,HandCode hand=RIGHT_HAND){
        if(joint_index<0||joint_index>=HAND_JOINTS_COUNT) return {};
        return glm::vec3(mHandJoints[hand][joint_index][3]);
    }

    void update(const glm::mat4 &camera_mat){
        for(int hand=0;hand<(int)HANDS_COUNT;++hand) handleGesture((HandCode)hand,camera_mat);
    }

    int add_callback(GestureResult gesture,const std::function<void()> &func){
        if(gesture==None) return 0;
        mGestureCallbackList[gesture].emplace_back(func);
        return (int)mGestureCallbackList[gesture].size();
    }


    inline static Gesture &Global(){
        static Gesture inst;
        return inst;
    }

private:
    float _dx,_dt,_df;
    void handleGesture(HandCode hand_code,const glm::mat4 &camera_mat){
        if(hand_code!=RIGHT_HAND) return;
        // 选取 wrist
        glm::vec4 wrist_4=(mHandJoints[hand_code][PALM_INDEX])[3];
        wrist_4=camera_mat*wrist_4;
        glm::vec3 wrist=wrist_4;
//        wrist=(mHandJoints[hand_code][WRIST_INDEX])[3];

        if(wrist.x==0||wrist.y==0||wrist.z==0){
            mHistoryPosition.clear();
            mLastNoHandTimestamp=mHandUpdateTimestamp[hand_code]; return;
        }
        infof("gesture position: %f, %f, %f",wrist.x,wrist.y,wrist.z)

        mHistoryPosition.push_back({wrist,mHandUpdateTimestamp[hand_code]});
        while(mHistoryPosition.size()>HISTORY_SIZE) mHistoryPosition.pop_front();
        if(mHistoryPosition.size()!=HISTORY_SIZE) return;

        for(int dframes=minGestureFrames;dframes<=std::min(maxGestureFrames,HISTORY_SIZE);++dframes){
            int l=0,r=l+dframes-1;
            auto dx=mHistoryPosition[r].pos.x-mHistoryPosition[l].pos.x;
            if(std::abs(dx)>maxSwingDistance) continue;
            if(dx<0&&std::abs(dx)<minSwingLeftDistance) continue;
            if(dx>0&&std::abs(dx)<minSwingRightDistance) continue;
            auto dt=mHistoryPosition[r].timestamp-mHistoryPosition[l].timestamp;
            bool check_bound=true;
            for(int i=l+1;i<=r;++i){
                auto dy=mHistoryPosition[i].pos.y-mHistoryPosition[i-1].pos.y,dz=mHistoryPosition[i].pos.z-mHistoryPosition[i-1].pos.z;
                if(dx<0){
                    if(std::abs(dy)>SwipeLeftYLimit||std::abs(dz)>SwipeLeftZLimit){
                        check_bound=false; break;
                    }
                }
                else{
                    if(std::abs(dy)>SwipeRightYLimit||std::abs(dz)>SwipeRightZLimit){
                        check_bound=false; break;
                    }
                }
            }
            if(!check_bound) continue;
            if(mHandUpdateTimestamp[hand_code]-mLastNoHandTimestamp>GestureDurationLimit) continue;
            mHistoryPosition.clear();
            _dt=mHandUpdateTimestamp[hand_code]-mLastNoHandTimestamp; _dx=dx; _df=dframes;
            if(dx<0) process_gesture_result(GestureResult::SwipeLeft);
            else process_gesture_result(GestureResult::SwipeRight);
        }
    }
    void process_gesture_result(GestureResult result){
        if(result==GestureResult::None) return;
        if(CurrentMSecsSinceEpoch()-mLastGestureTimestamp<mGestureMinGap) return;
        if(result==GestureResult::SwipeLeft){
            infof("Gesture result:\n ◀ 向左挥手(平面检测), dx: %f, dt: %.0f ms, dframes: %.0f",_dx,_dt,_df)
        }
        else if(result==GestureResult::SwipeRight){
            infof("Gesture result:\n ▶ 向右挥手(平面检测), dx: %f, dt: %.0f ms, dframes: %.0f",_dx,_dt,_df)
        }
        for(const auto &cb:mGestureCallbackList[result]) cb();
        mLastGestureTimestamp=CurrentMSecsSinceEpoch();
    }
private:
    glm::mat4 mHandJoints[HANDS_COUNT][HAND_JOINTS_COUNT]{};
    long long mHandUpdateTimestamp[HANDS_COUNT]{};
    long long mLastGestureTimestamp{0};
    long long mGestureMinGap{1000};
    long long mLastNoHandTimestamp{0}; //最后一次手部关节点不出现在视野里的时间戳
    std::vector<std::function<void()>> mGestureCallbackList[GestureResult::None];

    //Configs
    struct JointInfo{
        glm::vec3 pos{};
        long long timestamp{};
    };
    std::deque<JointInfo> mHistoryPosition;
    const int minGestureFrames=5;   // 至少持续 5 帧
    const int maxGestureFrames=29; // 至多持续 20 帧
    const long long GestureDurationLimit=200; //ms
    const float minSwingLeftDistance=0.09f;  // 触发挥动最小总位移
    const float minSwingRightDistance=0.15f;  // 触发挥动最小总位移
    const float maxSwingDistance=0.55f;  // 触发挥动最大总位移
    // --- 限制 Y / Z 平面 ---
    const float SwipeLeftYLimit=0.30f;   // 最大允许上下抖动
    const float SwipeLeftZLimit=0.955f;  // 最大允许前后抖动
    const float SwipeRightYLimit=0.38f;   // 最大允许上下抖动
    const float SwipeRightZLimit=0.715f;  // 最大允许前后抖动
};



#endif //ROKIDOPENXRANDROIDDEMO_GESTURE_HPP
