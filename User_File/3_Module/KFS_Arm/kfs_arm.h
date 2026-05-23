#pragma once

#include "dvc_motor_dji.h"
#include "planned_joint.h"
#include "cylinder.h"
#include "arm_sequence_player.h"
#include "main.h"


/*========== Core Class =============*/
class KFS_Arm {
public:
#ifdef DEBUG_MODE
    enum Joint1_Pos{
        JOINT1_0 = 0U,
        JOINT1_90 = 90U,
        JOINT1_180 = 180U
    };

    enum Joint2_Pos{
        JOINT2_0mm = 0U,
        JOINT2_200mm = 200U,
        JOINT2_400mm = 400U,
        JOINT2_600mm = 600U
    };
#endif
    enum HandCmd{
        REACH_OUT,
        PULL_BACK
    };
    KFS_Arm(PlannedJoint& joint1, PlannedJoint& joint2, Cylinder& hand, ArmSequencePlayer<2>& player)
    : joint1(joint1)
    , joint2(joint2)
    , hand_(hand)
    , player(player)
    , state_(ORIGIN)
    {
        player.setContext(this);
    };
    void rotateTo(float target);
    void moveVerticallyTo(float target);
    void reachHandOut();
    void pullHandBack();
#ifdef DEBUG_MODE
    bool checkJoint1PosLimits(Joint1_Pos target);
    bool checkJoint2PosLimits(Joint2_Pos target);
#endif
    bool checkJoint1IsMoving() const {
        return this->joint1.IsMoving();
    }
    bool checkJoint2IsMoving() const {
        return this->joint2.IsMoving();
    }
#ifdef DEBUG_MODE
    float Joint1PosToRad(Joint1_Pos pos);
    float Joint2PosToM(Joint2_Pos pos);
    Joint1_Pos RadToJoint1Pos(float rad);
    Joint2_Pos MToJoint2Pos(float m);
#endif
    volatile enum States{
        ORIGIN,
        INIT_POS,
        ALLOW_MOTION,
        DRAWING,
        WITH_KFS,
        PUT_KFS,
    } state_;
    volatile enum Joint1MotionState{
        DEGS,
        DEGB,
    } joint1_motion_state_;
    volatile enum Joint2MotionState{
        UP,
        DOWN,
    } joint2_motion_state_;

    volatile bool MOTION_EN = false; // 允许运动标志，抓取阶段才允许遥控
    volatile bool DRAWING_KFS = false; // 正在抓取 KFS 的标志
    volatile bool PUTTING_KFS = false; // 正在放置 KFS 的标志
    volatile bool ALLOW_NEXT_CMD = true; // 允许接受下一条指令的标志，正在执行动作时为false，完成后置true

    // 序列播放器（由 KFS_Arm 统一管理，外部不直接访问）
    void playSequence(const ArmStep<2>* sequence, uint16_t step_count) {
        player.Play(sequence, step_count);
    }
    bool isSequencePlaying() const {
        return player.IsPlaying();
    }
    void updateSequence() {
        player.Update();
    }
private:
    PlannedJoint& joint1;
    PlannedJoint& joint2;
    ArmSequencePlayer<2>& player;
    Cylinder& hand_;


};


