#pragma once

#include "dvc_motor_dji.h"
#include "planned_joint.h"
#include "cylinder.h"
#include "arm_sequence_player.h"
#include "main.h"


/*========== Core Class =============*/
class KFS_Arm {
public:
    KFS_Arm(PlannedJoint& joint_h, PlannedJoint& joint_v, Cylinder& claw, ArmSequencePlayer<2>& player)
    : joint_h(joint_h)
    , joint_v(joint_v)
    , claw_(claw)
    , player(player)
    , state_(INIT_POS)
    {
        player.setContext(this);
    };
    void reachOutTo(float target);
    void moveVerticallyTo(float target, float duration = 2.0f);
    void releaseClaw();
    void tightenClaw();

    void stopJointV();
#ifdef DEBUG_MODE
    bool checkJoint1PosLimits(Joint1_Pos target);
    bool checkJoint2PosLimits(Joint2_Pos target);
#endif
    bool checkJointHIsMoving() const {
        return this->joint_h.IsMoving();
    }
    bool checkJointVIsMoving() const {
        return this->joint_v.IsMoving();
    }
    volatile enum States{
        INIT_POS,
        ALLOW_MOTION,
        DRAWING_0,
        DRAWING_1,
        DRAWING_2,
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
    float getJointVCurPos(){
        return this->joint_v.getCurrentTarget();
    }
    inline float getJointVCurTorque(){
        return this->joint_v.getCurrentTorque();
    }
private:
    PlannedJoint& joint_h;
    PlannedJoint& joint_v;
    ArmSequencePlayer<2>& player;
    Cylinder& claw_;
};


