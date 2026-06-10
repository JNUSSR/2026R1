//
// Created by chengfeng on 2026/4/18.
//

#include "kfs_arm.h"


/*=========== KFS_Arm ===========*/
void KFS_Arm::reachOutTo(float target, float duration){
    this->joint_h.Move(target, duration);
}

void KFS_Arm::moveVerticallyTo(float target){
    this->joint_v.Move(target);
}

void KFS_Arm::releaseClaw(){
    this->claw_.release();
}

void KFS_Arm::tightenClaw(){
    this->claw_.charge();
}

void KFS_Arm::stopJointV(){
    // SlopeJoint 没有 stop 方法，通过设当前目标为当前位置来实现"停住"
    this->joint_v.Move(this->joint_v.getCurrentTarget());
}