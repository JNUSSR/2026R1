#include "slope_joint.h"


bool SlopeJoint::IsWithinLimits(float target_val) const {
    return (target_val >= this->config_.min_limit && target_val <= this->config_.max_limit);
}

void SlopeJoint::Move(float target) {
    if (this->IsWithinLimits(target))
        this->planner_.Set_Target(target);
}

void SlopeJoint::Update() {
    // Target 模式下无需反馈真实位置，规划器纯前馈平滑
    this->planner_.TIM_Calculate_PeriodElapsedCallback();
    float next_pos = this->planner_.Get_Out();
    if (this->IsWithinLimits(next_pos)) {
        float target_angle = this->cvrtTargetToAngle(next_pos);
        this->motor_.Set_Target_Angle(target_angle); 
    }
}

void SlopeJoint::setZeroPos(float zero_pos)  {
    this->config_.zero_pos = zero_pos;
    this->planner_.Set_Target(zero_pos);
}