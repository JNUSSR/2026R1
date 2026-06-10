#include "slope_joint.h"


bool SlopeJoint::IsWithinLimits(float target_val) const {
    return (target_val >= this->config_.min_limit && target_val <= this->config_.max_limit);
}

void SlopeJoint::Move(float target) {
    if (this->IsWithinLimits(target))
        this->planner_.Set_Target(target);
}

void SlopeJoint::Update() {
    float now_pos = this->cvrtAngleToJointPos(this->motor_.Get_Current_Angle());
    this->planner_.Set_Now_Real(now_pos); // 将当前实际位置反馈给斜坡函数，确保规划的连续性
    this->planner_.TIM_Calculate_PeriodElapsedCallback(); // 计算下一个位置
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