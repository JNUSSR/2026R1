#include "slope_joint.h"

SlopeJoint::SlopeJoint(BaseMotor &motor, JointConfig config, float dt)
    : motor_(motor)
    , config_(config) 
{
    float ds = config.max_speed * dt; // 每次更新的最大位移
    this->planner_.Init(ds, ds, Slope_First_TARGET); // 目标值优先：规划器不跳变，不受CAN反馈异常影响
    this->prev_ds = ds;
}

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

void SlopeJoint::setMaxSpeed(float speed){
    float now_ds = speed * this->dt_;
    this->planner_.Set_Decrease_Value(now_ds);
    this->planner_.Set_Increase_Value(now_ds);
}

void SlopeJoint::resumeDefaultSpeed(){
    float ds = this->config_.max_speed * this->dt_;
    this->planner_.Set_Decrease_Value(ds);
    this->planner_.Set_Increase_Value(ds);
}