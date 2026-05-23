#include "planned_joint.h"


bool PlannedJoint::IsWithinLimits(float target_val) const {
    return (target_val >= this->config_.min_limit && target_val <= this->config_.max_limit);
}

void PlannedJoint::Move(float target, float duration) {
    // 只有在空闲且目标合法的情况下才启动新规划
    if (!this->planner_.IsPlanning() && this->IsWithinLimits(target)) {
        this->planner_.Plan(this->planner_.GetCurrentTarget(), target, duration);
    }
}

bool PlannedJoint::IsMoving() const {
    return this->planner_.IsPlanning();
}

void PlannedJoint::Update() {
    float next_pos = this->planner_.GetNextPosition(this->dt_);
    if (this->IsWithinLimits(next_pos)) {
        float target_angle = (next_pos - this->config_.zero_pos) * this->config_.direction * this->config_.ratio;
        this->motor_.Set_Target_Angle(target_angle); 
    }
}

void PlannedJoint::setZeroPos(float zero_pos)  {
    this->config_.zero_pos = zero_pos;
    this->planner_.Plan(this->planner_.GetCurrentTarget(), zero_pos, 5);
}