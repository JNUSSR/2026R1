#pragma once

#include "main.h"
#include "dvc_motor_dji.h"
#include "alg_slope.h"
#include "motor_base.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ====== 带规划的关节 ====== 
   允许目标值在任意时刻被更新，关节会平滑地移动到新的目标值
*/
class SlopeJoint {
public:
    struct JointConfig{
        float min_limit; // 关节的最小位置限制
        float max_limit; // 关节的最大位置限制
        float zero_pos;  // 关节在物理位置0时的电机位置（弧度）
        float direction; // 关节的正方向（1或-1）
        float ratio;     // 电机转动弧度与关节实际转动弧度的比值
        float max_speed; // 关节的最大速度限制，单位为位置单位/秒
    };
    /**
    * @param motor 电机对象引用
    * @param min_limit 关节的最小位置限制
    * @param max_limit 关节的最大位置限制
    * @param zero_pos 关节在物理位置0时的电机位置（弧度）
    * @param dir 关节的正方向（1或-1）
    * @param ratio 电机转动弧度与关节实际转动弧度的比值。设Δθ为电机输出轴单位转动弧度，Δm为外部对应的单位长度/弧度等，ratio = Δθ/Δm。
    * @param dt 更新周期，单位秒
    */
    SlopeJoint(BaseMotor &motor, JointConfig config, float dt)
        : motor_(motor)
        , config_(config) 
    {
        float ds = config.max_speed * dt; // 每次更新的最大位移
        this->planner_.Init(ds, ds, Slope_First_TARGET); // 目标值优先：规划器不跳变，不受CAN反馈异常影响
    }

    bool IsWithinLimits(float target_val) const;
    void Move(float target);
    void Move(float target, float /*duration*/) { Move(target); }  // 兼容 ArmSequencePlayer，duration 忽略
    bool IsMoving() const;
    void Update();
    void setZeroPos(float zero_pos);

    inline float getCurrentTorque(){
        return this->motor_.Get_Current_Torque();
    }

    inline float getCurrentTarget() const {
        return this->planner_.Get_Out();
    }

    inline float cvrtAngleToJointPos(float angle) const {
        return angle / (this->config_.direction * this->config_.ratio) + this->config_.zero_pos;
    }
    inline float cvrtTargetToAngle(float target) const {
        return (target - this->config_.zero_pos) * this->config_.direction * this->config_.ratio;
    }

    // Methods merged from former RobotJoint
    

private:
    BaseMotor &motor_;
    JointConfig config_;
    Class_Slope planner_;
    float dt_;
};