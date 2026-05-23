#pragma once

#include "main.h"
#include "dvc_motor_dji.h"
#include "QuinticPlanner.h"


#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
/* ====== 位置式电机接口 ====== */
class BaseMotor {
public:
    virtual ~BaseMotor() = default;
    virtual void Set_Target_Angle(float angle) = 0; // 设置目标角度(弧度)
};

class MotorAdapter_C610 : public BaseMotor {
public:
    explicit MotorAdapter_C610(Class_Motor_DJI_C610& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
private:
    Class_Motor_DJI_C610& real_motor_;
};

class MotorAdapter_C620 : public BaseMotor {
public:
    explicit MotorAdapter_C620(Class_Motor_DJI_C620& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
private:
    Class_Motor_DJI_C620& real_motor_;
};
/* ====== 带规划的关节 ====== */
class PlannedJoint {
public:
    struct JointConfig{
        float min_limit; // 关节的最小位置限制
        float max_limit; // 关节的最大位置限制
        float zero_pos;  // 关节在物理位置0时的电机位置（弧度）
        float direction; // 关节的正方向（1或-1）
        float ratio;     // 电机转动弧度与关节实际转动弧度的比值
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
    PlannedJoint(BaseMotor &motor, JointConfig config, float dt)
        : motor_(motor), config_(config), planner_(config.zero_pos), dt_(dt) {}

    bool IsWithinLimits(float target_val) const;
    void Move(float target, float duration);
    bool IsMoving() const;
    void Update();
    void setZeroPos(float zero_pos);
    float getCurrentTarget() const {
        return this->planner_.GetCurrentTarget();
    }

    // Methods merged from former RobotJoint
    

private:
    BaseMotor &motor_;
    JointConfig config_;
    QuinticPlanner planner_;
    float dt_;
};