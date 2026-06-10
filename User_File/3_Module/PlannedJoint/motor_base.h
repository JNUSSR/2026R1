#pragma once

#include "dvc_motor_dji.h"

/* ====== 位置式电机接口 ====== */
class BaseMotor {
public:
    virtual ~BaseMotor() = default;
    virtual void Set_Target_Angle(float angle) = 0; // 设置目标角度(弧度)
    virtual float Get_Current_Torque() = 0; // 获取当前扭矩
    virtual float Get_Current_Angle() = 0; // 获取当前角度(弧度)
};

class MotorAdapter_C610 : public BaseMotor {
public:
    explicit MotorAdapter_C610(Class_Motor_DJI_C610& real_motor) : real_motor_(real_motor) {}
    void Set_Target_Angle(float angle) override {
        real_motor_.Set_Target_Angle(angle);
    }
    float Get_Current_Torque() override {
        return real_motor_.Get_Now_Torque();
    }
    float Get_Current_Angle() override {
        return real_motor_.Get_Now_Angle();
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
    float Get_Current_Angle() override {
        return real_motor_.Get_Now_Angle();
    }
    float Get_Current_Torque() override {
        return real_motor_.Get_Now_Torque();
    }
private:
    Class_Motor_DJI_C620& real_motor_;
};