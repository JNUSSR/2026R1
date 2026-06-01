/**
 * @file uart_wheelchassis.h
 * @brief 底盘UART/MAVLink控制接口（含斜坡规划）
 */

#ifndef UART_WHEELCHASSIS_H
#define UART_WHEELCHASSIS_H

#include "crt_wheelchassis.h"
#include "alg_slope.h"

class Class_Uart_Wheelchassis : public Class_Chassis_Mecanum
{
public:
	// 绑定底层底盘对象并初始化电机与斜坡规划器
	void Init(FDCAN_HandleTypeDef* hcan);

	// 设置上位机/串口给定的原始目标速度（m/s, m/s, rad/s）
	void SetMotion(float vx, float vy, float vw);

	// 先更新目标，再做一次斜坡规划并下发到底盘控制周期
	void WithSlopeTask(float vx, float vy, float vw);

	// 使用当前缓存目标做一次斜坡规划并下发到底盘控制周期
	void WithSlopeTask();

	// 运行时调整斜坡步长（单位为每1ms控制周期增量）
	void SetSlopeStep(float v_inc, float v_dec, float w_inc, float w_dec);

	// 初始化斜坡规划器,用于急停时置零斜坡规划器使用
	void InitSlopePlanner();

private:
	// 将斜坡规划器的初始化从private移动到public，允许外部调用以实现急停时的斜坡规划器置零
	// void InitSlopePlanner();
	static float ApplyDeadzone(float value, float deadzone);

private:
	static constexpr float SPEED_DEADZONE = 0.001f; //死区设置，避免小输入导致底盘抖动

	// 默认斜率规划参数（1kHz）
	static constexpr float VELOCITY_INCREASE_STEP = 0.012f;// 0.004f对应约4m/s/s的加速度
	static constexpr float VELOCITY_DECREASE_STEP = 0.015f;// 0.006f对应约6m/s/s的减速度
	static constexpr float ANGULAR_INCREASE_STEP = 0.003f;// 0.003f对应约3rad/s²的加速度
	static constexpr float ANGULAR_DECREASE_STEP = 0.005f;// 0.005f对应约5rad/s²的减速度

	FDCAN_HandleTypeDef* hcan_ = nullptr;

	float target_vx_raw_ = 0.0f;
	float target_vy_raw_ = 0.0f;
	float target_vw_raw_ = 0.0f;

	Class_Slope slope_vx_;
	Class_Slope slope_vy_;
	Class_Slope slope_vw_;
};

#endif
