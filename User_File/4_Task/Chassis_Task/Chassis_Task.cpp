/**
 * @file Chassis_Task.cpp
 * @brief 舵轮底盘控制任务实现
 *
 * 移植自 test_total_Rc 工程，适配 STM32H723 + FreeRTOS + FDCAN
 */

/* Includes ------------------------------------------------------------------*/

#include "Chassis_Task.h"

#include "cmsis_os.h"
#include "fdcan.h"
#include "usart.h"
#include "gpio.h"
#include "v_filter.h"
#include "bsp_bmi088.h"
// 中间件驱动
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/UART/drv_uart.h"
#include "1_Middleware/Driver/UART_printf/uart_printf.h"

// 设备驱动
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Plotter/Serialplot/dvc_serialplot.h"

#include "ADC_TO_CHANNEL.h"

#include <math.h>
#include <string.h>

VelocitySmoother smoother;

extern Class_Matrix_f32<3, 1> Euler;
/* Private macros ------------------------------------------------------------*/

#define PI 3.14159265358979323846f

// 底盘参数
#define WHEEL_BASE_X     0.38f    // 前后轮距的一半 (m)
#define WHEEL_BASE_Y     0.38f    // 左右轮距的一半 (m)
#define WHEEL_RADIUS     0.06f   // 轮子半径 (m)

// 遥控参数
#define RC_BASE_LINEAR_SPEED    2.0f   // 摇杆推满时底盘基准线速度 (m/s)
// 四个轮子独立的摇杆输入缩放比例: 各轮角速度 = 基准角速度 × 该比例
// 例如 RC_OMEGA_GAIN_M1=1.5 则该轮最高 50rad/s, RC_OMEGA_GAIN_M3=0.6 则最高 20rad/s
#define RC_OMEGA_GAIN_M1        1//1.28f   // 3508#1 (左前) 输入缩放 （1.30）
#define RC_OMEGA_GAIN_M2        1//1.0f   // 3508#2 (右前) 输入缩放
#define RC_OMEGA_GAIN_M3        1//1.24f   // 3508#3 (右后) 输入缩放
#define RC_OMEGA_GAIN_M4        1//1.112f   // 3508#4 (左后) 输入缩放
#define RC_OMEGA_GAIN_PER_WHEEL {RC_OMEGA_GAIN_M1, RC_OMEGA_GAIN_M2, RC_OMEGA_GAIN_M3, RC_OMEGA_GAIN_M4}
#define RC_TO_VW_SCALE         0.03f   // 旋转角速度比例
#define RC_DEADBAND            15.0f  // 右摇杆平移死区 (%)
#define RC_ROTATION_DEADBAND   15.0f   // 左摇杆旋转死区 (%)
#define RC_ANGLE_DEADBAND      0.1f  // 角度变化死区 (rad)约2.8°(0.05f)，小于此变化不更新方向
#define RC_RETURN_DELAY_TICKS  1600    // 摇杆居中延迟

// IBUS 帧长度
#define IBUS_FRAME_LENGTH 32

/* Private types -------------------------------------------------------------*/

/**
 * @brief 
 */
typedef struct
{
    uint16_t Lx_Min, Lx_Mid, Lx_Max;
    uint16_t Ly_Min, Ly_Max;
    uint16_t Rx_Min, Rx_Mid, Rx_Max;
    uint16_t Ry_Min, Ry_Mid, Ry_Max;
    uint16_t VRA_Min, VRA_Max;
    uint16_t VRB_Min, VRB_Max;
    uint16_t SWA_Close, SWA_Open;
    uint16_t SWB_Close, SWB_Open;
    uint16_t SWC_Close, SWC_First, SWC_Second;
    uint16_t SWD_Close, SWD_Open;
} ChannelParameters;

// 通道索引
#define LXChannel 4
#define LYChannel 3
#define RXChannel 1
#define RYChannel 2
#define VRAChannel 5
#define VRBChannel 6
#define SWAChannel 7
#define SWBChannel 8
#define SWCChannel 9
#define SWDChannel 10

/**
 * @brief IBUS 遥控器通道值结构体 (移植自 test_total_Rc/removecontrol.h)
 */
typedef struct
{
    float Lx;   // -100~0~100
    float Ly;   // 0~100 (单向!)
    float Rx;   // -100~0~100
    float Ry;   // -100~0~100
    float VRA;  // 0~100
    float VRB;  // 0~100
    uint8_t SWA; // 0,1
    uint8_t SWB; // 0,1
    uint8_t SWC; // 0,1,2
    uint8_t SWD; // 0,1
} ProcessedChannelValue;

/* Private variables ---------------------------------------------------------*/

// ==================== 电机对象 ====================
static Class_Motor_DJI_C620  Motor_3508[4];
static Class_Motor_DM_Normal Motor_DM6220[4];

// ==================== 串口绘图 ====================
static Class_Serialplot_UART Serialplot;         // UART7 (已注释, 与IBUS共用)
static Class_Serialplot_UART Serialplot_Debug;   // USART1 独立调试, 发3508速度

// ==================== 模式与状态 ====================
static uint8_t Remote_Mode = 1;  // 0:串口控制 1:遥控(IBUS)
static uint8_t moddel      = 0;  // 0:UART7=IBUS  1:UART7=VOFA串口(发速度数据)

// ==================== IBUS 数据 ====================
static bool     ibus_ready = false;                 // 初始化完成后才处理 IBUS
static uint8_t  Ibus_Buffer[IBUS_FRAME_LENGTH];
static uint16_t Ibus_ChannelData[14];
static ProcessedChannelValue G_Value;
static ProcessedChannelValue G_Value_Last;         // 低通滤波上一帧
static ChannelParameters    G_ChannelParam;         // 校准参数

// ==================== 底盘控制变量 ====================
static float Chassis_Angles[4] = {0};
static float Chassis_Speeds[4] = {0};
static float target_angles[4]  = {0};
static float rot_angles[4]     = {0};
static float last_move_angle   = 0.0f;

// 串口绘图变量名列表 (UART7 调试)
static const char *Variable_Assignment_List[] =
{
    "W", "S", "A", "D",
    "KP", "KI", "KD",
    "An", "Sa", "MODE"
};
static const uint8_t Variable_Assignment_Num = sizeof(Variable_Assignment_List) / sizeof(Variable_Assignment_List[0]);

// USART1 调试变量名: 3508#1 目标速度 / 当前速度
static const char *Debug_Var_List[] = { "Tgt", "Now" };
static const uint8_t Debug_Var_Num = sizeof(Debug_Var_List) / sizeof(Debug_Var_List[0]);

/* Private function declarations ---------------------------------------------*/

// IBUS
static void IBUS_Parse(const uint8_t *data, uint16_t channels[14]);
static void IBUS_ProcessChannelData(const uint16_t channels[14]);

// 底盘控制
static void Chassis_ProcessRotation(float lx);
static void Chassis_ProcessTranslation(float rx, float ry, float lx);
static void Chassis_SetMotion(float move_angle_rad, float speed, float rotation);
static void Chassis_CalcWheelControl(int idx, float vx, float vy, float wz);
static float Chassis_NormalizeAngle(float angle);

// 串口指令处理
static void Serialplot_HandleCommand(int var_index, float var_value);

// UART7 回调 
static void UART7_Shared_Callback(uint8_t *Buffer, uint16_t Length);

// CAN1 回调 (M3508)
static void CAN1_Motor_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

/* Function implementations --------------------------------------------------*/

// ==================== IBUS 解析 ====================

/**
 * @brief 解析 IBUS 32字节帧 → 14通道
 */
static void IBUS_Parse(const uint8_t *data, uint16_t channels[14])
{
    for (uint8_t i = 0; i < 14; i++)
    {
        channels[i] = (uint16_t)(data[3 + 2 * i] << 8) | data[2 + 2 * i];
    }
}

/**
 * @brief 遥控器校准映射 (移植自 test_total_Rc/removecontrol.c)
 *
 * @param mid   中位值, 0 表示单向(无中位)
 * @param min   最小值
 * @param max   最大值
 * @param val   当前通道原始值
 * @return float -1.0~1.0 (单向时 0~1.0)
 */
static float RC_Math(uint16_t mid, uint16_t min, uint16_t max, uint16_t val)
{
    if (mid == 0)
    {
        return (float)(val - min) / (float)(max - min);
    }
    else
    {
        return (val >= mid)
            ? (float)(val - mid) / (float)(max - mid)
            : (float)(val - mid) / (float)(mid - min);
    }
}

/**
 * @brief IBUS 通道值 → 工程单位 (移植自 test_total_Rc)
 */
static void IBUS_ProcessChannelData(const uint16_t channels[14])
{
    const float alpha = 1.0f;  // 低通系数, 1.0=不过滤

    G_Value.Lx = alpha * 100.0f * RC_Math(G_ChannelParam.Lx_Mid, G_ChannelParam.Lx_Min, G_ChannelParam.Lx_Max, channels[LXChannel - 1])
               + (1.0f - alpha) * G_Value_Last.Lx;
    G_Value.Ly = alpha * 100.0f * RC_Math(0, G_ChannelParam.Ly_Min, G_ChannelParam.Ly_Max, channels[LYChannel - 1])
               + (1.0f - alpha) * G_Value_Last.Ly;
    G_Value.Rx = alpha * 100.0f * RC_Math(G_ChannelParam.Rx_Mid, G_ChannelParam.Rx_Min, G_ChannelParam.Rx_Max, channels[RXChannel - 1])
               + (1.0f - alpha) * G_Value_Last.Rx;
    G_Value.Ry = alpha * 100.0f * RC_Math(G_ChannelParam.Ry_Mid, G_ChannelParam.Ry_Min, G_ChannelParam.Ry_Max, channels[RYChannel - 1])
               + (1.0f - alpha) * G_Value_Last.Ry;
    G_Value.VRA = alpha * 100.0f * RC_Math(0, G_ChannelParam.VRA_Min, G_ChannelParam.VRA_Max, channels[VRAChannel - 1])
                + (1.0f - alpha) * G_Value_Last.VRA;
    G_Value.VRB = alpha * 100.0f * RC_Math(0, G_ChannelParam.VRB_Min, G_ChannelParam.VRB_Max, channels[VRBChannel - 1])
                + (1.0f - alpha) * G_Value_Last.VRB;

    G_Value_Last = G_Value;

    // 开关通道
    G_Value.SWA = (channels[SWAChannel - 1] == G_ChannelParam.SWA_Open) ? 1 : 0;
    G_Value.SWB = (channels[SWBChannel - 1] == G_ChannelParam.SWB_Open) ? 1 : 0;

    if      (channels[SWCChannel - 1] == G_ChannelParam.SWC_Second) G_Value.SWC = 2;
    else if (channels[SWCChannel - 1] == G_ChannelParam.SWC_First)  G_Value.SWC = 1;
    else                                                              G_Value.SWC = 0;

    G_Value.SWD = (channels[SWDChannel - 1] == G_ChannelParam.SWD_Open) ? 1 : 0;
}

// ==================== UART7 共享回调 ====================

/**
 * @brief UART7 接收回调 —— 区分 IBUS 与串口绘图
 *
 * IBUS 帧特征: 0x20 0x40 开头, 32字节
 * 串口绘图帧: 0xAB 开头
 */
static void UART7_Shared_Callback(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0 || Buffer == nullptr) return;
    if (!ibus_ready) return;

    if (moddel == 0)
    {
        // moddel=0: IBUS 模式
        if (Length == IBUS_FRAME_LENGTH && Buffer[0] == 0x20 && Buffer[1] == 0x40)
        {
            IBUS_Parse(Buffer, Ibus_ChannelData);
            IBUS_ProcessChannelData(Ibus_ChannelData);
        }
    }
    else
    {
        // moddel=1: VOFA/串口绘图模式
        // Serialplot.UART_RxCpltCallback(Buffer, Length);
        // int32_t var_index = Serialplot.Get_Variable_Index();
        // float   var_value = Serialplot.Get_Variable_Value();
        // Serialplot_HandleCommand(var_index, var_value);
    }
}

// ==================== 串口指令处理 ====================

static void Serialplot_HandleCommand(int var_index, float var_value)
{
    switch (var_index)
    {
    case 0:  // W — 前进 (调试模式)
    {
        if (Remote_Mode != 0) break;
        // 获取当前 DM 角度
        float a[4];
        for (int i = 0; i < 4; i++)
        {
            a[i] = Motor_DM6220[i].Get_Now_Angle();
            a[i] = fmodf(a[i], 2.0f * PI);
            if (a[i] < 0.0f) a[i] += 2.0f * PI;
        }
        for (int i = 0; i < 4; i++)
        {
            if ((a[i] >= 0.0f && a[i] < PI / 2.0f) || (a[i] >= PI * 3.0f / 2.0f && a[i] < PI * 2.0f))
            {
                Motor_DM6220[i].Set_Control_Angle(0.0f);
                Motor_3508[i].Set_Target_Omega(50.0f * PI);
            }
            else
            {
                Motor_DM6220[i].Set_Control_Angle(PI);
                Motor_3508[i].Set_Target_Omega(-50.0f * PI);
            }
        }
        break;
    }
    case 1:  // S — 后退 (调试模式)
    {
        if (Remote_Mode != 0) break;
        float a[4];
        for (int i = 0; i < 4; i++)
        {
            a[i] = Motor_DM6220[i].Get_Now_Angle();
            a[i] = fmodf(a[i], 2.0f * PI);
            if (a[i] < 0.0f) a[i] += 2.0f * PI;
        }
        for (int i = 0; i < 4; i++)
        {
            if ((a[i] >= 0.0f && a[i] < PI / 2.0f) || (a[i] >= PI * 3.0f / 2.0f && a[i] < PI * 2.0f))
            {
                Motor_DM6220[i].Set_Control_Angle(0.0f);
                Motor_3508[i].Set_Target_Omega(-50.0f * PI);
            }
            else
            {
                Motor_DM6220[i].Set_Control_Angle(PI);
                Motor_3508[i].Set_Target_Omega(50.0f * PI);
            }
        }
        break;
    }
    case 2:  // A — 左移
    {
        if (Remote_Mode != 0) break;
        for (int i = 0; i < 4; i++)
        {
            Motor_DM6220[i].Set_Control_Angle(PI / 2.0f);
            Motor_3508[i].Set_Target_Omega(50.0f * PI);
        }
        break;
    }
    case 3:  // D — 右移
    {
        if (Remote_Mode != 0) break;
        for (int i = 0; i < 4; i++)
        {
            Motor_DM6220[i].Set_Control_Angle(3.0f * PI / 2.0f);
            Motor_3508[i].Set_Target_Omega(50.0f * PI);
        }
        break;
    }
    case 4:  // KP
    {
        Motor_3508[0].PID_Omega.Set_K_P(var_value);
        break;
    }
    case 5:  // KI
    {
        Motor_3508[0].PID_Omega.Set_K_I(var_value);
        break;
    }
    case 6:  // KD
    {
        Motor_3508[0].PID_Omega.Set_K_D(var_value);
        break;
    }
    case 7:  // An — 相对角度
    {
        Motor_DM6220[1].Set_Control_Angle(Motor_DM6220[0].Get_Now_Angle() - var_value);
        break;
    }
    case 8:  // Sa — 保存零点
    {
        Motor_DM6220[1].CAN_Send_Save_Zero();
        break;
    }
    case 9:  // MODE — 模式切换
    {
        Remote_Mode = (uint8_t)var_value;
        if (Remote_Mode > 1) Remote_Mode = 0;
        break;
    }
    default:
        break;
    }
}

// ==================== CAN 回调 ====================

/**
 * @brief FDCAN1 回调 — M3508 (0x201~0x204) + DM6220 (0x05~0x08)
 */
static void CAN1_Motor_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    // M3508 速度轮
    case 0x201: Motor_3508[0].CAN_RxCpltCallback(); break;
    case 0x202: Motor_3508[1].CAN_RxCpltCallback(); break;
    case 0x203: Motor_3508[2].CAN_RxCpltCallback(); break;
    case 0x204: Motor_3508[3].CAN_RxCpltCallback(); break;
    // DM6220 方向舵
    case 0x05:  Motor_DM6220[0].CAN_RxCpltCallback(); break;
    case 0x06:  Motor_DM6220[1].CAN_RxCpltCallback(); break;
    case 0x07:  Motor_DM6220[2].CAN_RxCpltCallback(); break;
    case 0x08:  Motor_DM6220[3].CAN_RxCpltCallback(); break;
    default: break;
    }
}

// ==================== 底盘控制算法 ====================

/**
 * @brief 左摇杆 — 自转控制
 */
static void Chassis_ProcessRotation(float lx)
{
    if (fabsf(lx) < RC_ROTATION_DEADBAND) return;

    // 固定目标角度: 左前45°, 右前135°, 右后225°, 左后315°
    const float targets[4] = {
        PI / 4.0f, 3.0f * PI / 4.0f,
        5.0f * PI / 4.0f, 7.0f * PI / 4.0f
    };

    float wz = -RC_TO_VW_SCALE * lx;
    float dist = sqrtf(WHEEL_BASE_X * WHEEL_BASE_X + WHEEL_BASE_Y * WHEEL_BASE_Y);
    float motor_omega = fabsf(wz) * dist / WHEEL_RADIUS;
    if (wz > 0) motor_omega = -motor_omega;

    for (int i = 0; i < 4; i++)
    {
        float cur_mod = fmodf(rot_angles[i], 2.0f * PI);
        if (cur_mod < 0) cur_mod += 2.0f * PI;

        float delta = targets[i] - cur_mod;
        while (delta >  PI) delta -= 2.0f * PI;
        while (delta < -PI) delta += 2.0f * PI;

        float final_angle, final_omega;
        if (fabsf(delta) > PI / 2.0f)
        {
            final_angle = rot_angles[i] + delta + (delta > 0 ? -PI : PI);
            final_omega = -motor_omega;
        }
        else
        {
            final_angle = rot_angles[i] + delta;
            final_omega = motor_omega;
        }

        rot_angles[i] = final_angle;
        Motor_DM6220[i].Set_Control_Angle(final_angle);
        Motor_3508[i].Set_Target_Omega(final_omega);
    }
}

/**
 * @brief 右摇杆 — 平移控制
 */
static void Chassis_ProcessTranslation(float rx, float ry, float lx)
{
    float distance = sqrtf(rx * rx + ry * ry);
    if (distance < RC_DEADBAND) return;

    float angle = atan2f(rx, ry);
    if (angle < 0) angle += 2.0f * PI;

    float angle_diff = angle - last_move_angle;
    while (angle_diff >  PI) angle_diff -= 2.0f * PI;
    while (angle_diff < -PI) angle_diff += 2.0f * PI;
    if (fabsf(angle_diff) < RC_ANGLE_DEADBAND)
        angle = last_move_angle;
    else
        last_move_angle = angle;

    float speed = (distance / 100.0f) * RC_BASE_LINEAR_SPEED;

    float wz = -RC_TO_VW_SCALE * lx;

    Chassis_SetMotion(angle, speed, wz);
}

/**
 * @brief 底盘运动分解
 */
static void Chassis_SetMotion(float move_angle_rad, float speed, float rotation)
{
    float vx = speed * cosf(move_angle_rad);
    float vy = speed * sinf(move_angle_rad);

    for (int i = 0; i < 4; i++)
        Chassis_CalcWheelControl(i, vx, vy, rotation);
}

/**
 * @brief 单轮运动学解算 (舵轮)
 *
 * 轮编号: 0-左前, 1-右前, 2-右后, 3-左后
 */
static void Chassis_CalcWheelControl(int idx, float vx, float vy, float wz)
{
    float wheel_vx, wheel_vy;

    switch (idx)
    {
    case 0: // 左前 (+X, +Y)
        wheel_vx = vx + wz * WHEEL_BASE_Y;
        wheel_vy = vy - wz * WHEEL_BASE_X;
        break;
    case 1: // 右前 (+X, -Y)
        wheel_vx = vx - wz * WHEEL_BASE_Y;
        wheel_vy = vy - wz * WHEEL_BASE_X;
        break;
    case 2: // 右后 (-X, -Y)
        wheel_vx = vx - wz * WHEEL_BASE_Y;
        wheel_vy = vy + wz * WHEEL_BASE_X;
        break;
    case 3: // 左后 (-X, +Y)
        wheel_vx = vx + wz * WHEEL_BASE_Y;
        wheel_vy = vy + wz * WHEEL_BASE_X;
        break;
    default:
        return;
    }

    float wheel_speed = sqrtf(wheel_vx * wheel_vx + wheel_vy * wheel_vy);

    if (wheel_speed < 0.01f)
    {
        Motor_3508[idx].Set_Target_Omega(0.0f);
        return;
    }

    // 目标滚动方向 [0, 2π)
    float target_raw = atan2f(wheel_vy, wheel_vx);
    if (target_raw < 0) target_raw += 2.0f * PI;

    // 连续角度追踪
    float cur_cont = target_angles[idx];
    float cur_mod  = fmodf(cur_cont, 2.0f * PI);
    if (cur_mod < 0) cur_mod += 2.0f * PI;

    float delta = target_raw - cur_mod;
    while (delta >  PI) delta -= 2.0f * PI;
    while (delta < -PI) delta += 2.0f * PI;

    // 互补角优化 (与 test_total_Rc 完全一致)
    //   |delta|>90° → 翻转180°+反转, tracking存储翻转后的角度
    float target_cont = cur_cont + delta;       // 连续角度
    float final_angle = target_cont;             // 输出角度
    float speed_sign  = 1.0f;                    // 速度符号

    if (fabsf(delta) > PI / 2.0f)
    {
        final_angle = target_cont + (delta > 0 ? -PI : PI);
        speed_sign  = -1.0f;
    }

    float motor_omega = speed_sign * wheel_speed / WHEEL_RADIUS;

    // 各轮独立的输入缩放: omega × gain, gain=1.5则该轮比基准快50%
    static const float omega_gain[4] = RC_OMEGA_GAIN_PER_WHEEL;
    motor_omega *= omega_gain[idx];

    target_angles[idx] = final_angle;
    Chassis_Angles[idx]  = final_angle;
    Chassis_Speeds[idx]  = motor_omega;

    Motor_DM6220[idx].Set_Control_Angle(final_angle);
    Motor_3508[idx].Set_Target_Omega(motor_omega);
}

static float Chassis_NormalizeAngle(float angle)
{
    while (angle < 0.0f)       angle += 2.0f * PI;
    while (angle >= 2.0f * PI) angle -= 2.0f * PI;
    return angle;
}

// ==================== 初始化 ====================

/**
 * @brief 底盘任务初始化
 */
void Chassis_Task_Init(void)
{
    // ---- 1. M3508 PID 初始化
    init_smoother(&smoother,0.01,0.01,0.4363,10);


    for (int i = 0; i < 4; i++)
    {
        //Motor_3508[i].PID_Omega.Init(20.0f, 0.001f, 0.2f, 0.0f, 2500.0f, 2500.0f);
        Motor_3508[i].PID_Omega.Init(0.05f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f);
    }
    

    // ---- 2. M3508 绑定 FDCAN1 ----
    Motor_3508[0].Init(&hfdcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_OMEGA);
    Motor_3508[1].Init(&hfdcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_OMEGA);
    Motor_3508[2].Init(&hfdcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_OMEGA);
    Motor_3508[3].Init(&hfdcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_OMEGA);

    // ---- 3. DM6220 绑定 FDCAN1 (传统模式, MIT控制) ----
    //      CAN_Rx_ID=0x05~0x08, CAN_Tx_ID 同 Rx_ID
    Motor_DM6220[0].Init(&hfdcan1, 0x05, 0x05, Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 45.0f, 10.0f, 10.261194f);
    Motor_DM6220[1].Init(&hfdcan1, 0x06, 0x06, Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 45.0f, 10.0f, 10.261194f);
    Motor_DM6220[2].Init(&hfdcan1, 0x07, 0x07, Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 45.0f, 10.0f, 10.261194f);
    Motor_DM6220[3].Init(&hfdcan1, 0x08, 0x08, Motor_DM_Control_Method_NORMAL_MIT,
                         12.5f, 45.0f, 10.0f, 10.261194f);


    // ---- 4. 遥控器校准参数初始化 (移植自 test_total_Rc) ----
    //      必须在 UART DMA 启动前完成, 防止 IBUS 数据到达时读到垃圾值
    //      FS-i6 默认范围 1000~2000, 中位 1500
    G_ChannelParam.Lx_Min = 1000; G_ChannelParam.Lx_Mid = 1500; G_ChannelParam.Lx_Max = 2000;
    G_ChannelParam.Ly_Min = 1000;                               G_ChannelParam.Ly_Max = 2000;
    G_ChannelParam.Rx_Min = 1000; G_ChannelParam.Rx_Mid = 1500; G_ChannelParam.Rx_Max = 2000;
    G_ChannelParam.Ry_Min = 1000; G_ChannelParam.Ry_Mid = 1500; G_ChannelParam.Ry_Max = 2000;
    G_ChannelParam.VRA_Min = 1000;                              G_ChannelParam.VRA_Max = 2000;
    G_ChannelParam.VRB_Min = 1000;                              G_ChannelParam.VRB_Max = 2000;
    G_ChannelParam.SWA_Close = 1000;                            G_ChannelParam.SWA_Open  = 2000;
    G_ChannelParam.SWB_Close = 1000;                            G_ChannelParam.SWB_Open  = 2000;
    G_ChannelParam.SWC_Close = 1000; G_ChannelParam.SWC_First = 1500; G_ChannelParam.SWC_Second = 2000;
    G_ChannelParam.SWD_Close = 1000;                            G_ChannelParam.SWD_Open  = 2000;

    // // ---- 5a. 串口绘图绑定 UART7 (仅存储句柄, 不启动 DMA) ----
    // Serialplot.Init(&huart7, Serialplot_Checksum_8_ENABLE,
    //                 Variable_Assignment_Num, Variable_Assignment_List,
    //                 Serialplot_Data_Type_FLOAT, 0xAB);

    // ---- 6. FDCAN1 初始化 (M3508 + DM6220 共用) ----
    CAN_Init(&hfdcan1, CAN1_Motor_Callback);

    // [原代码] FDCAN2 初始化 — DM6220 已移至 CAN1
    // CAN_Init(&hfdcan2, CAN2_Motor_Callback);

    // ---- 7. DM6220 使能 ----
    osDelay(100);
    for (int i = 0; i < 4; i++)
    {
        Motor_DM6220[i].CAN_Send_Enter();
        osDelay(10);
    }

    // ---- 8. 状态初始化 ----
    Remote_Mode = 1;
    memset(Chassis_Angles, 0, sizeof(Chassis_Angles));
    memset(Chassis_Speeds, 0, sizeof(Chassis_Speeds));
    memset(target_angles,  0, sizeof(target_angles));
    memset(rot_angles,     0, sizeof(rot_angles));
    last_move_angle = 0.0f;
    memset(&G_Value,       0, sizeof(G_Value));
    memset(&G_Value_Last,  0, sizeof(G_Value_Last));
    memset(Ibus_Buffer,    0, sizeof(Ibus_Buffer));
    memset(Ibus_ChannelData, 0, sizeof(Ibus_ChannelData));

    // ---- 9. UART7 DMA 启动 (moddel=0:IBUS, moddel=1:VOFA串口) ----
    //UART_Init(&huart7, UART7_Shared_Callback);
    ibus_ready = true;
}

// ==================== FreeRTOS 任务 ====================

/**
 * @brief 底盘主任务 (1ms 周期)
 */
void Chassis_Task(void *argument) {
    (void)argument;

    Chassis_Task_Init();

    uint32_t dm_send_round = 0;
    uint32_t motor_alive   = 0;

    for (;;)
    {
        // ---- 1. M3508 + DM6220 心跳 (每1000ms) ----
        if (++motor_alive >= 1000)
        {
            motor_alive = 0;
            for (int i = 0; i < 4; i++)
            {
                Motor_3508[i].TIM_100ms_Alive_PeriodElapsedCallback();
                Motor_DM6220[i].TIM_100ms_Alive_PeriodElapsedCallback();
            }
        }

        // ---- 2. 遥控模式: 处理 IBUS 数据 ----
        if (Remote_Mode == 1)
        {
            static uint32_t center_ticks = 0;
            static bool     in_deadband = false;  // 滞回状态

            //float lx = G_Value.Lx;
            //float rx = G_Value.Rx;
            //float ry = G_Value.Ry;
            float lx = g_channel_data.Lx;
            float rx = g_channel_data.Rx;
            float ry = g_channel_data.Ry;
            float filter_vx, filter_vy;
            smooth_velocity(&smoother,rx,ry,&filter_vx,&filter_vy);
            rx = filter_vx;
            ry = filter_vy;

            //TODO:将角度改为底盘的yaw角
            float theta = Euler[0][0];  // 使用BMI088获取的yaw角
            float car_vx = cos(theta)*rx + sin(theta)*ry;
            float car_vy = -sin(theta)*rx + cos(theta)*ry;
            rx = car_vx;
            ry = car_vy;

            float dist = sqrtf(rx * rx + ry * ry);

            // 滞回死区: 进用 RC_DEADBAND, 出用 1.5x
            float exit_threshold = RC_DEADBAND * 1.5f;

            if (!in_deadband)
            {
                // 当前在控制中, 检查是否进入死区
                if (fabsf(lx) < RC_ROTATION_DEADBAND && dist < RC_DEADBAND)
                {
                    in_deadband = true;
                    center_ticks = 0;
                }
            }
            else
            {
                // 当前在死区中, 检查是否退出 (需要更大阈值)
                if (fabsf(lx) > exit_threshold || dist > exit_threshold)
                {
                    in_deadband = false;
                    center_ticks = 0;
                }
            }

            if (in_deadband)
            {
                // 松摇杆 → 3508 逐步减速 (不是急停), DM6220 延时后回零
                for (int i = 0; i < 4; i++)
                {
                    Motor_3508[i].Set_Target_Omega(0.0f);
                }

                if (++center_ticks > RC_RETURN_DELAY_TICKS)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        Motor_DM6220[i].Set_Control_Angle(0.0f);
                        target_angles[i] = 0.0f;
                        rot_angles[i]    = 0.0f;
                    }
                }
            }
            else
            {
                Chassis_ProcessRotation(lx);
                Chassis_ProcessTranslation(rx, ry, lx);
            }
        }

        // ---- 3. UART7 VOFA TX: moddel=1时发送底盘数据, moddel=0时跳过 ----
        // if (moddel == 1)
        // {
        //     Serialplot.Set_Data(8,
        //         &Chassis_Angles[0], &Chassis_Angles[1],
        //         &Chassis_Angles[2], &Chassis_Angles[3],
        //         &Chassis_Speeds[0], &Chassis_Speeds[1],
        //         &Chassis_Speeds[2], &Chassis_Speeds[3]
        //     );
        //     Serialplot.TIM_1ms_Write_PeriodElapsedCallback();
        // }

        // ---- 4. DM6220 MIT 参数设置 ----
        for (int i = 0; i < 4; i++)
        {
            Motor_DM6220[i].Set_Control_Omega(0.0f);
            Motor_DM6220[i].Set_Control_Torque(0.0f);
            Motor_DM6220[i].Set_K_P(2.0f);
            Motor_DM6220[i].Set_K_D(0.1f);
        }

        // ---- 5. DM6220 分时发送 ----
        dm_send_round = (dm_send_round + 1) % 4;
        Motor_DM6220[dm_send_round].TIM_Send_PeriodElapsedCallback();

        // ---- 6-7. M3508 PID + 测试 + CAN (moddel=1原子完成, moddel=0走原PID) ----
        if (moddel == 1)
        {
            // 每1秒切换一次方向 (count 0~1999, 每500ms切)
            static int count = 0;
            count++;
            float test_omega;
            if (count < 5000)       test_omega =  10.0f * PI;
            else if (count < 10000)  test_omega = 30.0f * PI;
            else                   { count = 0; test_omega = 20.0f * PI; }

            // for (int i = 0; i < 4; i++)
            // {
            //     Motor_3508[i].Set_Target_Omega(test_omega);
            //     Motor_3508[i].TIM_Calculate_PeriodElapsedCallback();
            // }
            Motor_3508[0].Set_Target_Omega(test_omega / 1.300f);
            Motor_3508[0].TIM_Calculate_PeriodElapsedCallback();

            Motor_3508[1].Set_Target_Omega(test_omega);
            Motor_3508[1].TIM_Calculate_PeriodElapsedCallback();

            Motor_3508[2].Set_Target_Omega(test_omega / 1.24f);
            Motor_3508[2].TIM_Calculate_PeriodElapsedCallback();

            Motor_3508[3].Set_Target_Omega(test_omega / 1.112f);
            Motor_3508[3].TIM_Calculate_PeriodElapsedCallback();




            // 每100ms发一次VOFA
            static uint32_t vofa_tick = 0;
            if (++vofa_tick >= 100)
            {
                vofa_tick = 0;
                float vofa_data[8];
                for (int i = 0; i < 4; i++)
                {
                    vofa_data[i * 2]     = Motor_3508[i].Get_Target_Omega();
                    vofa_data[i * 2 + 1] = Motor_3508[i].Get_Now_Omega();
                }
                //vofa_printf(vofa_data, 8);
            }
        }
        else
        {
            for (int i = 0; i < 4; i++)
                Motor_3508[i].TIM_Calculate_PeriodElapsedCallback();
        }

        osDelay(1);
    }
}