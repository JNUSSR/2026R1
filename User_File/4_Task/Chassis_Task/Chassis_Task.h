/**
 * @file Chassis_Task.h
 * @brief 舵轮底盘控制任务 —— 串口绘图 + IBUS遥控 + 大疆3508(CAN1) + 达妙6220(CAN2)
 *
 * 架构：
 *   UART7 ─┬─ 调试模式(TX/RX): 串口绘图(Serialplot)
 *           └─ 遥控模式(RX):    IBUS遥控器
 *   FDCAN1 ─── 大疆 M3508 电机 (x4)  — 速度轮
 *   FDCAN2 ─── 达妙 DM6220 电机 (x4) — 方向舵
 *
 * @version 1.0
 * @date 2026-05-14
 */

#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>

/* Exported function declarations --------------------------------------------*/

void Chassis_Task_Init(void);
void Chassis_Task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // CHASSIS_TASK_H
