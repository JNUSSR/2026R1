#pragma once

#include <cstdint>
#include "cmsis_os2.h"

#if DEBUG_MODE == 0
#include "2_Device/Plotter/Mavlink/remote_control/mavlink.h"
#endif
/**
 * @brief UART 命令模式选择
 *
 * DEBUG_MODE = 0: 使用 Mavlink 协议解析（通过 mavlink_msg_act.h）
 * DEBUG_MODE = 1: 使用原有文本协议解析（cmd1/cmd2/init1/TA/TO/...）
 */
#define DEBUG_MODE 0

#if DEBUG_MODE == 1
/**
 * @brief UART 下发的机械臂动作指令
 *
 * UART 中断中解析字符串后生产此结构体入队，
 * KFS_Arm 任务消费并执行。
 */
struct ArmUartCommand {
    uint8_t arm;    /**< 0 = ARM1, 1 = ARM2 */
    uint8_t action; /**< 0-8, 与 UART 协议中 cmd1/cmd2 的数值一致 */
};

/** @brief UART → KFS_Arm 的命令队列句柄，在 Task_UART.cpp 中创建 */
extern osMessageQueueId_t g_arm_cmd_queue;

/**
 * @brief 上电初始化保护标志
 *
 * UART 收到 "init1" 后由中断设置，KFS_Arm 任务消费后清零。
 * 仅在置位后才会执行 InitOnActivate 上电动作序列，防止 MCU 上电瞬间机械臂突然动作。
 */
extern volatile bool g_arm_init1_received;
#else
enum CtrlButtons {
    Btn_HandleRotate = 1,
    Btn_Clamping = 2,
    Btn_ArmSwitch = 3,
    Btn_Joint1 = 4,
    Btn_Joint2 = 5,
    Btn_Hand = 6,
    NO_BTN = 0
};


/** @brief Mavlink → CtrlButtons 的按钮队列句柄，在 Task_UART.cpp 中创建 */
extern osMessageQueueId_t g_ctrl_btn_queue;
extern osMessageQueueId_t g_kfs_queue;
extern osMessageQueueId_t g_ctrl_handleclamping_queue;
#endif
