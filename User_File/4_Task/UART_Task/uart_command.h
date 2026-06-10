#pragma once

#include <cstdint>
#include "cmsis_os2.h"

#include "2_Device/Plotter/Mavlink/remote_control/mavlink.h"

enum CtrlButtons {
    Btn_HandleRotate = 1,
    Btn_Clamping = 2,
    Btn_Hand1Release = 3,
    Btn_Hand2Release = 4,
    Btn_Hand1Tighten = 5,
    Btn_Hand2Tighten = 6,
    NO_BTN = 0
};


/** @brief Mavlink → CtrlButtons 的按钮队列句柄，在 Task_UART.cpp 中创建 */
extern osMessageQueueId_t g_ctrl_btn_queue;
extern osMessageQueueId_t g_kfs_queue;
extern osMessageQueueId_t g_ctrl_handleclamping_queue;
