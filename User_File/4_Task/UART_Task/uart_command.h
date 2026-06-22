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
};//TODO: 可能不再需要

enum HandClampingCmd {
    HandleRotate = 1,
    Clamping = 2,
};

union SuckerCmd_t {
    uint8_t raw;
    struct {
        bool sucker_a : 1;
        bool sucker_b : 1;
        bool up : 1;
        bool down : 1;
        bool arm_switch : 1;
        uint8_t reserved : 3;
    };
};


/** @brief Mavlink → CtrlButtons 的按钮队列句柄，在 Task_UART.cpp 中创建 */
extern osMessageQueueId_t g_sucker_ctrl_queue;
extern osMessageQueueId_t g_encoder0_queue;
extern osMessageQueueId_t g_encoder1_queue;
extern osMessageQueueId_t g_encoder2_queue;
extern osMessageQueueId_t g_encoder3_queue;
extern osMessageQueueId_t g_kfs_queue;
extern osMessageQueueId_t g_ctrl_handleclamping_queue;
