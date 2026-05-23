#include "Task_UART.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "ADC_TO_CHANNEL.h"
#include "drv_uart.h"
#include "dvc_motor_dji.h"
#include "kfs_arm.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_uart_ex.h"
#include "uart_command.h"
#include "usart.h"

constexpr uint8_t kJustFloatChannelCount = 11;
constexpr uint8_t kRxLineBufferSize = 96;
constexpr uint8_t kTxFrameTail[4] = {0x00, 0x00, 0x80, 0x7f};
uint16_t rx_len = 0;
Class_Motor_DJI_C610 *now_motor1 = &arm2_joint1_motor;
Class_Motor_DJI_C610 *now_motor2 = &arm2_joint2_motor;
Class_Motor_DJI_C610 *now_motor = &arm1_joint1_motor; // 供调试用的当前选定电机指针

// UART ?1?7?KFS_Arm 的命令队列（中断中生产，KFS_Arm 任务消费?1?7?
osMessageQueueId_t g_arm_cmd_queue = nullptr;

osMessageQueueId_t g_ctrl_btn_queue = nullptr;
// 控制队列夹取武器杆机构队列
osMessageQueueId_t g_ctrl_handleclamping_queue = nullptr;
osMessageQueueId_t g_kfs_queue = nullptr;
osMessageQueueId_t g_adc_queue = nullptr;

osSemaphoreId_t uartSemaphoreHandle;
// 上电初始化保护标志，收到 "init1" 后置位
volatile bool g_arm_init1_received = false;
volatile __attribute__((section(".ram_d2"))) uint8_t UART_Rx_Buffer[kRxLineBufferSize] = {0};

void Send_JustFloatFrame(const float *channels, uint8_t channel_count) {
    uint8_t tx_buffer[sizeof(float) * kJustFloatChannelCount + sizeof(kTxFrameTail)] = {0};
    std::memcpy(tx_buffer, channels, sizeof(float) * channel_count);
    std::memcpy(tx_buffer + sizeof(float) * channel_count, kTxFrameTail, sizeof(kTxFrameTail));
    UART_Transmit_Data(&huart7, tx_buffer, static_cast<uint16_t>(sizeof(float) * channel_count + sizeof(kTxFrameTail)));
}

void UartCallback(uint8_t *Buffer, uint16_t Length) {
    rx_len = Length;
    osSemaphoreRelease(uartSemaphoreHandle);
}

// void UART7_Callback(uint8_t *Buffer, uint16_t Length);

extern "C" void Task_VOFA_TX_Init(void) {
#if DEBUG_MODE == 1
    g_arm_cmd_queue = osMessageQueueNew(16, sizeof(ArmUartCommand), NULL);
#else
    g_ctrl_btn_queue = osMessageQueueNew(1, sizeof(uint8_t), NULL);
    g_ctrl_handleclamping_queue = osMessageQueueNew(1, sizeof(uint8_t), NULL);
    g_kfs_queue = osMessageQueueNew(1, sizeof(uint8_t[3][4]), NULL);
    // g_adc_queue = osMessageQueueNew(1,sizeof(mavlink_adc_t), NULL);
#endif
    UART_Init(&huart7, UartCallback);
}


extern "C" void Task_VOFA_TX_Impl(void) {
    float channels[kJustFloatChannelCount] = {0.0f};

    channels[0] = now_motor1->Get_Target_Angle();
    channels[1] = now_motor1->Get_Now_Angle();
    channels[2] = now_motor2->Get_Target_Angle();
    channels[3] = now_motor2->Get_Now_Angle();

    channels[4] = now_motor1->Get_Now_Angle();
    channels[5] = now_motor1->Get_Now_Omega();
    channels[6] = now_motor1->Get_Now_Torque();
    channels[7] = now_motor2->Get_Now_Angle();
    channels[8] = now_motor2->Get_Now_Omega();
    channels[9] = now_motor2->Get_Now_Torque();

    // Send_JustFloatFrame(channels, kJustFloatChannelCount);
}

void Uart_Task() {
    for (;;) {
        osSemaphoreAcquire(uartSemaphoreHandle, osWaitForever);
        if (rx_len > 0) {
            static mavlink_status_t mav_status;
            mavlink_message_t mav_msg;

            for (uint16_t i = 0; i < rx_len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, UART7_Manage_Object.Rx_Buffer_Ready[i], &mav_msg,&mav_status)) {
                    if (mav_msg.msgid == MAVLINK_MSG_ID_ACT) {
                        uint8_t act_val = mavlink_msg_act_get_act(&mav_msg);
                        // CtrlButtons 有效范围：Btn1(3) ~ Btn4(6)
                        if (act_val >= Btn_ArmSwitch && act_val <= Btn_Hand) {
                            osMessageQueuePut(g_ctrl_btn_queue, &act_val, 0U, 0U);
                        } else {
                            osMessageQueuePut(g_ctrl_handleclamping_queue, &act_val, 0U, 0U);
                        }
                    }
                    else if (mav_msg.msgid == MAVLINK_MSG_ID_KFS) {
                        uint32_t kfs_val = mavlink_msg_kfs_get_kfs(&mav_msg);
                        // CtrlButtons 有效范围：Btn1(3) ~ Btn4(6)
                        uint8_t kfs_matrix[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

                        for (int i = 0; i < 3; i++) {
                            for (int j = 0; j < 4; j++) {
                                kfs_matrix[i][j] = (uint8_t) (kfs_val >> ((i * 4 + j) * 2)) & 0x03;
                            }
                        }
                        osMessageQueuePut(g_kfs_queue, &kfs_matrix, 0U, 0U);
                    }
                    else if (mav_msg.msgid == MAVLINK_MSG_ID_adc) {
                        mavlink_adc_t adc_val;
                        mavlink_msg_adc_decode(&mav_msg, &adc_val);
                        ADC_to_Channel(adc_val.adc1, adc_val.adc2, adc_val.adc3);
                        // osMessageQueuePut(g_adc_queue, &adc_val, 0U, 0U);
                    }
                }
            }
            rx_len = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
        }
    }
}


// void UART7_Callback(uint8_t *Buffer, uint16_t Length) {
// #if DEBUG_MODE == 0
//     // ===== Mavlink 协议解析模式 =====
//     static mavlink_status_t mav_status;
//     mavlink_message_t mav_msg;
//
//     for (uint16_t i = 0; i < Length; i++) {
//         if (mavlink_parse_char(MAVLINK_COMM_0, Buffer[i], &mav_msg,
//                                &mav_status)) {
//             if (mav_msg.msgid == MAVLINK_MSG_ID_ACT) {
//                 uint8_t act_val = mavlink_msg_act_get_act(&mav_msg);
//                 // CtrlButtons 有效范围：Btn1(3) ~ Btn4(6)
//                 if (act_val >= Btn_ArmSwitch && act_val <= Btn_Hand) {
//                     osMessageQueuePut(g_ctrl_btn_queue, &act_val, 0U, 0U);
//                 } else {
//                     osMessageQueuePut(g_ctrl_handleclamping_queue, &act_val, 0U, 0U);
//                 }
//             }
//             if (mav_msg.msgid == MAVLINK_MSG_ID_KFS) {
//                 uint32_t kfs_val = mavlink_msg_kfs_get_kfs(&mav_msg);
//                 // CtrlButtons 有效范围：Btn1(3) ~ Btn4(6)
//                 uint8_t kfs_matrix[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
//
//                 for (int i = 0; i < 3; i++) {
//                     for (int j = 0; j < 4; j++) {
//                         kfs_matrix[i][j] = (uint8_t) (kfs_val >> ((i * 4 + j) * 2)) & 0x03;
//                     }
//                 }
//                 osMessageQueuePut(g_kfs_queue, &kfs_matrix, 0U, 0U);
//             }
//             if (mav_msg.msgid == MAVLINK_MSG_ID_adc) {
//                 mavlink_adc_t adc_val;
//                 mavlink_msg_adc_decode(&mav_msg, &adc_val);
//                 ADC_to_Channel(adc_val.adc1, adc_val.adc2, adc_val.adc3);
//                 // osMessageQueuePut(g_adc_queue, &adc_val, 0U, 0U);
//             }
//         }
//     }
// #else
//     // ===== 原有文本协议解析模式 =====
//     HAL_UART_Transmit(&huart7, (uint8_t *) Buffer, Length, 1000);
//     char tmp[64];
//     size_t copy_len =
//             (static_cast<size_t>(Length) < 63U) ? static_cast<size_t>(Length) : 63U;
//     std::memcpy(tmp, (uint8_t *) Buffer, copy_len);
//     tmp[copy_len] = '\0';
//     char *cmd = std::strtok(tmp, " ");
//     char *param = std::strtok(nullptr, " ");
//     if (cmd != nullptr) {
//         // ===== 机械臂指令（cmd1/cmd2）：入队，由 KFS_Arm 任务消费 =====
//         if (std::strcmp(cmd, "cmd1") == 0 || std::strcmp(cmd, "cmd2") == 0) {
//             ArmUartCommand uart_cmd;
//             uart_cmd.arm = (cmd[3] == '1') ? 0U : 1U;
//             uart_cmd.action = static_cast<uint8_t>(std::strtol(param, nullptr, 10));
//             osMessageQueuePut(g_arm_cmd_queue, &uart_cmd, 0U, 0U);
//         } // ===== 上电初始化允许指令：设置标志，KFS_Arm 任务消费后执行
//         // InitOnActivate =====
//         else if (std::strcmp(cmd, "init1") == 0) {
//             g_arm_init1_received = true;
//         } // ===== 单电机 PID 调试指令：中断中直接执行（后续再迁移） =====
//         else if (std::strcmp(cmd, "TA") == 0) {
//             now_motor->Set_Target_Angle(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "TO") == 0) {
//             now_motor->Set_Target_Omega(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "TT") == 0) {
//             now_motor->Set_Target_Torque(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "A_P") == 0) {
//             now_motor->PID_Angle.Set_K_P(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "A_I") == 0) {
//             now_motor->PID_Angle.Set_K_I(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "A_D") == 0) {
//             now_motor->PID_Angle.Set_K_D(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "O_P") == 0) {
//             now_motor->PID_Omega.Set_K_P(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "O_I") == 0) {
//             now_motor->PID_Omega.Set_K_I(std::strtof(param, nullptr));
//         } else if (std::strcmp(cmd, "O_D") == 0) {
//             now_motor->PID_Omega.Set_K_D(std::strtof(param, nullptr));
//         }
//         HAL_UART_Transmit(&huart7, (uint8_t *) "RCV", 5, 1000);
//     }
// #endif
// }
