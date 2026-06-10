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
Class_Motor_DJI_C610 *now_motor = &arm1_joint1_motor; // �������õĵ�ǰѡ�����ָ��

// UART ?1?7?KFS_Arm ��������У��ж���������KFS_Arm ��������?1?7?
osMessageQueueId_t g_arm_cmd_queue = nullptr;

osMessageQueueId_t g_ctrl_btn_queue = nullptr;
// ���ƶ��м�ȡ�����˻�������
osMessageQueueId_t g_ctrl_handleclamping_queue = nullptr;
osMessageQueueId_t g_kfs_queue = nullptr;
osMessageQueueId_t g_adc_queue = nullptr;

osSemaphoreId_t uartSemaphoreHandle;
// �ϵ��ʼ��������־���յ� "init1" ����λ
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

extern "C" void Task_UART_Init(void) {
    g_ctrl_btn_queue = osMessageQueueNew(1, sizeof(uint8_t), NULL);
    g_ctrl_handleclamping_queue = osMessageQueueNew(1, sizeof(uint8_t), NULL);
    g_kfs_queue = osMessageQueueNew(1, sizeof(uint8_t[3][4]), NULL);
    // g_adc_queue = osMessageQueueNew(1,sizeof(mavlink_adc_t), NULL);
    UART_Init(&huart7, UartCallback);
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
                        // CtrlButtons ��Ч��Χ��Btn1(3) ~ Btn4(6)
                        if (act_val >= Btn_Hand1Release && act_val <= Btn_Hand2Tighten) {
                            osMessageQueuePut(g_ctrl_btn_queue, &act_val, 0U, 0U);
                        } else {
                            osMessageQueuePut(g_ctrl_handleclamping_queue, &act_val, 0U, 0U);
                        }
                    }
                    else if (mav_msg.msgid == MAVLINK_MSG_ID_KFS) {
                        uint32_t kfs_val = mavlink_msg_kfs_get_kfs(&mav_msg);
                        // CtrlButtons ��Ч��Χ��Btn1(3) ~ Btn4(6)
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
