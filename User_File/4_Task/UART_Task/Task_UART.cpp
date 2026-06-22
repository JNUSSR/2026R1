#include "Task_UART.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include "ADC_TO_CHANNEL.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "drv_uart.h"
#include "dvc_motor_dji.h"
#include "kfs_arm.h"
#include "stm32h7xx_hal_uart.h"
#include "stm32h7xx_hal_uart_ex.h"
#include "uart_command.h"
#include "usart.h"

uint16_t rx_len = 0;

// UART ?1?7?KFS_Arm ��������У��ж���������KFS_Arm ��������?1?7?
osMessageQueueId_t g_arm_cmd_queue = nullptr;
osMessageQueueId_t g_encoder0_queue = nullptr;
osMessageQueueId_t g_encoder1_queue = nullptr;
osMessageQueueId_t g_encoder2_queue = nullptr;
osMessageQueueId_t g_encoder3_queue = nullptr;
osMessageQueueId_t g_sucker_ctrl_queue = nullptr;
// ���ƶ��м�ȡ�����˻�������
osMessageQueueId_t g_ctrl_handleclamping_queue = nullptr;
osMessageQueueId_t g_kfs_queue = nullptr;
osMessageQueueId_t g_kfs_put_queue = nullptr;
osMessageQueueId_t g_adc_queue = nullptr;

osSemaphoreId_t uartSemaphoreHandle;
// �ϵ��ʼ��������־���յ� "init1" ����λ
volatile bool g_arm_init1_received = false;

void UartCallback(uint8_t *Buffer, uint16_t Length) {
    rx_len = Length;
    osSemaphoreRelease(uartSemaphoreHandle);
}

// void UART7_Callback(uint8_t *Buffer, uint16_t Length);

extern "C" void Task_UART_Init(void) {
    g_sucker_ctrl_queue = osMessageQueueNew(1, sizeof(uint8_t), NULL);
    g_ctrl_handleclamping_queue = osMessageQueueNew(1, sizeof(uint8_t), NULL);
    g_kfs_queue = osMessageQueueNew(1, sizeof(uint8_t[3][4]), NULL);
    g_kfs_put_queue = osMessageQueueNew(1, sizeof(uint8_t[2][3]), NULL);
    g_encoder0_queue = osMessageQueueNew(3, sizeof(int16_t), NULL);
    g_encoder1_queue = osMessageQueueNew(3, sizeof(int16_t), NULL);
    g_encoder2_queue = osMessageQueueNew(3, sizeof(int16_t), NULL);
    g_encoder3_queue = osMessageQueueNew(3, sizeof(int16_t), NULL);

    // g_adc_queue = osMessageQueueNew(1,sizeof(mavlink_adc_t), NULL);
    UART_Init(&huart7, UartCallback);
}

union ActionCmd_t {
    uint8_t raw;
    struct {
        bool sucker_a : 1;
        bool claw : 1;
        bool sucker_b : 1;
        bool act4 : 1;
        bool act5 : 1;
        bool act6 : 1;
        uint8_t reserved : 2;
    };
};


void Uart_Task() {
    for (;;) {
        osSemaphoreAcquire(uartSemaphoreHandle, osWaitForever);
        if (rx_len > 0) {
            static mavlink_status_t mav_status;
            mavlink_message_t mav_msg;

            for (uint16_t i = 0; i < rx_len; i++) {
                if (mavlink_parse_char(MAVLINK_COMM_0, UART7_Manage_Object.Rx_Buffer_Ready[i], &mav_msg,&mav_status)) {
                    if (mav_msg.msgid == MAVLINK_MSG_ID_REMOTE_CONTROL_STATE) {
                            //旋钮
                            int16_t encoder_0 = mavlink_msg_remote_control_state_get_encoder_0(&mav_msg);
                            int16_t encoder_1 = mavlink_msg_remote_control_state_get_encoder_1(&mav_msg);
                            int16_t encoder_2 = mavlink_msg_remote_control_state_get_encoder_2(&mav_msg);
                            int16_t encoder_3 = mavlink_msg_remote_control_state_get_encoder_3(&mav_msg);

                            osMessageQueuePut(g_encoder0_queue, &encoder_0, 0U, 0U);
                            osMessageQueuePut(g_encoder1_queue, &encoder_1, 0U, 0U);

                            osMessageQueuePut(g_encoder2_queue, &encoder_2, 0U, 0U);
                            osMessageQueuePut(g_encoder3_queue, &encoder_3, 0U, 0U);
                            //动作
                            ActionCmd_t action_cmd;
                            action_cmd.raw = mavlink_msg_remote_control_state_get_act(&mav_msg);

                            SuckerCmd_t sucker_cmd;
                            sucker_cmd.sucker_a = action_cmd.sucker_a;
                            sucker_cmd.sucker_b = action_cmd.sucker_b;
                            sucker_cmd.up = action_cmd.act4;
                            sucker_cmd.down = action_cmd.act6;
                            sucker_cmd.arm_switch = action_cmd.act5;
                            osMessageQueuePut(g_sucker_ctrl_queue, &sucker_cmd, 0U, 0U);

                            //TODO 夹爪命令解析
                            uint8_t handle_cmd = 0;
                            osMessageQueuePut(g_ctrl_handleclamping_queue, &handle_cmd, 0U, 0U);
                            
                            // 梅林矩阵解包：4行3列，每元素2位
                            uint32_t kfs_val = mavlink_msg_remote_control_state_get_kfs(&mav_msg);
                            uint8_t kfs_matrix[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

                            for (int i = 0; i < 3; i++) {
                                for (int j = 0; j < 4; j++) {
                                    kfs_matrix[i][j] = (uint8_t) (kfs_val >> ((i * 4 + j) * 2)) & 0x03;
                                }
                            }
                            osMessageQueuePut(g_kfs_queue, &kfs_matrix, 0U, 0U);
                            // 九宫格解包：2行3列，每元素1位
                            uint8_t kfs_put_val = mavlink_msg_remote_control_state_get_kfs_put(&mav_msg);
                            uint8_t kfs_put_matrix[2][3] = {{0, 0, 0}, {0, 0, 0}};
                            for (int i = 0; i < 2; i++) {
                                for (int j = 0; j < 3; j++) {
                                    kfs_put_matrix[i][j] = (uint8_t) (kfs_put_val >> ((i * 3 + j) * 1)) & 0x01;
                                }
                            }
                            osMessageQueuePut(g_kfs_put_queue, &kfs_put_matrix, 0U, 0U);//TODO 消费端
                            //摇杆 //TODO
                            int16_t move_0, move_1, move_2, move_3;
                            move_0 = mavlink_msg_remote_control_state_get_move_0(&mav_msg);
                            move_1 = mavlink_msg_remote_control_state_get_move_1(&mav_msg);
                            move_2 = mavlink_msg_remote_control_state_get_move_2(&mav_msg);
                            move_3 = mavlink_msg_remote_control_state_get_move_3(&mav_msg);

                            //ADC_to_Channel(adc_val.adc1, adc_val.adc2, adc_val.adc3);
                            // osMessageQueuePut(g_adc_queue, &adc_val, 0U, 0U);

                            //动作模式 //TODO
                    }
                }
            }
            rx_len = 0;
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, UART7_Manage_Object.Rx_Buffer_Active, UART_BUFFER_SIZE);
        }
    }
}
