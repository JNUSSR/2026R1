/**
 * @file tsk_config_and_callback.cpp
 * @author 
 * @brief 临时任务调度测试用函数, 后续用来存放个人定义的回调函数以及若干任务
 *        Task_Init在freertos.c的MX_FREERTOS_Init中被调用, 用于初始化回调函数等
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "tsk_config_and_callback.h"

#include "HandleClamping_Task.h"
#include "Task_KFS_Arm.h"
#include "Task_UART.h"
#include "bsp_bmi088.h"
#include "fdcan.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "uart_printf.h"

#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "stm32h7xx_hal_uart.h"
#include "usart.h"
#include <cstdint>

#include "Chassis_Task.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
extern Class_BMI088 BSP_BMI088;
Class_Matrix_f32<3, 1> Euler;

uint8_t rx_buffer[64] = {0}; // UART接收缓冲区
// 全局初始化完成标志位
bool init_finished = false;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief CAN1回调函数
 *
 *
 */
// void CAN1_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
// {
//     switch (Header.Identifier)
//     {
//         case (0x01):
//         {
//             //Handle_CAN_Rx_Dispatch(Header, Buffer);
//             break;
//         }
//         default:
//         {
//             break;
//         }
//     }
// }

/**
 * @brief CAN2回调函数
 *
 *
 */
void CAN2_Callback(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    switch (Header.Identifier)
    {
    default:
    {
        break;
    }
    }
    KFS_Arms_Motors_CAN_RxCpltCallback(Header.Identifier);
    Handle_CAN_Rx_Dispatch(Header, Buffer);
}

/**
 * @brief SPI2回调函数
 *
 *
 */
void SPI2_Callback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
    BSP_BMI088.SPI_RxCpltCallback();
}

/**
 * @brief 每3600s调用一次
 *
 */
void Task3600s_Callback()
{
    SYS_Timestamp.TIM_3600s_PeriodElapsedCallback();
}

/**
 * @brief 每1s调用一次
 *
 */
void Task1s_Callback()
{
}

/**
 * @brief 每1ms调用一次
 *
 */
void Task1ms_Callback()
{
    if(!init_finished)
        return;
    static uint32_t tick_counter = 0;
    static uint32_t mod10 = 0;
    tick_counter++;
    // Task_KFS_Arm_Impl();
    // Can定时发送
    TIM_1ms_CAN_PeriodElapsedCallback();
    Euler = BSP_BMI088.Get_Euler_Angle();

    if(tick_counter % 100 == 0){
        // Task_VOFA_TX_Impl();
        KFS_Arms_Motors_AliveChecker();
    }
    
    
    // 喂狗
    TIM_1ms_IWDG_PeriodElapsedCallback();
}

/**
 * @brief 每125us调用一次
 *
 */
void Task125us_Callback()
{
    BSP_BMI088.TIM_125us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 每10us调用一次
 *
 */
void Task10us_Callback()
{
    BSP_BMI088.TIM_10us_Calculate_PeriodElapsedCallback();
}

/**
 * @brief 初始化任务
 *
 */
void Task_Init()
{
    SYS_Timestamp.Init(&htim5);

    // 使能5V电源
    HAL_GPIO_WritePin(DC5__OUTPUT_GPIO_Port, DC5__OUTPUT_Pin, GPIO_PIN_SET);

     // 定时器中断初始化
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim5);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim8);
    SPI_Init(&hspi2,SPI2_Callback);
    BSP_BMI088.Init();
    // HandleClamping_Task_Init();



    // // 电机的CAN
    // CAN_Init(&hfdcan1, CAN1_Callback);

    CAN_Init(&hfdcan2, CAN2_Callback);
    Task_KFS_Arm_Init();
    // 夹取武器杆任务初始化，该任务初始化中调用了HAL_Delay，因此必须在其他启用其他外设中断之前调用（如串口初始化等）
    HandleClamping_Task_Init();

    //Chassis_Task_Init();
    Task_VOFA_TX_Init();

    // 标记初始化完成
    init_finished = true;
}

/**
 * @brief 前台循环任务
 *
 */
void Task_Loop()
{
}

/**
 * @brief GPIO中断回调函数
 *
 * @param GPIO_Pin 中断引脚
 */
extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (!init_finished)
    {
        return;
    } 

    if(GPIO_Pin == BMI088_ACCEL__INTERRUPT_Pin || GPIO_Pin == BMI088_GYRO__INTERRUPT_Pin)
    {
        BSP_BMI088.EXTI_Flag_Callback(GPIO_Pin);
    }
}

/**
 * @brief 定时器中断回调函数
 *
 * @param htim
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (!init_finished)
    {
        return;
    }

    // 选择回调函数
    if (htim->Instance == TIM4)
    {
        Task10us_Callback();
    }
    else if (htim->Instance == TIM5)
    {
        Task3600s_Callback();
    }
    else if (htim->Instance == TIM6)
    {
        Task1s_Callback();
    }
    else if (htim->Instance == TIM7)
    {
        Task1ms_Callback();
    }
    else if (htim->Instance == TIM8)
    {
        Task125us_Callback();
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
