/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Task_UART.h"
#include "Task_KFS_Arm.h"
#include "tsk_config_and_callback.h"
#include "HandleClamping_Task.h"
#include "Chassis_Task.h"
extern UART_HandleTypeDef huart7;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Task_KFS_Arm */
osThreadId_t Task_KFS_ArmHandle;
const osThreadAttr_t Task_KFS_Arm_attributes = {
  .name = "Task_KFS_Arm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_UART */
osThreadId_t Task_UARTHandle;
const osThreadAttr_t Task_UART_attributes = {
  .name = "Task_UART",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for HandleClamping_ */
osThreadId_t HandleClamping_Handle;
const osThreadAttr_t HandleClamping__attributes = {
  .name = "HandleClamping_",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Task_KFS_Arm_main(void *argument);
void Task_UART_main(void *argument);
void StartHandleClamping_Task(void *argument);
void StartTaskChassis(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  Task_Init();
  //Task_KFS_Arm_Init();
  //Task_VOFA_TX_Init();
  

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_KFS_Arm */
  Task_KFS_ArmHandle = osThreadNew(Task_KFS_Arm_main, NULL, &Task_KFS_Arm_attributes);

  /* creation of Task_UART */
  Task_UARTHandle = osThreadNew(Task_UART_main, NULL, &Task_UART_attributes);

  /* creation of HandleClamping_ */
  HandleClamping_Handle = osThreadNew(StartHandleClamping_Task, NULL, &HandleClamping__attributes);

  /* creation of Chassis */
  ChassisHandle = osThreadNew(StartTaskChassis, NULL, &Chassis_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Task_KFS_Arm_main */
/**
* @brief Function implementing the Task_KFS_Arm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_KFS_Arm_main */
void Task_KFS_Arm_main(void *argument)
{
  /* init code for USB_DEVICE */
//  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Task_KFS_Arm_main */
  /* Infinite loop */
  for(;;)
  {
    Task_KFS_Arm_Impl();
    osDelay(1);
  }
  /* USER CODE END Task_KFS_Arm_main */
}

/* USER CODE BEGIN Header_Task_UART_main */
/**
* @brief Function implementing the Task_UART thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_UART_main */
void Task_UART_main(void *argument)
{
  /* USER CODE BEGIN Task_UART_main */
  Uart_Task();
  /* Infinite loop */
  for(;;)
  {
    //Task_VOFA_TX_Impl();
    osDelay(100);
  }
  /* USER CODE END Task_UART_main */
}

/* USER CODE BEGIN Header_StartHandleClamping_Task */
/**
* @brief Function implementing the HandleClamping_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHandleClamping_Task */
void StartHandleClamping_Task(void *argument)
{
  /* USER CODE BEGIN StartHandleClamping_Task */
  HandleClamping_Task();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartHandleClamping_Task */
}

/* USER CODE BEGIN Header_StartTaskChassis */
/**
* @brief Function implementing the Chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskChassis */
void StartTaskChassis(void *argument)
{
  /* USER CODE BEGIN StartTaskChassis */

    Chassis_Task(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskChassis */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

