#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os2.h"

void Task_UART_Init(void);

void Uart_Task();

#ifdef __cplusplus
}

#include "Task_KFS_Arm.h"
extern Class_Motor_DJI_C610 arm1_joint1_motor;
extern Class_Motor_DJI_C610 arm1_joint2_motor;
extern Class_Motor_DJI_C610 arm2_joint1_motor;
extern Class_Motor_DJI_C610 arm2_joint2_motor;

// extern KFS_Arm::Joint1_Pos arm1_joint1_target;
// extern KFS_Arm::Joint2_Pos arm1_joint2_target;
// extern KFS_Arm::HandCmd arm1_hand_cmd;

// extern KFS_Arm::Joint1_Pos arm2_joint1_target;
// extern KFS_Arm::Joint2_Pos arm2_joint2_target;
// extern KFS_Arm::HandCmd arm2_hand_cmd;

extern KFS_Arm arm1_kfs_arm;
extern KFS_Arm arm2_kfs_arm;


#endif
