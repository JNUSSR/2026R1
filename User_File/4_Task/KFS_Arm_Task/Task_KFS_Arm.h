#pragma once

#include "cmsis_os2.h"

#ifdef __cplusplus
#include "kfs_arm.h"
#include "uart_command.h"
extern "C" {
#endif



void Task_KFS_Arm_Init();
void Task_KFS_Arm_Impl();

void KFS_Arms_Motors_AliveChecker();
void KFS_Arms_Motors_CAN_RxCpltCallback(uint32_t id);

#ifdef __cplusplus
}
#endif