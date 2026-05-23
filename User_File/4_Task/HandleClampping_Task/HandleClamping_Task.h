#ifndef HANDLE_CLAMPING_TASK_H
#define HANDLE_CLAMPING_TASK_H

#include "drv_can.h"
#include "dvc_motor_dm.h"

#ifdef __cplusplus
extern "C" {
#endif

void HandleClamping_Task_Init();

void HandleClamping_Calibrate();

void HandleClamping_Task();

void Control_Callback();

void Handle_MoveToReset();

void Handle_MoveToReach();

void Handle_MoveToAbutting();

void Clamping_Close();

void Clamping_Open();


void Handle_Set_Angle(float Target);

void Handle_Set_Omega(float Target);

void printf_flag_set(uint8_t flag);

void Motor_Unload();

void Motor_Load();

#ifdef __cplusplus
}
#endif

typedef enum
{
    STATE_RCH = 0,
    STATE_ABT,
} Enum_Handle_State;

typedef enum
{
    STATE_CLOSED = 0,
    STATE_OPEN,
} Enum_Clamping_State;



void Handle_Motor_SetMode(Enum_Motor_DM_Control_Method Mode);

void Handle_CAN_Rx_Dispatch(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);
#endif /*HANDLE_CLAMPING_TASK_H*/