#include "HandleClamping_Task.h"
#include "HandleClamping.h"
#include "dvc_motor_dm.h"
#include "uart_command.h"
#include "cmsis_os2.h"


//test111 
#include "uart_printf.h"

Class_HandleClamping HandleClamping_ctrl(PID_DIRECTLY_MODE);

//串口打印标志位
uint8_t printf_flag = 0;

/******************************************************************************
 * @brief 夹取武器杆任务初始化
 * 
 ******************************************************************************/
void HandleClamping_Task_Init()
{
    HandleClamping_ctrl.Init();
}

/******************************************************************************
 * @brief 夹取武器杆机构点击位置较准
 * 
 ******************************************************************************/
void HandleClamping_Calibrate()
{
    HandleClamping_ctrl.Start_Calibrate();
}

/******************************************************************************
 * @brief 夹取武器杆任务入口
 * 
 ******************************************************************************/
void HandleClamping_Task()
{
    

    for(;;)
    {
            // 消费按钮队列 → 触发状态转移（单次调用）
        uint8_t btn_raw;
        static Enum_Handle_State handle_state = STATE_ABT;
        static Enum_Clamping_State clamping_state = STATE_CLOSED;
        while (osMessageQueueGet(g_ctrl_handleclamping_queue, &btn_raw, NULL, 0U) == osOK)
        {
            auto btn = static_cast<CtrlButtons>(btn_raw);
            switch (btn)
            {
                case Btn_HandleRotate:
                {
                    if (handle_state == STATE_RCH)
                    {
                        Handle_MoveToAbutting();
                        handle_state = STATE_ABT;
                    }
                    else if(handle_state == STATE_ABT)
                    {
                        Handle_MoveToReach();
                        handle_state = STATE_RCH;
                    }
                    break;
                }
                case Btn_Clamping:
                {
                    if(clamping_state == STATE_CLOSED)
                    {
                        Clamping_Open();
                        clamping_state = STATE_OPEN;
                    }
                    else if(clamping_state == STATE_OPEN)
                    {
                        Clamping_Close();
                        clamping_state = STATE_CLOSED;
                    }
                    break;
                }

                break;
            default:
                break;
            }
        }


        static int timestamp = 0;
        static int mod10 = 0;
        if(printf_flag && mod10 >= 10)
        {
            //uart_printf("%f,%f\n", HandleClamping_ctrl.Get_Motor_Handle_2().Get_Now_Torque(), HandleClamping_ctrl.Get_Motor_Handle_2().Get_Now_Angle());
            mod10 = 0;
        }
        if(HandleClamping_ctrl.Get_State() == STATE_CALIBRATING)
        {
            HandleClamping_ctrl.Calibrate();
        }
        HandleClamping_ctrl.TIM_1ms_Control_PeriodElapsedCallback();

        mod10++;
        timestamp++;
        osDelay(1); // 每1ms执行一次

    }
}

/******************************************************************************
 * @brief 直接发送控制报文
 * 
 ******************************************************************************/
void Control_Callback()
{
    HandleClamping_ctrl.TIM_1ms_Control_PeriodElapsedCallback();
}

/******************************************************************************
 * @brief 设置电机目标角度为武器杆垂直时的角度
 * 
 ******************************************************************************/
void Handle_MoveToReset()
{
    HandleClamping_ctrl.Motor_Handle_Set_Angle(HandleClamping_ctrl.Get_Angle_Reset());
}

/******************************************************************************
 * @brief 设置电机目标角度为夹取武器杆时的角度
 * 
 ******************************************************************************/
void Handle_MoveToReach()
{
    HandleClamping_ctrl.Motor_Handle_Set_Angle(HandleClamping_ctrl.Get_Angle_Reach());
}

/******************************************************************************
 * @brief 设置电机目标角度为对接时的角度
 * 
 ******************************************************************************/
void Handle_MoveToAbutting()
{
    HandleClamping_ctrl.Motor_Handle_Set_Angle(HandleClamping_ctrl.Get_Angle_Abutting());
}

/******************************************************************************
 * @brief 闭合夹爪
 * 
 ******************************************************************************/
void Clamping_Close()
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
}

/******************************************************************************
 * @brief 打开夹爪
 * 
 ******************************************************************************/
void Clamping_Open()
{
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
}

/******************************************************************************
 * @brief 设置电机目标角度外部接口
 * 
 * @param Target 目标速度，rad
 ******************************************************************************/
void Handle_Set_Angle(float Target)
{
    HandleClamping_ctrl.Get_Motor_Handle_2().Set_Control_Angle(Target);
}

/******************************************************************************
 * @brief 设置电机目标角速度外部接口
 * 
 * @param Target 目标速度，rad/s
 ******************************************************************************/
void Handle_Set_Omega(float Target)
{
    HandleClamping_ctrl.Get_Motor_Handle_2().Set_Control_Omega(Target);
}

/******************************************************************************
 * @brief CAN通信接收回调函数
 * 
 * @param Header CAN接收消息头
 * @param Buffer CAN接收数据缓冲区
 * 
 * @note 该函数会被CAN总线接收中断调用, 在tsk_config_and_callback.cpp中调用
 ******************************************************************************/
void Handle_CAN_Rx_Dispatch(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    if (Header.Identifier == 0 || Buffer == nullptr)
    {
        return;
    }
    HandleClamping_ctrl.CAN_RxCallback(Header.Identifier, Buffer);
}

/******************************************************************************
 * @brief 串口打印使能标志位设置
 * 
 * @note 测试用
 ******************************************************************************/
void printf_flag_set(uint8_t flag)
{
    printf_flag = flag;
}

/******************************************************************************
 * @brief 电机失能卸力
 * 
 * @note 测试用
 ******************************************************************************/
void Motor_Unload()
{
    HandleClamping_ctrl.Get_Motor_Handle_2().CAN_Send_Exit();
}
/******************************************************************************
 * @brief 电机使能上电
 * 
 * @note 测试用
 ******************************************************************************/
void Motor_Load()
{
    HandleClamping_ctrl.Get_Motor_Handle_2().CAN_Send_Enter();
}
/******************************************************************************
 * @brief 切换电机控制模式
 * 
 * @note 测试用
 ******************************************************************************/
void Handle_Motor_SetMode(Enum_Motor_DM_Control_Method Mode)
{
    HandleClamping_ctrl.Get_Motor_Handle_2().Mode_Switch(Mode);
}