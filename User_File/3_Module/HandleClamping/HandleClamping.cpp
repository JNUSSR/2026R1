#include "HandleClamping.h"
#include "Cylinder.h"


/******************************************************************************
 * @brief 武器杆夹模块初始化
 * 
 * @note 该函数会初始化达妙电机3519，将类中原本对4310做的默认参数进行修改做对应的适配；PID等相关内容 
 ******************************************************************************/
void Class_HandleClamping::Init()
{
    //初始化时，CANTx_ID直接填上位机的的CAN_ID,初始化中会根据选中的模式自动调整发送CAN_ID
    
    //初始化电机
    Motor_Handle_2_.Init(&hfdcan2, HANDLE_MOTOR_MST_ID, HANDLE_MOTOR_TX_ID, Motor_DM_Control_Method_NORMAL_OMEGA,  200.0f, 200.0f, 10.0f, 20.522388f);
    Motor_Handle_2_.Mode_Switch(Motor_DM_Control_Method_NORMAL_OMEGA);
    Motor_Handle_2_.CAN_Send_Save_Zero();
    Start_Calibrate();
    // Motor_Handle_2_.Init(&hfdcan1, HANDLE_MOTOR_MST_ID, HANDLE_MOTOR_TX_ID, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA,  60.288f, 200.0f, 10.0f, 20.522388f);
    // Motor_Handle_2_.Mode_Switch(Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
    //获取并设置软零点位置
    // Motor_Handle_2_.Set_Control_Omega(HANDLE_MOTOR_CONTROL_OMEGA);
    //设置三个模式的位置
    // float soft_zero = Motor_Handle_2_.Get_Now_Angle();
    // Angle_Reset_ = soft_zero + HANDLE_MOTOR_INIT_INCREANGLE; //根据电机的实际转动方向选择是Pos_Max_,是加还是减
    // Angle_Reach_ = Angle_Reset_ + HANDLE_POS_REACH_INCREANGLE;
    // Angle_Abutting_ = Angle_Reset_ + HANDLE_POS_ABUTTING_INCREANGLE;
    
    // // 刚初始化时先将目标位置设置为复位位置
    // Motor_Handle_Set_Angle(Angle_Reset_);

    //气缸GPIO口初始化
    // Handle_Cylinder_Init();

    

    
}

/******************************************************************************
 * @brief 开始校准函数
 * 
 * @note  若中途出现问题需要重新校准则调用该函数再次启动校准
 ******************************************************************************/
void Class_HandleClamping::Start_Calibrate()
{
    timestamp_start_ = 0;
    timestamp_ = 1;
    calibrate_step_ = 0;
    Motor_Handle_2_.Set_Control_Omega(HANDLE_MOTOR_CALIBRATE_OMEGA);
    Set_State(STATE_CALIBRATING);
    Motor_Handle_2_.Mode_Switch(Motor_DM_Control_Method_NORMAL_OMEGA);
}

/******************************************************************************
 * @brief 电机上电后的位置软校准函数
 * 
 * @note  用速度模式让电机正转和反转，找到两个方向的机械限位位置，从而得到最大最小位置，
 *        进而得到零位位置
 ******************************************************************************/
void Class_HandleClamping::Calibrate()
{

    if(timestamp_ - timestamp_start_ > 300) //距离启动时间大于0.5秒，认为电机已经加速完成
    {
        switch(calibrate_step_)
        {
            case 0:
            {
                
                //第一步，先让电机正转，找最大位置
                if(Motor_Handle_2_.Get_Now_Torque() > HANDLE_MOTOR_CALIBRATE_TORQUE_MAX)//电机速度小于2rad/s，认为电机已经达到限位，可以进入下一步
                {
                    calibrate_step_ = 1;
                    timestamp_start_ = timestamp_;//重置电机启动加速时间戳
                    Pos_Max_ = Motor_Handle_2_.Get_Now_Angle();
                    // Motor_Handle_2_.Set_Control_Angle(45.0f);
                    Motor_Handle_2_.Set_Control_Omega(-HANDLE_MOTOR_CALIBRATE_OMEGA);
                }
                break;
            }

            case 1:
            {
                //第二步，让电机反转找最小位置
                if(Motor_Handle_2_.Get_Now_Torque() < -HANDLE_MOTOR_CALIBRATE_TORQUE_MAX)//电机速度小于2rad/s，认为电机已经达到限位，可以进入下一步
                {
                    calibrate_step_ = 2;
                    Pos_Min_ = Motor_Handle_2_.Get_Now_Angle();
                    Motor_Handle_2_.Set_Control_Omega(0.0f);
                    Motor_Handle_2_.Set_Control_Angle(Pos_Min_);
                    TIM_1ms_Control_PeriodElapsedCallback();
                }
                break;
            }

            case 2:
            {
                //校准成功，进入运行状态
                Set_State(STATE_RUNNING);
                Motor_Handle_2_.Mode_Switch(Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
                Motor_Handle_2_.Set_Control_Omega(HANDLE_MOTOR_CONTROL_OMEGA);
                // float soft_zero = Motor_Handle_2_.Get_Now_Angle();
                Angle_Reset_ = Pos_Min_ + 45.0f + HANDLE_MOTOR_INIT_INCREANGLE; //根据电机的实际转动方向选择是Pos_Max_,是加还是减
                Angle_Reach_ = Angle_Reset_ + HANDLE_POS_REACH_INCREANGLE;
                Angle_Abutting_ = Pos_Min_ + 45.0f;
                Motor_Handle_Set_Angle(Pos_Min_ + 45.0f);
                    //获取并设置软零点位置
                break;
            }

            default:
            {
                break;
            }
        }
    }
    timestamp_++;

}

/******************************************************************************
 * @brief 武器杆夹模块CAN回调函数
 * 
 * @param std_id CAN ID
 * @param data CAN数据
 * 
 * @note 该函数会先再任务接口中被封装成CAN_Callback_Dispatch,然后被CAN接收中断调用，
 *       判断CAN ID后调用对应的电机CAN回调函数
 ******************************************************************************/
void Class_HandleClamping::CAN_RxCallback(uint32_t std_id, uint8_t *data)
{
    if (std_id == HANDLE_MOTOR_MST_ID )
    {
        Motor_Handle_2_.CAN_RxCpltCallback();

    }
}

/******************************************************************************
 * @brief 武器杆夹模块关节目标角度设定
 * 
 * @param Target 目标角度
 * 
 * @note 
 ******************************************************************************/
void Class_HandleClamping::Motor_Handle_Set_Angle(float Target)
{
    switch(Control_Mode_)
    {
        case PID_DIRECTLY_MODE:
        {
            // 直接将目标位置设置给电机的目标位置
            // Motor_Handle_.Set_Target_Angle(Target);
            Motor_Handle_2_.Set_Control_Angle(Target);

            break;
        }

        case PLANNER_MODE:
        {
            // 规划器模式下，先将最终目标位置给规划器，然后调用规划器计算出当前的目标位置，再设置给电机
            Angle_Planner_.Plan(Motor_Handle_.Get_Now_Angle(), Target, HANDLE_PLAN_DURATION_S);
            Motor_Handle_.Set_Target_Angle(Angle_Planner_.GetNextPosition(HANDLE_PLAN_DURATION_S));
            Motor_Handle_.TIM_Calculate_PeriodElapsedCallback();
            break;
        }

        default:
        {
            break;
        }
    }
}



/******************************************************************************
 * @brief 武器杆夹模块定时控制回调(任务入口)
 * 
 * 
 * @note 该函数会直接调用CAN的发送函数，但不会影响到其他CAN_ID的设备，仅会发送该
 *       达妙电机的控制帧
 ******************************************************************************/
void Class_HandleClamping::TIM_1ms_Control_PeriodElapsedCallback()
{
    switch(Control_Mode_)
    {
        case PID_DIRECTLY_MODE:
        {
            //直接将当前设置的目标位置输给PID进行计算
            Motor_Handle_2_.TIM_Send_PeriodElapsedCallback();
            break;
        }

        case PLANNER_MODE:
        {
            // 规划器模式下，先调用规划器计算目标角度，再调用电机的PID计算函数
            float Target_Angle = Angle_Planner_.GetNextPosition(0.001f);
            Motor_Handle_.Set_Target_Angle(Target_Angle);
            Motor_Handle_.TIM_Calculate_PeriodElapsedCallback();
            break;
        }

        default:
        {
            break;
        }
    }

    
}