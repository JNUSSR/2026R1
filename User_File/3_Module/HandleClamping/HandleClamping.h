#ifndef HANDLE_CLAMPING_H
#define HANDLE_CLAMPING_H

#include "dvc_motor_dm.h"
#include "QuinticPlanner.h"
/*=======================================
 * 1.CAN和电机初始化参数宏定义
 *=======================================*/
#define HANDLE_MOTOR_CAN_ID             Motor_DM_ID_0x301 //一拖四模式的CAN ID
#define HANDLE_MOTOR_STD_ID             (0x301U)          //一拖四模式反馈帧标准ID
#define HANDLE_MOTOR_MST_ID             (0x01U)           //达妙电机反馈帧ID
#define HANDLE_MOTOR_TX_ID              (0x01U)           //达妙电机控制帧ID
#define HANDLE_MOTOR_CONTROLMODE        Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA //达妙电机控制模式枚举变量
#define HANDLE_MOTOR_ENCODER_OFFSET     (0)               //编码器偏移量（点击初始化参数）
#define HANDLE_MOTOR_GEARBOX_RATE       (19.2f)           //减速比（点击初始化参数）

/*=======================================
 * 2.电机控制相关宏定义
 *=======================================*/
#define HANDLE_MOTOR_CALIBRATE_OMEGA      (0.8f)            //位置模式的最大限幅速度
#define HANDLE_MOTOR_CALIBRATE_TORQUE_MAX (5.0f)
#define HANDLE_MOTOR_CONTROL_OMEGA        (1.0f)            //位置模式的最大限幅速度
#define HANDLE_MOTOR_INIT_INCREANGLE      (0.0f)         //电机上电后复位到垂直角度的角度增量
#define HANDLE_POS_REACH_INCREANGLE       (-2.68f)           //从垂直角度到夹取角度的角度增量
#define HANDLE_POS_ABUTTING_INCREANGLE    (30.144f)         //从垂直角度到对接角度的角度增量
/*========================================
 * 3.一拖四电机控制相关宏定义（暂时用不上）
 *========================================*/
#define HANDLE_PID_OMEGA_KP           (0.0f)
#define HANDLE_PID_OMEGA_KI           (0.0f)
#define HANDLE_PID_OMEGA_KD           (0.0f)
#define HANDLE_PID_ANGLE_KP           (0.0f)
#define HANDLE_PID_ANGLE_KI           (0.0f)
#define HANDLE_PID_ANGLE_KD           (0.0f)

#define HANDLE_PLAN_DT_S            (0.001f)
#define HANDLE_PLAN_DURATION_S      (0.5f)

#define HANDLE_TORQUE_FEEDFORWARD     (0.0f)


// 为适配一拖四模式使用选择位置规划的枚举变量，但现在用不到一拖四电机，直接配置PID_DIRECTLY_MODE就行
typedef enum
{
    PID_DIRECTLY_MODE = 0,
    PLANNER_MODE,

}HandleClamping_ControlMode;

typedef enum
{
    STATE_CALIBRATING = 0,
    STATE_RUNNING,

}HandleClamping_State;


class Class_HandleClamping
{
public:

    explicit Class_HandleClamping(HandleClamping_ControlMode control_mode)
        : Control_Mode_(control_mode), Motor_Handle_(), Angle_Planner_(0.0f) {
    }

    void Init();

    void Calibrate();

    void Start_Calibrate();

    void TIM_1ms_Control_PeriodElapsedCallback();

    void Motor_Handle_Set_Angle(float Target);

    void CAN_RxCallback(uint32_t std_id, uint8_t *data);

    inline float Get_Angle_Reset();
    
    inline float Get_Angle_Reach();

    inline float Get_Angle_Abutting();

    inline float Get_Pos_Max();

    inline float Get_Pos_Min();

    inline HandleClamping_State Get_State(void);

    inline void Set_State(HandleClamping_State state);


    // 兼容一拖四模式，但目前固件还没有适配一拖四电机，所以暂时不使用这个对象
    inline Class_Motor_DM_1_To_4 &Get_Motor_Handle(void);
    // 目前主要使用达妙的位置速度模式，所以使用这个对象
    inline Class_Motor_DM_Normal &Get_Motor_Handle_2(void);

private:

    HandleClamping_ControlMode Control_Mode_ = PID_DIRECTLY_MODE;

    HandleClamping_State State_ = STATE_CALIBRATING;

    Class_Motor_DM_1_To_4 Motor_Handle_;

    Class_Motor_DM_Normal Motor_Handle_2_;

    QuinticPlanner        Angle_Planner_;

    float Pos_Max_ = 0.0f;

    float Pos_Min_ = 0.0f;

    // 预设的几个位置（绝对角度），在初始化时根据零位加上对应增量进行配置
    float Angle_Reset_ = 0.0f;
    //垂直角度

    float Angle_Reach_ = 0.0f;
    //夹取角度

    float Angle_Abutting_ = 0.0f;
    //对接角度

    // 校准启动时的时间戳，校准用
    uint32_t timestamp_start_ = 0;

    // 时间戳，校准用
    uint32_t timestamp_ = 1;

    // 校准步骤，校准用
    uint8_t calibrate_step_ = 0;
    
};

inline float Class_HandleClamping::Get_Angle_Reset()
{
    return Angle_Reset_;
}

inline float Class_HandleClamping::Get_Angle_Reach()
{
    return Angle_Reach_;
}

inline float Class_HandleClamping::Get_Angle_Abutting()
{
    return Angle_Abutting_;
}

inline float Class_HandleClamping::Get_Pos_Max()
{
    return Pos_Max_;
}

inline float Class_HandleClamping::Get_Pos_Min()
{
    return Pos_Min_;
}

inline Class_Motor_DM_1_To_4 &Class_HandleClamping::Get_Motor_Handle(void)
{
    return Motor_Handle_;
}

inline Class_Motor_DM_Normal &Class_HandleClamping::Get_Motor_Handle_2(void)
{
    return Motor_Handle_2_;
}

inline HandleClamping_State Class_HandleClamping::Get_State(void)
{
    return State_;
}

inline void Class_HandleClamping::Set_State(HandleClamping_State state)
{
    State_ = state;
}

#endif // HANDLE_CLAMPING_H