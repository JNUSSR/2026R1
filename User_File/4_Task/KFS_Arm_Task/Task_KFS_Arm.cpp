#include "Task_KFS_Arm.h"
#include "planned_joint.h"
#include "slope_joint.h"
#include "stm32h723xx.h"
#include "uart_command.h"
#include "dvc_motor_dji.h"
#include "kfs_arm.h"
#include <cmath>

/* ======== Configs ========= */
const float dt  = 0.001f; // 1ms 更新周期
constexpr float ratio_jointh = (120.0 / 30.0);
constexpr float ratio_jointv = (2.0 / 0.039);

const Enum_Motor_DJI_ID arm1_jointh_motor_id = Motor_DJI_ID_0x206;
const Enum_Motor_DJI_ID arm1_jointv_motor_id = Motor_DJI_ID_0x202;
const Enum_Motor_DJI_ID arm2_jointh_motor_id = Motor_DJI_ID_0x201;
const Enum_Motor_DJI_ID arm2_jointv_motor_id = Motor_DJI_ID_0x205;

constexpr float jointh_in  = 0;
constexpr float jointh_out = 0.2f;

constexpr float jointh_zero_pos = 0;
constexpr float jointv_zero_pos = 0;
constexpr float jointh_init_pos = 0.0f;
constexpr float jointv_init_pos = 0.42f;

/* ======== 编码器映射参数 ======== */
constexpr float ENCODER_MIN  = -200.0f;
constexpr float ENCODER_MAX  =  200.0f;
constexpr float JOINTV_POS_MIN = 0.0f;
constexpr float JOINTV_POS_MAX = 0.6f;

// 编码器值 → joint_v 位置线性映射
static inline float encoderToJointVPos(float enc) {
    if (enc < ENCODER_MIN) enc = ENCODER_MIN;
    if (enc > ENCODER_MAX) enc = ENCODER_MAX;
    return (enc - ENCODER_MIN) / (ENCODER_MAX - ENCODER_MIN)
           * (JOINTV_POS_MAX - JOINTV_POS_MIN) + JOINTV_POS_MIN;
}




/* ======== 实例化 ======== */
Class_Motor_DJI_C610 arm1_jointh_motor;
Class_Motor_DJI_C610 arm1_jointv_motor;
Class_Motor_DJI_C610 arm2_jointh_motor;
Class_Motor_DJI_C610 arm2_jointv_motor;
MotorAdapter_C610 arm1_jointh_servo(arm1_jointh_motor);
MotorAdapter_C610 arm1_jointv_servo(arm1_jointv_motor);
MotorAdapter_C610 arm2_jointh_servo(arm2_jointh_motor);
MotorAdapter_C610 arm2_jointv_servo(arm2_jointv_motor);

// joint_h: 保持 PlannedJoint (QuinticPlanner 阶跃指令)
PlannedJoint arm1_jointh(
    arm1_jointh_servo,
    { .min_limit = 0, .max_limit = M_PI, .zero_pos = jointh_zero_pos,
      .direction = -1, .ratio = ratio_jointh },
    dt
);
PlannedJoint arm2_jointh(
    arm2_jointh_servo,
    { .min_limit = 0, .max_limit = M_PI, .zero_pos = jointh_zero_pos,
      .direction = 1, .ratio = ratio_jointh },
    dt
);

// joint_v: 改为 SlopeJoint (斜坡跟踪)
SlopeJoint arm1_jointv(
    arm1_jointv_servo,
    { .min_limit = 0.0f, .max_limit = 0.6f, .zero_pos = jointv_zero_pos,
      .direction = 1, .ratio = ratio_jointv, .max_speed = 0.15f },
    dt
);
SlopeJoint arm2_jointv(
    arm2_jointv_servo,
    { .min_limit = 0.0f, .max_limit = 0.6f, .zero_pos = jointv_zero_pos,
      .direction = 1, .ratio = ratio_jointv, .max_speed = 0.15f },
    dt
);

Cylinder arm1_hand(GPIOA, GPIO_PIN_2);
Cylinder arm2_hand(GPIOA, GPIO_PIN_0);

KFS_Arm arm1_kfs_arm(arm1_jointh, arm1_jointv, arm1_hand);
KFS_Arm arm2_kfs_arm(arm2_jointh, arm2_jointv, arm2_hand);

// 编码器旋钮值（由外部任务写入，范围 [-200, +200]）
volatile float g_arm1_encoder_val = 0.0f;
volatile float g_arm2_encoder_val = 0.0f;

void Task_KFS_Arm_Init(void){
    // Init arm1 motor pair
    arm1_jointh_motor.PID_Omega.Init(
        1.0f
        , 1.0f
        , 0.0f
        , 0.0f
        , 1.0f
    );
    arm1_jointh_motor.PID_Angle.Init(
        10.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 15.0f
    );
    arm1_jointh_motor.Init(
        &hfdcan2
        , arm1_jointh_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    arm1_jointv_motor.PID_Omega.Init(
        1.0f
        , 5.0f
        , 0.0f
        , 0.0f
        , 3.0f
        , 3.0f
    );
    arm1_jointv_motor.PID_Angle.Init(
        5.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 10.0f
    );
    arm1_jointv_motor.Init(
        &hfdcan2
        , arm1_jointv_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    // Init arm2 motor pair
    arm2_jointh_motor.PID_Omega.Init(
        1.0f
        , 1.0f
        , 0.0f
        , 0.0f
        , 1.0f
        , 2.0f
    );
    arm2_jointh_motor.PID_Angle.Init(
        10.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 15.0f
    );
    arm2_jointh_motor.Init(
        &hfdcan2
        , arm2_jointh_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    arm2_jointv_motor.PID_Omega.Init(
        1.0f
        , 5.0f
        , 0.0f
        , 0.0f
        , 3.0f
        , 3.0f
    );
    arm2_jointv_motor.PID_Angle.Init(
        5.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 10.0f
    );
    arm2_jointv_motor.Init(
        &hfdcan2
        , arm2_jointv_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    // float zero_pos_jointh = 0.0f; // 根据实际情况设置零点位置
    // float zero_pos_jointv = 0.0f; // 根据实际情况设置零点位置
    // zero_pos_jointh = arm1_jointh_motor.Get_Now_Angle() / ratio_jointh; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm1_jointh.setZeroPos(zero_pos_jointh);
    // zero_pos_jointv = arm1_jointv_motor.Get_Now_Angle() / ratio_jointv; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm1_jointv.setZeroPos(zero_pos_jointv);

    // zero_pos_jointh = arm2_jointh_motor.Get_Now_Angle() / ratio_jointh; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm2_jointh.setZeroPos(zero_pos_jointh);
    // zero_pos_jointv = arm2_jointv_motor.Get_Now_Angle() / ratio_jointv; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm2_jointv.setZeroPos(zero_pos_jointv);
}

extern "C" void Task_KFS_Arm_Impl(){
    // ===== 消费按钮队列 =====
    uint8_t btn_raw;
    while (osMessageQueueGet(g_ctrl_btn_queue, &btn_raw, NULL, 0U) == osOK)
    {
        auto btn = static_cast<CtrlButtons>(btn_raw);
        switch (btn) {
            case Btn_Hand1Tighten:
                arm1_kfs_arm.tightenClaw();
                break;
            case Btn_Hand2Tighten:
                arm2_kfs_arm.tightenClaw();
                break;
            case Btn_Hand1Release:
                arm1_kfs_arm.releaseClaw();
                break;
            case Btn_Hand2Release:
                arm2_kfs_arm.releaseClaw();
                break;
            default:
                break;
        }
    }

    // ===== 编码器映射 joint_v =====
    arm1_kfs_arm.moveVerticallyTo(encoderToJointVPos(g_arm1_encoder_val));
    arm2_kfs_arm.moveVerticallyTo(encoderToJointVPos(g_arm2_encoder_val));

    // ===== 电机更新 =====
    arm1_jointh.Update();
    arm1_jointv.Update();
    arm2_jointh.Update();
    arm2_jointv.Update();
    arm1_jointh_motor.TIM_Calculate_PeriodElapsedCallback();
    arm1_jointv_motor.TIM_Calculate_PeriodElapsedCallback();
    arm2_jointh_motor.TIM_Calculate_PeriodElapsedCallback();
    arm2_jointv_motor.TIM_Calculate_PeriodElapsedCallback();
}

void KFS_Arms_Motors_AliveChecker(){
    arm1_jointh_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    arm1_jointv_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    arm2_jointh_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    arm2_jointv_motor.TIM_100ms_Alive_PeriodElapsedCallback();
}

void KFS_Arms_Motors_CAN_RxCpltCallback(uint32_t id){
    switch (id) {
        case 0x201:
            arm2_jointh_motor.CAN_RxCpltCallback();
            break;
        case 0x202:
            arm1_jointv_motor.CAN_RxCpltCallback();
            break;
        case 0x205:
            arm2_jointv_motor.CAN_RxCpltCallback();
            break;
        case 0x206:
            arm1_jointh_motor.CAN_RxCpltCallback();
            break;
    }
}
