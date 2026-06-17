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

constexpr float ratio_joint = (2.0 / 0.039);

const Enum_Motor_DJI_ID arm1_jointh_motor_id = Motor_DJI_ID_0x206;
const Enum_Motor_DJI_ID arm1_jointv_motor_id = Motor_DJI_ID_0x202;
const Enum_Motor_DJI_ID arm2_jointh_motor_id = Motor_DJI_ID_0x201;
const Enum_Motor_DJI_ID arm2_jointv_motor_id = Motor_DJI_ID_0x205;

constexpr float jointh_in  = 0;
constexpr float jointh_out = 0.2f;

constexpr float jointh_zero_pos = 0;
constexpr float jointv_zero_pos = 0;
constexpr float jointh_init_pos = 0.0f;
constexpr float jointv_init_pos = 0.0f;

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
// arm1：顶吸 arm2；侧吸
PlannedJoint arm1_jointh(
    arm1_jointh_servo,
    { .min_limit = 0, .max_limit = 0.66, .zero_pos = jointh_zero_pos,
      .direction = -1, .ratio = ratio_joint },
    dt
);
PlannedJoint arm2_jointh(
    arm2_jointh_servo,
    { .min_limit = 0, .max_limit = 0.56, .zero_pos = jointh_zero_pos,
      .direction = 1, .ratio = ratio_joint },
    dt
);

// joint_v: 改为 SlopeJoint (斜坡跟踪)
SlopeJoint arm1_jointv(
    arm1_jointv_servo,
    { .min_limit = 0.0f, .max_limit = 0.6f, .zero_pos = jointv_zero_pos,
      .direction = 1, .ratio = ratio_joint, .max_speed = 0.15f },
    dt
);
SlopeJoint arm2_jointv(
    arm2_jointv_servo,
    { .min_limit = 0.0f, .max_limit = 0.6f, .zero_pos = jointv_zero_pos,
      .direction = 1, .ratio = ratio_joint, .max_speed = 0.15f },
    dt
);

Cylinder arm1_hand(GPIOA, GPIO_PIN_2);
Cylinder arm2_hand(GPIOA, GPIO_PIN_0);

KFS_Arm arm1_kfs_arm(arm1_jointh, arm1_jointv, arm1_hand);
KFS_Arm arm2_kfs_arm(arm2_jointh, arm2_jointv, arm2_hand);

void Task_KFS_Arm_Init(void){
    // Init arm1 motor pair
    arm1_jointh_motor.PID_Omega.Init(
        1.0f
        , 1.0f
        , 0.0f
        , 0.0f
        , 1.0f
        , 2.0f
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
    SuckerCmd_t btn;
    while (osMessageQueueGet(g_sucker_ctrl_queue, &btn, NULL, 0U) == osOK)
    {
        if (btn.sucker_a == 1) {
            static bool arm1_claw_on = false;
            static bool arm1_joint_out = false;
            if (btn.action & Act4) {  // 气缸 toggle
                arm1_claw_on = !arm1_claw_on;
                if (arm1_claw_on) arm1_kfs_arm.tightenClaw();
                else              arm1_kfs_arm.releaseClaw();
            }
            if (btn.action & Act6) {  // 水平伸出/收回 toggle
                arm1_joint_out = !arm1_joint_out;
                if (arm1_joint_out) arm1_kfs_arm.reachOutTo(jointh_out);
                else                arm1_kfs_arm.reachOutTo(jointh_in);
            }
        }
        if (btn.sucker_b == 1) {
            static bool arm2_claw_on = false;
            static bool arm2_joint_out = false;
            if (btn.action & Act4) {  // 气缸 toggle
                arm2_claw_on = !arm2_claw_on;
                if (arm2_claw_on) arm2_kfs_arm.tightenClaw();
                else              arm2_kfs_arm.releaseClaw();
            }
            if (btn.action & Act6) {  // 水平伸出/收回 toggle
                arm2_joint_out = !arm2_joint_out;
                if (arm2_joint_out) arm2_kfs_arm.reachOutTo(jointh_out);
                else                arm2_kfs_arm.reachOutTo(jointh_in);
            }
        }
    }

    float arm1_encoder_val, arm2_encoder_val;
    // ===== 消费编码器队列（按时间顺序处理）=====
    if (osMessageQueueGet(g_encoder0_queue, &arm1_encoder_val, NULL, 0U) == osOK) {
        arm1_kfs_arm.moveVerticallyTo(encoderToJointVPos(arm1_encoder_val));
    }
    if (osMessageQueueGet(g_encoder3_queue, &arm2_encoder_val, NULL, 0U) == osOK) {
        arm2_kfs_arm.moveVerticallyTo(encoderToJointVPos(arm2_encoder_val));
    }

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
