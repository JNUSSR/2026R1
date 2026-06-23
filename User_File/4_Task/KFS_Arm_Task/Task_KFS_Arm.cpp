#include "Task_KFS_Arm.h"
#include "planned_joint.h"
#include "slope_joint.h"
#include "stm32h723xx.h"
#include "uart_command.h"
#include "dvc_motor_dji.h"
#include "kfs_arm.h"
#include <cmath>
#include <cstdint>

/* ======== Configs ========= */
// arm1：顶吸 arm2；侧吸
const float dt  = 0.001f; // 1ms 更新周期

constexpr float ratio_joint = (2.0 / 0.039);

const Enum_Motor_DJI_ID arm1_jointh_motor_id = Motor_DJI_ID_0x204;
const Enum_Motor_DJI_ID arm1_jointv_motor_id = Motor_DJI_ID_0x203;
const Enum_Motor_DJI_ID arm2_jointh_motor_id = Motor_DJI_ID_0x201;
const Enum_Motor_DJI_ID arm2_jointv_motor_id = Motor_DJI_ID_0x202;
static constexpr int idToHex(Enum_Motor_DJI_ID id){
    return static_cast<int>(id + 0x200);
}

constexpr float jointh_in  = 0;
constexpr float jointh_out = 0.2f;

constexpr float jointh_zero_pos = 0.0f;
constexpr float jointv_zero_pos = 0.0f;

constexpr float arm1_from_gnd = 0.26f;
constexpr float arm2_from_gnd = 0.27f;
constexpr float arm1_to_kfs_mid = 0.35f;
constexpr float arm2_to_kfs_mid = 0.175f;

constexpr float arm2_gear_to_sucker = 0.08f;

constexpr float tune_speed = 0.1f;
constexpr float tick_speed = 0.25f;
constexpr float jointh_speed = 0.2f;

constexpr int16_t encoder_min = 0.0f;
constexpr int16_t encoder_max = 300.0f;


/* ======== 垂直刻度定义 ======== */
/** arm1 垂直刻度位置 */
constexpr float arm1_calib = arm1_from_gnd - arm1_to_kfs_mid;//TODO
constexpr float arm1_vertical_ticks[] = {0.0f, 0.2f - arm1_calib, 0.4f - arm1_calib};
constexpr int    arm1_tick_count = sizeof(arm1_vertical_ticks) / sizeof(arm1_vertical_ticks[0]);
constexpr int    arm1_default_tick_idx = 0;

/** arm2 垂直刻度位置 */
constexpr float arm2_calib = arm2_from_gnd - arm2_to_kfs_mid + arm2_gear_to_sucker;
constexpr float arm2_vertical_ticks[] = {0.2f - arm2_calib, 0.4f - arm2_calib, 0.6f - arm2_calib};
constexpr int    arm2_tick_count = sizeof(arm2_vertical_ticks) / sizeof(arm2_vertical_ticks[0]);
constexpr int    arm2_default_tick_idx = 0;

constexpr float arm1_init_vpos = arm1_vertical_ticks[arm1_default_tick_idx];
constexpr float arm2_init_vpos = arm2_vertical_ticks[arm2_default_tick_idx];

/** @brief 编码器单步微调增量（关节位置单位） */
constexpr float encoder_fine_tune_step = 0.005f;

/* ======== 关节位置限幅 ======== */
constexpr float JOINTV_POS_MIN = 0.0f;
constexpr float JOINTV_POS_MAX = 0.6f;
constexpr float JOINTH1_POS_MIN = 0.0f;
constexpr float JOINTH1_POS_MAX = 0.66f;
constexpr float JOINTH2_POS_MIN = 0.0f;
constexpr float JOINTH2_POS_MAX = 0.56f;

// 编码器值 → 水平关节位置线性映射
static inline float encoderToJoint(float enc, float enc_min, float enc_max, float pos_min, float pos_max) {
    if (enc < enc_min) enc = enc_min;
    if (enc > enc_max) enc = enc_max;
    if (enc != enc) enc = enc_min; // NaN 检测，若为 NaN 则设为最小值
    return (enc - enc_min) / (enc_max - enc_min) * (pos_max - pos_min) + pos_min;
}

static inline float encoderToJointH1Pos(float enc) {
    return encoderToJoint(enc, encoder_min, encoder_max, JOINTH1_POS_MIN, JOINTH1_POS_MAX);
}

static inline float encoderToJointH2Pos(float enc) {
    return encoderToJoint(enc, encoder_min, encoder_max, JOINTH2_POS_MIN, JOINTH2_POS_MAX);
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

SlopeJoint arm1_jointh(
    arm1_jointh_servo,
    { .min_limit = 0, .max_limit = 0.66, .zero_pos = jointh_zero_pos,
      .direction = -1, .ratio = ratio_joint, .max_speed = jointh_speed },
    dt
);
SlopeJoint arm2_jointh(
    arm2_jointh_servo,
    { .min_limit = 0, .max_limit = 0.56, .zero_pos = jointh_zero_pos,
      .direction = 1, .ratio = ratio_joint, .max_speed = jointh_speed },
    dt
);

// joint_v: 改为 SlopeJoint (斜坡跟踪)
SlopeJoint arm1_jointv(
    arm1_jointv_servo,
    { .min_limit = 0.0f, .max_limit = 0.6f, .zero_pos = jointv_zero_pos,
      .direction = 1, .ratio = ratio_joint, .max_speed = tune_speed },
    dt
);
SlopeJoint arm2_jointv(
    arm2_jointv_servo,
    { .min_limit = 0.0f, .max_limit = 0.6f, .zero_pos = jointv_zero_pos,
      .direction = 1, .ratio = ratio_joint, .max_speed = tune_speed },
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
        , 0.5f
        , 0.0f
        , 0.0f
        , 3.0f
        , 3.0f
    );
    arm1_jointh_motor.PID_Angle.Init(
        5.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 4.0f
    );
    arm1_jointh_motor.Init(
        &hfdcan2
        , arm1_jointh_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    arm1_jointv_motor.PID_Omega.Init(
        0.8f
        , 0.5f
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
        , 4.0f
    );
    arm1_jointv_motor.Init(
        &hfdcan2
        , arm1_jointv_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    // Init arm2 motor pair
    arm2_jointh_motor.PID_Omega.Init(
        1.0f
        , 0.5f
        , 0.0f
        , 0.0f
        , 3.0f
        , 3.0f
    );
    arm2_jointh_motor.PID_Angle.Init(
        5.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 4.0f
    );
    arm2_jointh_motor.Init(
        &hfdcan2
        , arm2_jointh_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    arm2_jointv_motor.PID_Omega.Init(
        0.8f
        , 0.5f
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
        , 4.0f
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

    // ===== 设置初始刻度位置 =====
    arm1_kfs_arm.moveVerticallyTo(arm1_init_vpos);
    arm2_kfs_arm.moveVerticallyTo(arm2_init_vpos);

}

extern "C" void Task_KFS_Arm_Impl(){
    // ===== 活动臂与刻度状态 =====
    static bool   active_arm = false;        // false=arm1, true=arm2
    static int    arm1_tick_idx = arm1_default_tick_idx;
    static int    arm2_tick_idx = arm2_default_tick_idx;
    static float  arm1_fine_offset = 0.0f;
    static float  arm2_fine_offset = 0.0f;

    // ===== ACT 上升沿检测 =====
    static bool prev_act4 = false;
    static bool prev_act5 = false;
    static bool prev_act6 = false;

    // ===== 消费按钮队列 =====
    bool tick_moved_arm1 = false;
    bool tick_moved_arm2 = false;

    SuckerCmd_t btn;
    while (osMessageQueueGet(g_sucker_ctrl_queue, &btn, NULL, 0U) == osOK)
    {
        // === 吸盘控制 ===
        if (btn.sucker_a == 1) {
            arm1_kfs_arm.tightenClaw();
        }
        else{
            arm1_kfs_arm.releaseClaw();
        }
        if (btn.sucker_b == 1) {
            arm2_kfs_arm.tightenClaw();
        }
        else{
            arm2_kfs_arm.releaseClaw();
        }

        // === ACT5: 切换活动机械臂（上升沿）===
        if (btn.arm_switch && !prev_act5) {
            active_arm = !active_arm;
        }
        prev_act5 = btn.arm_switch;

        // === ACT4: 往上一个刻度（增大位置值，上升沿）===
        if (btn.up && !prev_act4) {
            if (!active_arm) { // arm1
                if (arm1_tick_idx < arm1_tick_count - 1) {
                    arm1_tick_idx++;
                    arm1_fine_offset = 0.0f;
                    tick_moved_arm1 = true;
                    arm1_kfs_arm.setJointVMaxSpeed(tick_speed);
                }
            } else { // arm2
                if (arm2_tick_idx < arm2_tick_count - 1) {
                    arm2_tick_idx++;
                    arm2_fine_offset = 0.0f;
                    tick_moved_arm2 = true;
                    arm2_kfs_arm.setJointVMaxSpeed(tick_speed);
                }
            }
        }
        prev_act4 = btn.up;

        // === ACT6: 往下一个刻度（减小位置值，上升沿）===
        if (btn.down && !prev_act6) {
            if (!active_arm) { // arm1
                if (arm1_tick_idx > 0) {
                    arm1_tick_idx--;
                    arm1_fine_offset = 0.0f;
                    tick_moved_arm1 = true;
                    arm1_kfs_arm.setJointVMaxSpeed(tick_speed);
                }
            } else { // arm2
                if (arm2_tick_idx > 0) {
                    arm2_tick_idx--;
                    arm2_fine_offset = 0.0f;
                    tick_moved_arm2 = true;
                    arm2_kfs_arm.setJointVMaxSpeed(tick_speed);
                }
            }
        }
        prev_act6 = btn.down;
    }

    static int16_t arm1v_encoder_val, arm2v_encoder_val, arm1h_encoder_val, arm2h_encoder_val;

    // ===== 消费编码器队列 =====
    // 垂直 arm1: 刻度移动完成后才允许编码器微调，微调时使用 tune_speed
    static int16_t prev_arm1v_enc = 0;
    static bool    first_arm1v_enc = true;
    if (osMessageQueueGet(g_encoder0_queue, &arm1v_encoder_val, NULL, 0U) == osOK) {
        // 等待刻度移动完成后再允许微调
        if (!arm1_kfs_arm.isJointVMoving()) {
            if (tick_moved_arm1) {
                // 刻度移动刚完成，切换到微调速度
                tick_moved_arm1 = false;
                arm1_kfs_arm.setJointVMaxSpeed(tune_speed);
            }
            if (first_arm1v_enc) {
                prev_arm1v_enc = arm1v_encoder_val;
                first_arm1v_enc = false;
            } else {
                int16_t delta = arm1v_encoder_val - prev_arm1v_enc;
                prev_arm1v_enc = arm1v_encoder_val;
                if (delta > 0) {
                    arm1_fine_offset += encoder_fine_tune_step;
                } else if (delta < 0) {
                    arm1_fine_offset -= encoder_fine_tune_step;
                }
            }
        }
        float target = Basic_Math_Constrain(arm1_vertical_ticks[arm1_tick_idx] + arm1_fine_offset,
                                   JOINTV_POS_MIN, JOINTV_POS_MAX);
        arm1_kfs_arm.moveVerticallyTo(target);
    }
    // 水平 arm1
    while (osMessageQueueGet(g_encoder1_queue, &arm1h_encoder_val, NULL, 0U) == osOK) {
        arm1_kfs_arm.reachOutTo(encoderToJointH1Pos((float)arm1h_encoder_val));
    }
    // 水平 arm2
    while (osMessageQueueGet(g_encoder2_queue, &arm2h_encoder_val, NULL, 0U) == osOK) {
        arm2_kfs_arm.reachOutTo(encoderToJointH2Pos((float)arm2h_encoder_val));
    }
    // 垂直 arm2: 刻度移动完成后才允许编码器微调，微调时使用 tune_speed
    static int16_t prev_arm2v_enc = 0;
    static bool    first_arm2v_enc = true;
    if (osMessageQueueGet(g_encoder3_queue, &arm2v_encoder_val, NULL, 0U) == osOK) {
        // 等待刻度移动完成后再允许微调
        if (!arm2_kfs_arm.isJointVMoving()) {
            if (tick_moved_arm2) {
                // 刻度移动刚完成，切换到微调速度
                tick_moved_arm2 = false;
                arm2_kfs_arm.setJointVMaxSpeed(tune_speed);
            }
            if (first_arm2v_enc) {
                prev_arm2v_enc = arm2v_encoder_val;
                first_arm2v_enc = false;
            } else {
                int16_t delta = arm2v_encoder_val - prev_arm2v_enc;
                prev_arm2v_enc = arm2v_encoder_val;
                if (delta > 0) {
                    arm2_fine_offset += encoder_fine_tune_step;
                } else if (delta < 0) {
                    arm2_fine_offset -= encoder_fine_tune_step;
                }
            }
        }

        float target = Basic_Math_Constrain(arm2_vertical_ticks[arm2_tick_idx] + arm2_fine_offset,
                           JOINTV_POS_MIN, JOINTV_POS_MAX);                           
        arm2_kfs_arm.moveVerticallyTo(target);
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
        case idToHex(arm2_jointh_motor_id):
            arm2_jointh_motor.CAN_RxCpltCallback();
            break;
        case idToHex(arm1_jointv_motor_id):
            arm1_jointv_motor.CAN_RxCpltCallback();
            break;
        case idToHex(arm2_jointv_motor_id):
            arm2_jointv_motor.CAN_RxCpltCallback();
            break;
        case idToHex(arm1_jointh_motor_id):
            arm1_jointh_motor.CAN_RxCpltCallback();
            break;
    }
}
