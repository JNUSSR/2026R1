#include "Task_KFS_Arm.h"
#include "arm_sequence_player.h"
#include "planned_joint.h"
#include "stm32h723xx.h"
#include "uart_command.h"
#include "dvc_motor_dji.h"
#include "kfs_arm.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <math.h>
/* ======== Configs ========= */
const float dt  = 0.001f; // 1ms 更新周期
constexpr float ratio_joint1 = (120.0 / 30.0); // 电机输出轴转动弧度与关节实际转动弧度的比�?
constexpr float ratio_joint2 = (2.0 / 0.039); // 电机输出轴转动弧度与关节实际位移的比�?

const Enum_Motor_DJI_ID arm1_joint1_motor_id = Motor_DJI_ID_0x206;
const Enum_Motor_DJI_ID arm1_joint2_motor_id = Motor_DJI_ID_0x202;
const Enum_Motor_DJI_ID arm2_joint1_motor_id = Motor_DJI_ID_0x201;
const Enum_Motor_DJI_ID arm2_joint2_motor_id = Motor_DJI_ID_0x205;

/* ==== 以下数字单位都是米、弧度 ==== */
constexpr float zero_pos_from_gnd = 0.165f; // 关节2在最低位置时，吸盘中心距离地面的高度，单位米
constexpr float kfs_radius = 0.175f; // 作为一个修正项，让吸盘对准的始终是kfs的中心

// 几个刻度位置，
constexpr float joint1_0 = 0;
constexpr float joint1_90 = M_PI / 2.0f;
constexpr float joint1_180 = M_PI;

constexpr float joint2_0 = 0.0f - zero_pos_from_gnd + kfs_radius;
constexpr float joint2_200 = 0.2f - zero_pos_from_gnd + kfs_radius;
constexpr float joint2_400 = 0.4f - zero_pos_from_gnd + kfs_radius;
constexpr float joint2_600 = 0.6f - zero_pos_from_gnd + kfs_radius;
/*下面所有东西的参考系的原点都在（(joint1转到与两个结构安装那条边垂直，收进车身内那个角度)，(joint2的最低点)）*/
constexpr float joint2_when_joint1_at_acute_angle_min_pos = 0.300f + 0.01f;

//这两个常量对应上电前机械臂的位置
// constexpr float joint1_zero_pos = M_PI / 6.0f; // joint1的零点位置，单位弧度
// constexpr float joint2_zero_pos = joint2_when_joint1_at_acute_angle_min_pos; // joint2的零点位置，单位米
constexpr float joint1_zero_pos = M_PI; // joint1的零点位置，单位弧度
constexpr float joint2_zero_pos = 0; // joint2的零点位置，单位米
//激活后机械臂的初始位置（与上电前位置可能不同，取决于机械臂的安装位置和期望的激活位置）
constexpr float joint1_init_pos = M_PI / 6.0f;
constexpr float joint2_init_pos = joint2_when_joint1_at_acute_angle_min_pos; 

/* 下面的上场时根据实际情况定 */

const float joint1_motion_s = joint1_90;
const float joint1_motion_b = joint1_180;
//因为只有俩arm，所以最多只有两个可能要去的高度，就可以直接用上或下表示
const float joint2_motion_up = joint2_400;
const float joint2_motion_down = joint2_0;

const float arm1_joint1_ready_motion_pos = joint1_motion_s; // arm1准备就绪的动作位置，单位米
const float arm2_joint1_ready_motion_pos = joint1_motion_s; // arm2准备就绪的动作位置，单位米
const float arm1_joint2_ready_motion_pos = joint2_motion_down; // arm1准备就绪的动作位置，单位米
const float arm2_joint2_ready_motion_pos = joint2_motion_down; // arm2准备就绪的动作位置，单位米

const float joint2_put_kfs_pos = joint2_400; // 放下阶段关节2的目标位置




/* ======== 实例化 ======== */
KFS_Arm::HandCmd arm1_hand_cmd;
float arm2_joint1_target;
float arm2_joint2_target;
KFS_Arm::HandCmd arm2_hand_cmd;

Class_Motor_DJI_C610 arm1_joint1_motor;
Class_Motor_DJI_C610 arm1_joint2_motor;
Class_Motor_DJI_C610 arm2_joint1_motor;
Class_Motor_DJI_C610 arm2_joint2_motor;
MotorAdapter_C610 arm1_joint1_servo(arm1_joint1_motor);
MotorAdapter_C610 arm1_joint2_servo(arm1_joint2_motor);
MotorAdapter_C610 arm2_joint1_servo(arm2_joint1_motor);
MotorAdapter_C610 arm2_joint2_servo(arm2_joint2_motor);

PlannedJoint arm1_joint1(
    arm1_joint1_servo,
    {
        .min_limit = 0,
        .max_limit = M_PI,
        .zero_pos = 0.0f + joint1_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = -1,
        .ratio = ratio_joint1
    },
    dt
);
PlannedJoint arm1_joint2(
    arm1_joint2_servo,
    {
        .min_limit = 0.0f,
        .max_limit = 0.6f,
        .zero_pos = 0.0f + joint2_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = 1,
        .ratio = ratio_joint2
    },
    dt
);
PlannedJoint arm2_joint1(
    arm2_joint1_servo,
    {
        .min_limit = 0,
        .max_limit = M_PI,
        .zero_pos = 0.0f + joint1_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = 1,
        .ratio = ratio_joint1
    },
    dt
);
PlannedJoint arm2_joint2(
    arm2_joint2_servo,
    {
        .min_limit = 0.0f,
        .max_limit = 0.6f,
        .zero_pos = 0.0f + joint2_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = 1,
        .ratio = ratio_joint2
    },
    dt
);

Cylinder arm1_hand(GPIOA, GPIO_PIN_2); 
Cylinder arm2_hand(GPIOA, GPIO_PIN_0);

ArmSequencePlayer<2> player1(arm1_joint1, arm1_joint2);
ArmSequencePlayer<2> player2(arm2_joint1, arm2_joint2);

KFS_Arm arm1_kfs_arm(arm1_joint1, arm1_joint2, arm1_hand, player1);
KFS_Arm arm2_kfs_arm(arm2_joint1, arm2_joint2, arm2_hand, player2);


const ArmStep<2> InitOnActivate[] = {
    {{{0, 0.0f}, {joint2_init_pos, 2.0f}},
    nullptr},
    {{{joint1_init_pos, 3.0f}, {joint2_init_pos, 0}},
    nullptr}
};

const ArmStep<2> GetReady4Motion_Arm1[] = {
    {{VOID_CMD, {joint2_when_joint1_at_acute_angle_min_pos, 1.5f}},
    nullptr},
    {{{arm1_joint1_ready_motion_pos, 2.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {arm1_joint2_ready_motion_pos, 3.0f}},
    nullptr}
};
const ArmStep<2> GetReady4Motion_Arm2[] = {
    {{VOID_CMD, {joint2_when_joint1_at_acute_angle_min_pos, 1.5f}},
    nullptr},
    {{{arm2_joint1_ready_motion_pos, 2.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {arm2_joint2_ready_motion_pos, 3.0f}},
    nullptr}
};

const ArmStep<2> DrawKFS[] = {
    {{{NAN, 3.0f}, {NAN, 3.0f}}, //等3s气缸伸出
    [](void *context){((KFS_Arm*)context)->reachHandOut();}, nullptr },
    {{VOID_CMD, {joint2_init_pos, 2.0f}},
    nullptr},
    {{{joint1_init_pos, 3.0f}, VOID_CMD},
    nullptr}, 
};

const ArmStep<2> RetryDrawing_Arm1[] = {
    {{VOID_CMD, {NAN, 1.0f}},
    [](void *context){((KFS_Arm*)context)->pullHandBack();}, nullptr},
    {{VOID_CMD, {joint2_when_joint1_at_acute_angle_min_pos, 1.5f}},
    nullptr},
        {{{arm1_joint1_ready_motion_pos, 2.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {arm1_joint2_ready_motion_pos, 2.0f}},
    nullptr}
};

const ArmStep<2> RetryDrawing_Arm2[] = {
    {{VOID_CMD, {NAN, 1.0f}},
    [](void *context){((KFS_Arm*)context)->pullHandBack();}, nullptr},
    {{VOID_CMD, {joint2_when_joint1_at_acute_angle_min_pos, 1.5f}},
    nullptr},
    {{{arm2_joint1_ready_motion_pos, 2.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {arm2_joint2_ready_motion_pos, 2.0f}},
    nullptr}
};

const ArmStep<2> PutKFSAndReturn[] = {
    {{{M_PI, 4.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {joint2_put_kfs_pos, 3.0f}},
    nullptr},
    {{{NAN, 3.0f}, {NAN, 3.0f}}, //等3s气缸缩回
    [](void *context){((KFS_Arm*)context)->pullHandBack();}, nullptr},
    {{VOID_CMD, {joint2_init_pos, 2.0f}},
    nullptr},
    {{{joint1_init_pos, 3.0f}, VOID_CMD},
    nullptr},
};
 





void Task_KFS_Arm_Init(void){
    // Init arm1 motor pair
    arm1_joint1_motor.PID_Omega.Init(
        1.0f
        , 1.0f
        , 0.0f
        , 0.0f
        , 1.0f
        , 2
    );
    arm1_joint1_motor.PID_Angle.Init(
        10.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 15.0f
    );
    arm1_joint1_motor.Init(
        &hfdcan2
        , arm1_joint1_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    arm1_joint2_motor.PID_Omega.Init(
        1.0f
        , 5.0f
        , 0.0f
        , 0.0f
        , 3.0f
        , 3.0f
    );
    arm1_joint2_motor.PID_Angle.Init(
        5.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 10.0f
    );
    arm1_joint2_motor.Init(
        &hfdcan2
        , arm1_joint2_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    // Init arm2 motor pair
    arm2_joint1_motor.PID_Omega.Init(
        1.0f
        , 1.0f
        , 0.0f
        , 0.0f
        , 1.0f
        , 2
    );
    arm2_joint1_motor.PID_Angle.Init(
        10.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 15.0f
    );
    arm2_joint1_motor.Init(
        &hfdcan2
        , arm2_joint1_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    arm2_joint2_motor.PID_Omega.Init(
        1.0f
        , 5.0f
        , 0.0f
        , 0.0f
        , 3.0f
        , 3.0f
    );
    arm2_joint2_motor.PID_Angle.Init(
        5.0f
        , 0.0f
        , 0.0f
        , 0.0f
        , 10.0f
        , 10.0f
    );
    arm2_joint2_motor.Init(
        &hfdcan2
        , arm2_joint2_motor_id
        , Motor_DJI_Control_Method_ANGLE
    );

    // float zero_pos_joint1 = 0.0f; // 根据实际情况设置零点位置
    // float zero_pos_joint2 = 0.0f; // 根据实际情况设置零点位置    
    // zero_pos_joint1 = arm1_joint1_motor.Get_Now_Angle() / ratio_joint1; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm1_joint1.setZeroPos(zero_pos_joint1);
    // zero_pos_joint2 = arm1_joint2_motor.Get_Now_Angle() / ratio_joint2; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm1_joint2.setZeroPos(zero_pos_joint2);

    // zero_pos_joint1 = arm2_joint1_motor.Get_Now_Angle() / ratio_joint1; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm2_joint1.setZeroPos(zero_pos_joint1);
    // zero_pos_joint2 = arm2_joint2_motor.Get_Now_Angle() / ratio_joint2; // 通过读取当前电机位置来设定零点，确保上电后机械臂保持当前位置不动
    // arm2_joint2.setZeroPos(zero_pos_joint2);
}
void Task_KFS_Arm_StateHandler(KFS_Arm& arm, CtrlButtons btn){//在消费指令时调用，只执行一次
    switch(arm.state_){
        case KFS_Arm::ORIGIN:
            if(btn != Btn_ArmSwitch){ // 任何非切换机械臂的指令都可以触发上电初始动作
                arm.state_ = KFS_Arm::INIT_POS;
                arm.playSequence(InitOnActivate, ARRAY_LEN(InitOnActivate));
            }
            break;
        case KFS_Arm::INIT_POS:
            if((btn == Btn_Joint1 || btn == Btn_Joint2) && arm.ALLOW_NEXT_CMD){
                arm.state_ = KFS_Arm::ALLOW_MOTION;
                if(&arm == &arm1_kfs_arm){
                    arm.playSequence(GetReady4Motion_Arm1, ARRAY_LEN(GetReady4Motion_Arm1));
                    arm.joint1_motion_state_ = arm1_joint1_ready_motion_pos == joint1_motion_s ? KFS_Arm::DEGS : KFS_Arm::DEGB;
                    arm.joint2_motion_state_ = arm1_joint2_ready_motion_pos == joint2_motion_up ? KFS_Arm::UP : KFS_Arm::DOWN;
                }
                else{
                    arm.playSequence(GetReady4Motion_Arm2, ARRAY_LEN(GetReady4Motion_Arm2));
                    arm.joint1_motion_state_ = arm2_joint1_ready_motion_pos == joint1_motion_s ? KFS_Arm::DEGS : KFS_Arm::DEGB;
                    arm.joint2_motion_state_ = arm2_joint2_ready_motion_pos == joint2_motion_up ? KFS_Arm::UP : KFS_Arm::DOWN;
                }
            }
            break;
        case KFS_Arm::ALLOW_MOTION:
            if((btn == Btn_Hand)){
                arm.state_ = KFS_Arm::DRAWING;
                arm.MOTION_EN = false;
                arm.DRAWING_KFS = true;
                arm.playSequence(DrawKFS, ARRAY_LEN(DrawKFS));
            }
            break;
        // case KFS_Arm::DRAWING: // 抓取阶段结束后在 Task_KFS_Arm_PlayingStateHandler 中自动进入 WITH_KFS 状态，等待放置指令
        case KFS_Arm::WITH_KFS:
            if(arm.ALLOW_NEXT_CMD){
                if((btn == Btn_Hand)){
                    arm.state_ = KFS_Arm::PUT_KFS;
                    arm.PUTTING_KFS = true;
                    arm.playSequence(PutKFSAndReturn, ARRAY_LEN(PutKFSAndReturn));
                }
                else if (btn == Btn_Joint1 || btn == Btn_Joint2){ //如果没吸到，重来
                    arm.state_ = KFS_Arm::ALLOW_MOTION;
                    // 注意：不在这里设置 MOTION_EN = true，否则 MotionCmdHandler
                    // 会在同一个消息循环中紧接着触发关节运动指令，与重试序列冲突。
                    // PlayingStateHandler 会在序列完成后自动置位 MOTION_EN。
                    if(&arm == &arm1_kfs_arm){
                        arm.playSequence(RetryDrawing_Arm1, ARRAY_LEN(RetryDrawing_Arm1));
                        arm.joint1_motion_state_ = arm1_joint1_ready_motion_pos == joint1_motion_s ? KFS_Arm::DEGS : KFS_Arm::DEGB;
                        arm.joint2_motion_state_ = arm1_joint2_ready_motion_pos == joint2_motion_up ? KFS_Arm::UP : KFS_Arm::DOWN;
                    }
                    else{
                        arm.playSequence(RetryDrawing_Arm2, ARRAY_LEN(RetryDrawing_Arm2));
                        arm.joint1_motion_state_ = arm2_joint1_ready_motion_pos == joint1_motion_s ? KFS_Arm::DEGS : KFS_Arm::DEGB;
                        arm.joint2_motion_state_ = arm2_joint2_ready_motion_pos == joint2_motion_up ? KFS_Arm::UP : KFS_Arm::DOWN;
                    }
                }
            }
            break;
        // case KFS_Arm::PUT_KFS: // 放置阶段结束后在 Task_KFS_Arm_PlayingStateHandler 中自动回到 INIT_POS 状态，等待下一轮指令
        default:
            break;
    }
}

//Motion状态的控制处理函数，在消费指令时调用，根据当前状态和指令执行相应的动作，单次调用
void Task_KFS_Arm_MotionCmdHandler(KFS_Arm &arm, CtrlButtons btn){
    if (arm.MOTION_EN){
        switch (btn) {
            case Btn_Joint1:
                if (arm.joint1_motion_state_ == KFS_Arm::DEGS) {
                    arm.joint1_motion_state_ = KFS_Arm::DEGB;
                    arm.rotateTo(joint1_motion_b);
                }
                else {
                    arm.joint1_motion_state_ = KFS_Arm::DEGS;
                    arm.rotateTo(joint1_motion_s);
                }
                break;
            case Btn_Joint2:
                if (arm.joint2_motion_state_ == KFS_Arm::UP) {
                    arm.joint2_motion_state_ = KFS_Arm::DOWN;
                    arm.moveVerticallyTo(joint2_motion_down);
                }
                else {
                    arm.joint2_motion_state_ = KFS_Arm::UP;
                    arm.moveVerticallyTo(joint2_motion_up);
                }
                break;
        }
    }

}
//持续更新状态，检查动作序列执行情况，完成后更新状态以接受下一条指令，轮询
void Task_KFS_Arm_PlayingStateHandler(KFS_Arm& arm){
    if (arm.isSequencePlaying() || arm.checkJoint1IsMoving() || arm.checkJoint2IsMoving()) {
        arm.ALLOW_NEXT_CMD = false;

    }
    else{
        arm.ALLOW_NEXT_CMD = true;
    }

    if(arm.state_ == KFS_Arm::ALLOW_MOTION && arm.ALLOW_NEXT_CMD){
        arm.MOTION_EN = true;
    }
    if(arm.state_ == KFS_Arm::DRAWING && arm.ALLOW_NEXT_CMD){
        arm.DRAWING_KFS = false;
        arm.state_ = KFS_Arm::WITH_KFS; // 抓取完成后进入 WITH_KFS 状态，等待放置指令
    }
    if(arm.state_ == KFS_Arm::PUT_KFS && arm.ALLOW_NEXT_CMD){
        arm.PUTTING_KFS = false;
        arm.state_ = KFS_Arm::INIT_POS; // 放置完成后回到 INIT_POS 状态，等待下一轮指令
    }
}

extern "C" void Task_KFS_Arm_Impl(){
    // ===== Mavlink 模式：状态机驱动 =====
    // 当前选中的活动机械臂（Btn_ArmSwitch 切换），默认 arm1
    static KFS_Arm* s_active_arm = &arm1_kfs_arm;

    // 消费按钮队列 → 触发状态转移（单次调用）
    uint8_t btn_raw;
    while (osMessageQueueGet(g_ctrl_btn_queue, &btn_raw, NULL, 0U) == osOK)
    {
        auto btn = static_cast<CtrlButtons>(btn_raw);
        if (btn == Btn_ArmSwitch) {
            // 切换活动机械臂：arm1 ↔ arm2
            s_active_arm = (s_active_arm == &arm1_kfs_arm) ? &arm2_kfs_arm : &arm1_kfs_arm;
            //TODO：可以在切换时添加一些视觉或声音反馈，提示用户当前选中的机械臂
        } else {
            // 其他按钮操作当前选中的活动臂
            Task_KFS_Arm_StateHandler(*s_active_arm, btn);
            Task_KFS_Arm_MotionCmdHandler(*s_active_arm, btn);
        }
    }
    // 轮询：检查两臂各自的动作序列完成情况，更新标志位
    Task_KFS_Arm_PlayingStateHandler(arm1_kfs_arm);
    Task_KFS_Arm_PlayingStateHandler(arm2_kfs_arm);


    // ===== 共用：电机更新 =====
    if (arm1_kfs_arm.isSequencePlaying()) {
        arm1_kfs_arm.updateSequence();
    }
    if (arm2_kfs_arm.isSequencePlaying()) {
        arm2_kfs_arm.updateSequence();
    }   
    arm1_joint1.Update();
    arm1_joint2.Update();
    arm2_joint1.Update();
    arm2_joint2.Update();
    arm1_joint1_motor.TIM_Calculate_PeriodElapsedCallback();
    arm1_joint2_motor.TIM_Calculate_PeriodElapsedCallback();
    arm2_joint1_motor.TIM_Calculate_PeriodElapsedCallback();
    arm2_joint2_motor.TIM_Calculate_PeriodElapsedCallback();
}

void KFS_Arms_Motors_AliveChecker(){
    arm1_joint1_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    arm1_joint2_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    arm2_joint1_motor.TIM_100ms_Alive_PeriodElapsedCallback();
    arm2_joint2_motor.TIM_100ms_Alive_PeriodElapsedCallback();
}

void KFS_Arms_Motors_CAN_RxCpltCallback(uint32_t id){
    switch (id) {
        case 0x201:
            arm2_joint1_motor.CAN_RxCpltCallback();
            break;
        case 0x202:
            arm1_joint2_motor.CAN_RxCpltCallback();
            break;
        case 0x205:
            arm2_joint2_motor.CAN_RxCpltCallback();
            break;
        case 0x206:
            arm1_joint1_motor.CAN_RxCpltCallback();
            break;
    }
}
