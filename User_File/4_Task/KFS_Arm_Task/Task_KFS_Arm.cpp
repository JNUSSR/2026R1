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
constexpr float ratio_jointh = (120.0 / 30.0); // 电机输出轴转动弧度与关节实际转动弧度的比 TODO
constexpr float ratio_jointv = (2.0 / 0.039); // 电机输出轴转动弧度与关节实际位移的比�?

const Enum_Motor_DJI_ID arm1_jointh_motor_id = Motor_DJI_ID_0x206;
const Enum_Motor_DJI_ID arm1_jointv_motor_id = Motor_DJI_ID_0x202;
const Enum_Motor_DJI_ID arm2_jointh_motor_id = Motor_DJI_ID_0x201;
const Enum_Motor_DJI_ID arm2_jointv_motor_id = Motor_DJI_ID_0x205;//TODO

const float grab_max_torque = 0.5f; // 抓取时的最大允许扭矩，单位牛米

/* ==== 以下数字单位都是米、弧度 ==== */
constexpr float zero_pos_from_gnd = 0.165f; // 关节2在最低位置时，吸盘中心距离地面的高度，单位米// TODO
constexpr float kfs_height = 0.35f; // 作为一个修正项，让吸盘对准的始终是kfs的上方

constexpr float max_drawing_descend_pos = 0.06f; // 抓取阶段关节2的最大下降位置，单位米，超过这个位置就不让它继续下降了
// 几个刻度位置，
constexpr float jointh_0 = 0;
constexpr float jointh_90 = M_PI / 2.0f;
constexpr float jointh_180 = M_PI;

constexpr float jointh_in = 0;
constexpr float jointh_out = 0.2f;

constexpr float jointv_0 = 0.0f - zero_pos_from_gnd + kfs_height;
constexpr float jointv_200 = 0.2f - zero_pos_from_gnd + kfs_height;
constexpr float jointv_400 = 0.4f - zero_pos_from_gnd + kfs_height;
constexpr float jointv_600 = 0.6f - zero_pos_from_gnd + kfs_height;
/*下面所有东西的参考系的原点都在（(jointh转到与两个结构安装那条边垂直，收进车身内那个角度)，(jointv的最低点)）*/
constexpr float jointv_when_jointh_at_acute_angle_min_pos = 0.300f + 0.01f;

//这两个常量对应上电前机械臂的位置
// constexpr float jointh_zero_pos = M_PI / 6.0f; // jointh的零点位置，单位弧度
// constexpr float jointv_zero_pos = jointv_when_jointh_at_acute_angle_min_pos; // jointv的零点位置，单位米
constexpr float jointh_zero_pos = M_PI; // jointh的零点位置，单位弧度
constexpr float jointv_zero_pos = 0; // jointv的零点位置，单位米
//激活后机械臂的初始位置（与上电前位置可能不同，取决于机械臂的安装位置和期望的激活位置）
constexpr float jointh_init_pos = M_PI / 6.0f;
constexpr float jointv_init_pos = 0.42f; 

/* 下面的上场时根据实际情况定 */

const float arm1_jointv_ready_motion_pos = jointv_400; // arm1准备就绪的动作位置，单位米
const float arm2_jointv_ready_motion_pos = jointv_0; // arm2准备就绪的动作位置，单位米

const float jointv_put_kfs_pos = jointv_400; // 放下阶段关节2的目标位置




/* ======== 实例化 ======== */
float arm2_jointh_target;
float arm2_jointv_target;

Class_Motor_DJI_C610 arm1_jointh_motor;
Class_Motor_DJI_C610 arm1_jointv_motor;
Class_Motor_DJI_C610 arm2_jointh_motor;
Class_Motor_DJI_C610 arm2_jointv_motor;
MotorAdapter_C610 arm1_jointh_servo(arm1_jointh_motor);
MotorAdapter_C610 arm1_jointv_servo(arm1_jointv_motor);
MotorAdapter_C610 arm2_jointh_servo(arm2_jointh_motor);
MotorAdapter_C610 arm2_jointv_servo(arm2_jointv_motor);

PlannedJoint arm1_jointh(
    arm1_jointh_servo,
    {
        .min_limit = 0,
        .max_limit = M_PI,
        .zero_pos = 0.0f + jointh_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = -1,
        .ratio = ratio_jointh
    },
    dt
);
PlannedJoint arm1_jointv(
    arm1_jointv_servo,
    {
        .min_limit = 0.0f,
        .max_limit = 0.6f,
        .zero_pos = 0.0f + jointv_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = 1,
        .ratio = ratio_jointv
    },
    dt
);
PlannedJoint arm2_jointh(
    arm2_jointh_servo,
    {
        .min_limit = 0,
        .max_limit = M_PI,
        .zero_pos = 0.0f + jointh_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = 1,
        .ratio = ratio_jointh
    },
    dt
);
PlannedJoint arm2_jointv(
    arm2_jointv_servo,
    {
        .min_limit = 0.0f,
        .max_limit = 0.6f,
        .zero_pos = 0.0f + jointv_zero_pos, // 将机械臂的物理零点位置映射为电机位置0
        .direction = 1,
        .ratio = ratio_jointv
    },
    dt
);

Cylinder arm1_hand(GPIOA, GPIO_PIN_2); 
Cylinder arm2_hand(GPIOA, GPIO_PIN_0);

ArmSequencePlayer<2> player1(arm1_jointh, arm1_jointv);
ArmSequencePlayer<2> player2(arm2_jointh, arm2_jointv);

KFS_Arm arm1_kfs_arm(arm1_jointh, arm1_jointv, arm1_hand, player1);
KFS_Arm arm2_kfs_arm(arm2_jointh, arm2_jointv, arm2_hand, player2);

// const ArmStep<2> GetReady4Motion_Arm1[] = {
//     {{VOID_CMD, {jointv_when_jointh_at_acute_angle_min_pos, 1.5f}},
//     nullptr},
//     {{{arm1_jointh_ready_motion_pos, 2.0f}, VOID_CMD},
//     nullptr},
//     {{VOID_CMD, {arm1_jointv_ready_motion_pos, 3.0f}},
//     nullptr}
// };
// const ArmStep<2> GetReady4Motion_Arm2[] = {
//     {{VOID_CMD, {jointv_when_jointh_at_acute_angle_min_pos, 1.5f}},
//     nullptr},
//     {{{arm2_jointh_ready_motion_pos, 2.0f}, VOID_CMD},
//     nullptr},
//     {{VOID_CMD, {arm2_jointv_ready_motion_pos, 3.0f}},
//     nullptr}
// };

const ArmStep<2> DrawKFS_2_Arm1[] = {
    {{VOID_CMD, {arm1_jointv_ready_motion_pos, 1.5f}},
    nullptr},
    {{{jointh_in, 2.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {jointv_0, 2.0f}},
    nullptr}
};

const ArmStep<2> DrawKFS_2_Arm2[] = {
    {{VOID_CMD, {arm2_jointv_ready_motion_pos, 1.5f}},
    nullptr},
    {{{jointh_in, 2.0f}, VOID_CMD},
    nullptr},
    {{VOID_CMD, {jointv_0, 2.0f}},
    nullptr}
};

const ArmStep<2> PutKFSAndReturn[] = {
    {{VOID_CMD, {jointv_put_kfs_pos, 3.0f}},
    nullptr},
    {{{jointh_out, 2.0f}, VOID_CMD},
    nullptr},
    {{{NAN, 2.0f}, {NAN, 2.0f}}, //等2s气缸缩回
    [](void *context){((KFS_Arm*)context)->releaseClaw();}, nullptr},
    {{{jointh_in, 2.0f}, VOID_CMD},
     nullptr},
    {{VOID_CMD, {jointv_init_pos, 3.0f}},
    nullptr},
};
 





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
void Task_KFS_Arm_StateHandler(KFS_Arm& arm, CtrlButtons btn){//在消费指令时调用，只执行一次
    switch(arm.state_){
        case KFS_Arm::INIT_POS:
            if((btn == Btn_Jointv_down || btn == Btn_Jointv_up) && arm.ALLOW_NEXT_CMD){
                arm.state_ = KFS_Arm::ALLOW_MOTION;
                if(&arm == &arm1_kfs_arm){
                    // arm.playSequence(GetReady4Motion_Arm1, ARRAY_LEN(GetReady4Motion_Arm1));
                    arm.moveVerticallyTo(arm1_jointv_ready_motion_pos);
                    
                }
                else{
                    // arm.playSequence(GetReady4Motion_Arm2, ARRAY_LEN(GetReady4Motion_Arm2));
                    arm.moveVerticallyTo(arm2_jointv_ready_motion_pos);
                }
            }
            break;
        case KFS_Arm::ALLOW_MOTION:
            if((btn == Btn_Hand)){
                arm.state_ = KFS_Arm::DRAWING_0;
                arm.MOTION_EN = false;
                arm.DRAWING_KFS = true;
                arm.reachOutTo(jointh_out);
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
                else if (btn == Btn_Jointv_down || btn == Btn_Jointv_up){ //如果没吸到，重来
                    arm.state_ = KFS_Arm::ALLOW_MOTION;
                    // 注意：不在这里设置 MOTION_EN = true，否则 MotionCmdHandler
                    // 会在同一个消息循环中紧接着触发关节运动指令，与重试序列冲突。
                    // PlayingStateHandler 会在序列完成后自动置位 MOTION_EN。
                    if(&arm == &arm1_kfs_arm){
                        arm.moveVerticallyTo(arm1_jointv_ready_motion_pos, 0.5f);
                    }
                    else{
                        arm.moveVerticallyTo(arm2_jointv_ready_motion_pos, 0.5f);
                    }
                    arm.releaseClaw(); // 关负压
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
        if(btn == Btn_Jointv_up){
            arm.moveVerticallyTo(arm.getJointVCurPos() + 0.01f, 0.5f);
        }
        else if(btn == Btn_Jointv_down){
            arm.moveVerticallyTo(arm.getJointVCurPos() - 0.01f, 0.5f);
        }
    }


}
//持续更新状态，检查动作序列执行情况，完成后更新状态以接受下一条指令，轮询
void Task_KFS_Arm_PlayingStateHandler(KFS_Arm& arm){
    if (arm.isSequencePlaying() || arm.checkJointHIsMoving() || arm.checkJointVIsMoving()) {
        arm.ALLOW_NEXT_CMD = false;

    }
    else{
        arm.ALLOW_NEXT_CMD = true;
    }

    if(arm.state_ == KFS_Arm::ALLOW_MOTION && arm.ALLOW_NEXT_CMD){
        arm.MOTION_EN = true;
    }
    if(arm.state_ == KFS_Arm::DRAWING_0 && arm.ALLOW_NEXT_CMD){
        arm.state_ = KFS_Arm::DRAWING_1; // 进入 DRAWING_1 状态，等待下降动作完成后再检查是否抓到东西
        arm.tightenClaw(); // 开负压
    }
    if(arm.state_ == KFS_Arm::DRAWING_1){
        if (arm.ALLOW_NEXT_CMD){ // 等待下降动作完成
            arm.moveVerticallyTo(arm.getJointVCurPos() - 0.005f, 0.4f); // 继续往下挪一点
        }
        float now_torque = arm.getJointVCurTorque();
        if (now_torque > grab_max_torque){ // 如果当前扭矩超过阈值，认为已经抓到东西了，进入放置阶段
            arm.stopJointV(); // 立即停止当前的下降动作
            arm.state_ = KFS_Arm::DRAWING_2; // 进入 DRAWING_2 状态，等待下降动作完全停止后再切换到 WITH_KFS 状态
            if (&arm == &arm1_kfs_arm){
                arm.playSequence(DrawKFS_2_Arm1, ARRAY_LEN(DrawKFS_2_Arm1));
            }
            else if (&arm == &arm2_kfs_arm){
                arm.playSequence(DrawKFS_2_Arm2, ARRAY_LEN(DrawKFS_2_Arm2));
            }
        }
    }
    if(arm.state_ == KFS_Arm::DRAWING_2 && arm.ALLOW_NEXT_CMD){
        arm.state_ = KFS_Arm::WITH_KFS; // 抓取完成，进入 WITH_KFS 状态，等待放置指令
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
