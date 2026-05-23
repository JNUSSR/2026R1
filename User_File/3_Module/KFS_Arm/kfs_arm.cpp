//
// Created by chengfeng on 2026/4/18.
//

#include "drv_can.h"
#include "stm32h7xx_hal_adc_ex.h"
#include "stm32h7xx_hal_gpio.h"
#include "kfs_arm.h"


// // 物理传动参数宏定义 (半径单位：米)
// #define M3508_R 0.01526f
// #define M2006_R 0.01450f
// // 旋转轴的减速比 (电机转动弧度 / 底盘实际转动弧度)
// #define YAW_RATIO 3.0f


/* ============ Joint ============ */


/*=========== KFS_Arm ===========*/
void KFS_Arm::rotateTo(float target){
    if (player.IsPlaying()) return; // 序列播放中时忽略直接指令

    this->joint1.Move(target, 2.0f); // 2秒内移动到目标位置
}

void KFS_Arm::moveVerticallyTo(float target){
    if (player.IsPlaying()) return; // 序列播放中时忽略直接指令

    this->joint2.Move(target, 2.0f); // 2秒内移动到目标位置
}

void KFS_Arm::reachHandOut(){
    this->hand_.charge();
}

void KFS_Arm::pullHandBack(){
    this->hand_.release();
}
#ifdef DEBUG_MODE
bool KFS_Arm::checkJoint1PosLimits(Joint1_Pos target){
    Joint1_Pos joint1_pos = this->RadToJoint1Pos(this->joint1.getCurrentTarget());
    Joint2_Pos joint2_pos = this->MToJoint2Pos(this->joint2.getCurrentTarget()); // 将当前目标位置从弧度转换为角度
    if(joint1_pos == JOINT1_0) {
        if(joint2_pos == JOINT2_0mm || joint2_pos == JOINT2_200mm){
            if(target == JOINT1_90 || target == JOINT1_180){
                return false;
            }
            else{
                return true;
            }
        }
        else{
            return true;
        }
    }
    else if (joint1_pos == JOINT1_90 || joint1_pos == JOINT1_180) {
        return true;
    }
    else{
        return false;
    }
    return false;
}

bool KFS_Arm::checkJoint2PosLimits(Joint2_Pos target){
    // Joint1_Pos joint1_pos = this->RadToJoint1Pos(this->joint1.getCurrentTarget());
    // Joint2_Pos joint2_pos = this->MToJoint2Pos(this->joint2.getCurrentTarget()); // 将当前目标位置从米转换为毫米
    return true; // 目前没有对Joint2的限制要求，直接返回true
}


float KFS_Arm::Joint1PosToRad(Joint1_Pos pos){
    return static_cast<float>(pos) * M_PI / 180.0f; // 将角度转换为弧度
}
float KFS_Arm::Joint2PosToM(Joint2_Pos pos){
    return static_cast<float>(pos) / 1000.0f; // 将毫米转换为米
}

KFS_Arm::Joint1_Pos KFS_Arm::RadToJoint1Pos(float rad){
    return static_cast<Joint1_Pos>(static_cast<int>(rad * 180.0f / M_PI)); // 将弧度转换为角度并转换为枚举
}

KFS_Arm::Joint2_Pos KFS_Arm::MToJoint2Pos(float m){
    return static_cast<Joint2_Pos>(static_cast<int>(m * 1000.0f)); // 将米转换为毫米并转换为枚举
}
#endif