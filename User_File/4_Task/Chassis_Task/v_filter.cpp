//
// Created by chengfeng on 2026/5/23.
//

#include "v_filter.h"
#include <stdio.h>
#include <math.h>

// 滤波器状态结构体，需要持久化保存


/**
 * @brief 初始化速度平滑器
 */
void init_smoother(VelocitySmoother *smoother, float alpha_v, float alpha_theta) {
    smoother->v_smooth = 0.0;
    smoother->theta_smooth = 0.0;
    smoother->alpha_v = alpha_v;
    smoother->alpha_theta = alpha_theta;
}

/**
 * @brief 规范化角度到 [-pi, pi] 之间
 */
float normalize_angle(float angle) {
    // 使用 atan2(sin, cos) 是将角度规范化到 [-PI, PI] 最鲁棒的方法
    return atan2(sin(angle), cos(angle));
}

/**
 * @brief 核心平滑函数
 * @param smoother 滤波器状态结构体指针
 * @param vx_in 输入的 vx 阶跃值
 * @param vy_in 输入的 vy 阶跃值
 * @param vx_out 输出的平滑后 vx 指针
 * @param vy_out 输出的平滑后 vy 指针
 */
void smooth_velocity(VelocitySmoother *smoother, float vx_in, float vy_in, float *vx_out, float *vy_out) {

    // 1. 将输入的笛卡尔坐标转换为极坐标 (目标大小和目标角度)
    float v_target = sqrt(vx_in * vx_in + vy_in * vy_in);
    float theta_target = smoother->theta_smooth; // 默认保持当前角度

    // 奇点保护：只有当目标速度大于一个极小值时，才更新目标角度。
    // 防止速度为 0 时 atan2(0,0) 产生随机角度跳变，导致原地乱转。
    if (v_target > 1e-5) {
        theta_target = atan2(vy_in, vx_in);
    }

    // 2. 独立平滑大小 (一阶低通滤波)
    smoother->v_smooth = (1.0 - smoother->alpha_v) * smoother->v_smooth + (smoother->alpha_v * v_target);

    // 3. 独立平滑角度
    // 计算最短角度差，解决 179 度到 -179 度直接相减导致大旋转的问题
    float delta_theta = normalize_angle(theta_target - smoother->theta_smooth);

    // 更新平滑角度，并重新规范化以防止数值溢出
    smoother->theta_smooth = normalize_angle(smoother->theta_smooth + smoother->alpha_theta * delta_theta);

    // 4. 将平滑后的极坐标还原回笛卡尔坐标输出
    *vx_out = smoother->v_smooth * cos(smoother->theta_smooth);
    *vy_out = smoother->v_smooth * sin(smoother->theta_smooth);
}