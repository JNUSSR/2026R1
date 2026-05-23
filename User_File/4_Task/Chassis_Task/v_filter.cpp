#include "v_filter.h"


#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



/**
 * @brief 初始化速度平滑器
 * @param deadzone_v 摇杆死区阈值。若你的速度输入范围是 0~1.0，通常设为 0.05 ~ 0.15。
 */
void init_smoother(VelocitySmoother *smoother, float alpha_v, float alpha_theta, float snap_angle_deg, float deadzone_v) {
    smoother->v_smooth = 0.0f;
    smoother->theta_smooth = 0.0f;
    smoother->alpha_v = alpha_v;
    smoother->alpha_theta = alpha_theta;
    smoother->snap_tolerance_rad = snap_angle_deg * ((float)M_PI / 180.0f);
    smoother->deadzone_v = deadzone_v; // 保存死区阈值
}

float normalize_angle(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

void smooth_velocity(VelocitySmoother *smoother, float vx_in, float vy_in, float *vx_out, float *vy_out) {

    // 1. 获取目标大小
    float v_target = sqrtf(vx_in * vx_in + vy_in * vy_in);
    float theta_target = smoother->theta_smooth; // 默认锁定当前角度

    // ==========================================
    // [核心修正]：物理死区保护
    // ==========================================
    if (v_target > smoother->deadzone_v) {
        // 摇杆推力大于死区，正常更新目标角度
        theta_target = atan2f(vy_in, vx_in);

        // 正交方向吸附
        if (smoother->snap_tolerance_rad > 0.0f) {
            float half_pi = (float)M_PI / 2.0f;
            float closest_cardinal = roundf(theta_target / half_pi) * half_pi;
            float diff = fabsf(normalize_angle(theta_target - closest_cardinal));
            if (diff <= smoother->snap_tolerance_rad) {
                theta_target = normalize_angle(closest_cardinal);
            }
        }
    } else {
        // 摇杆处于死区内 (包含回弹震荡状态)
        // 强制目标速度为 0，彻底消除角色的微小滑动
        v_target = 0.0f;

        // 注意：这里没有更新 theta_target。
        // 它会保持与 smoother->theta_smooth 一致，
        // 使得后续的 delta_theta = 0，完美避开 90 度突变逻辑。
    }

    // 2. 计算最短角度差
    float delta_theta = normalize_angle(theta_target - smoother->theta_smooth);

    // 3. 处理角度：大角度突变逻辑
    float half_pi = (float)M_PI / 2.0f;
    if (fabsf(delta_theta) > half_pi) {
        smoother->v_smooth = smoother->v_smooth * cosf(delta_theta);
        smoother->theta_smooth = theta_target;
    } else {
        smoother->theta_smooth = normalize_angle(smoother->theta_smooth + smoother->alpha_theta * delta_theta);
    }

    // 4. 速度大小进行平滑滤波
    smoother->v_smooth = (1.0f - smoother->alpha_v) * smoother->v_smooth + (smoother->alpha_v * v_target);

    // 5. 还原为笛卡尔坐标
    *vx_out = smoother->v_smooth * cosf(smoother->theta_smooth);
    *vy_out = smoother->v_smooth * sinf(smoother->theta_smooth);
}