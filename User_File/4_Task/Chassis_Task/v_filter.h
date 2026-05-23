//
// Created by chengfeng on 2026/5/23.
//

#ifndef DM02_TEST_V_FILTER_H
#define DM02_TEST_V_FILTER_H
// 速度平滑器状态与配置结构体
typedef struct {
    float v_smooth;
    float theta_smooth;

    // --- 配置参数 ---
    float alpha_v;
    float alpha_theta;
    float snap_tolerance_rad;
    float deadzone_v;         // [新增] 摇杆死区阈值 (滤除回弹震荡)
} VelocitySmoother;

void init_smoother(VelocitySmoother *smoother, float alpha_v, float alpha_theta, float snap_angle_deg, float deadzone_v);
void smooth_velocity(VelocitySmoother *smoother, float vx_in, float vy_in, float *vx_out, float *vy_out);

#endif // DM02_TEST_V_FILTER_H
