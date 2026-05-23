//
// Created by chengfeng on 2026/5/23.
//

#ifndef DM02_TEST_V_FILTER_H
#define DM02_TEST_V_FILTER_H

typedef struct {
    float v_smooth;     // 当前平滑后的速度大小
    float theta_smooth; // 当前平滑后的速度角度 (弧度)
    float alpha_v;      // 速度大小的平滑系数 (0.0 ~ 1.0, 越小越平滑)
    float alpha_theta;  // 速度角度的平滑系数 (0.0 ~ 1.0, 越小越平滑)
} VelocitySmoother;

void init_smoother(VelocitySmoother *smoother, float alpha_v, float alpha_theta);
void smooth_velocity(VelocitySmoother *smoother, float vx_in, float vy_in, float *vx_out, float *vy_out);

#endif // DM02_TEST_V_FILTER_H
