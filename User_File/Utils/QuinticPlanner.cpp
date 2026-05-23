#include "QuinticPlanner.h"

void QuinticPlanner::Plan(float start_pos, float target_pos, float time_duration) {
    if (time_duration == 0) {
        return;
    }
    q0 = start_pos;
    qf = target_pos;
    T = time_duration;
    t = 0.0f;
    state = PLANNING;
    current_target = start_pos;
}

float QuinticPlanner::GetNextPosition(float dt) {
    if (state == PLANNING) {
        t += dt;
        if (t >= T) {
            t = T;
            state = UNPLANNING;
            current_target = qf;
        } else {
            // 归一化时间
            float tau = t / T;
            float tau3 = tau * tau * tau;

            // 5次多项式 S型平滑曲线 (保证v0=0, a0=0, vf=0, af=0)
            // 公式提取公因式优化算力：tau^3 * (10 - 15*tau + 6*tau^2)
            float scale = tau3 * (10.0f - 15.0f * tau + 6.0f * tau * tau);
            current_target = q0 + (qf - q0) * scale;
        }
    }
    return current_target;
}

bool QuinticPlanner::IsPlanning() const {
    return state == PLANNING;
}

float QuinticPlanner::GetCurrentTarget() const {
    return current_target;
}

void QuinticPlanner::ForceSetPosition(float pos) {
    q0 = pos;
    qf = pos;
    current_target = pos;
    state = UNPLANNING; // 明确设定为空闲状态
}