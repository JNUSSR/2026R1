#pragma once

#include <stdint.h>
#include <cmath>

class QuinticPlanner {
public:
    enum State { UNPLANNING, PLANNING };

    explicit QuinticPlanner(float pos)
        : q0(pos), qf(pos), state(UNPLANNING), current_target(pos) {
    }

    // 规划一段新的轨迹
    void Plan(float start_pos, float target_pos, float time_duration);

    // 获取 dt 时间后的期望位置
    float GetNextPosition(float dt);

    bool IsPlanning() const;

    float GetCurrentTarget() const;

    void ForceSetPosition(float pos);

    void StopPlanning();

private:
    float q0, qf;
    float t, T;
    State state;
    float current_target;
};