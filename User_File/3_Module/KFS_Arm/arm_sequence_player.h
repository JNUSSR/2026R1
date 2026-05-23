#pragma once

#define ARRAY_LEN(arr) (sizeof(arr) / sizeof(arr[0]))
// 单个关节的动作指令
struct JointCmd {
    float target;
    float duration;
};

// 空指令（跳过该关节）
const JointCmd VOID_CMD = { 0.0f, 0.0f };

// 带有模板参数 N 的动作步骤，N 代表关节数
template <size_t N>
struct ArmStep {
    JointCmd cmds[N];
    void (*custom_action)(void *context); // 可选的自定义动作函数指针
    void *context = nullptr; // 可选的上下文指针，供 custom_action 使用
};

// 带有模板参数 N 的播放器类
template <size_t N>
class ArmSequencePlayer {
public:
    // 变长参数模板构造函数，可以直接接收任意数量的 PlannedJoint&
    template <typename... Joints>
    ArmSequencePlayer(Joints&... joints)
        : state_(IDLE), current_sequence_(nullptr), joints_{ &joints... }
    {
        // 编译期断言：确保传入的引用数量等于实例化的模板参数 N
        static_assert(sizeof...(Joints) == N, "传入的关节数量与实例化模板的 N 不匹配!");
    }


    void setContext(void* ctx) { context_ = ctx; }

    void Play(const ArmStep<N>* sequence, uint16_t step_count) {
        current_sequence_ = sequence;
        total_steps_ = step_count;
        current_step_index_ = 0;
        state_ = RUNNING_STEP;
    }

    void Stop() {
        state_ = IDLE;
    }

    bool IsPlaying() const {
        return state_ != IDLE;
    }

    void Update() {
        if (state_ == IDLE) return;

        if (state_ == RUNNING_STEP) {
            const ArmStep<N>& step = current_sequence_[current_step_index_];

            // 优先执行自定义动作
            if (step.custom_action != nullptr) {
                void* ctx = step.context ? step.context : context_;
                step.custom_action(ctx);
            }

            // 遍历并下发 N 个关节的指令
            for (size_t i = 0; i < N; ++i) {
                const auto& cmd = step.cmds[i];
                // duration == 0 → 跳过该关节
                if (cmd.duration == 0.0f) continue;
                // target == NAN → "空等"，设目标为当前位置
                if (std::isnan(cmd.target)) {
                    joints_[i]->Move(joints_[i]->getCurrentTarget(), cmd.duration);
                } else {
                    joints_[i]->Move(cmd.target, cmd.duration);
                }
            }

            state_ = WAITING_STEP;
        }
        else if (state_ == WAITING_STEP) {
            bool all_finished = true;
            // 检查所有的轴是否都已经运动完毕
            for (size_t i = 0; i < N; ++i) {
                if (joints_[i]->IsMoving()) {
                    all_finished = false;
                    break;
                }
            }

            if (all_finished) {
                current_step_index_++;
                if (current_step_index_ >= total_steps_) {
                    state_ = IDLE; // 完成
                } else {
                    state_ = RUNNING_STEP; // 下一步
                }
            }
        }
    }

private:
    enum State { IDLE, RUNNING_STEP, WAITING_STEP };
    State state_;

    const ArmStep<N>* current_sequence_;
    uint16_t total_steps_;
    uint16_t current_step_index_;
    void* context_ = nullptr;

    // 内部存放各个关节引用的指针数组
    PlannedJoint* joints_[N];
};