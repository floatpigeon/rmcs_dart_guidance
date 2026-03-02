#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// BeltMoveAction
//   驱动同步带以指定速度运行，直到检测到堵转（滑块到达机械极限）或超时。
//
//   完成判定逻辑：
//     1. 在最短运行时间（min_running_ticks_）内持续输出指令速度，不做判定。
//     2. 之后每帧检测左右同步带的反馈速度均值，若低于堵转阈值并持续
//        stall_confirm_ticks_ 帧，则判定到位 → SUCCESS。
//     3. 若总帧数超过 timeout_ticks_，认为运动异常 → FAILURE。
//
//   退出时（无论成功/失败/取消）自动将目标速度归零。
// ─────────────────────────────────────────────────────────────────────────────
class BeltMoveAction : public IAction {
public:
    /// @param name                    动作名称（如 "belt_move_down" / "belt_reset"）
    /// @param belt_target_velocity    【输出引用】同步带目标速度
    /// @param left_belt_velocity      【输入引用】左同步带实际速度
    /// @param right_belt_velocity     【输入引用】右同步带实际速度
    /// @param command_velocity        指令速度（正 = 上行/复位，负 = 下行）
    /// @param timeout_ticks           最大允许帧数，超时视为 FAILURE
    /// @param stall_velocity_threshold 堵转判定速度阈值（绝对值）
    /// @param stall_confirm_ticks     连续几帧低于阈值才确认堵转
    /// @param min_running_ticks       最短运行帧数（跳过启动阶段的低速段）
    BeltMoveAction(
        std::string name,
        double& belt_target_velocity,
        const double& left_belt_velocity,
        const double& right_belt_velocity,
        double command_velocity,
        uint64_t timeout_ticks,
        double stall_velocity_threshold = 1.0,
        uint64_t stall_confirm_ticks = 20,
        uint64_t min_running_ticks = 50)
        : IAction(std::move(name))
        , belt_target_vel_(belt_target_velocity)
        , left_belt_vel_(left_belt_velocity)
        , right_belt_vel_(right_belt_velocity)
        , command_vel_(command_velocity)
        , timeout_ticks_(timeout_ticks)
        , stall_threshold_(stall_velocity_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , min_running_ticks_(min_running_ticks) {}

    void on_enter() override {
        belt_target_vel_ = command_vel_;
        stall_counter_ = 0;
    }

    ActionStatus update() override {
        // ── 超时保护 ─────────────────────────────────────────────────────────
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::FAILURE;
        }

        // ── 堵转检测（跳过启动阶段）─────────────────────────────────────────
        if (elapsed_ticks() > min_running_ticks_) {
            double avg_vel =
                (std::abs(left_belt_vel_) + std::abs(right_belt_vel_)) / 2.0;

            if (avg_vel < stall_threshold_) {
                ++stall_counter_;
                if (stall_counter_ >= stall_confirm_ticks_) {
                    return ActionStatus::SUCCESS; // 确认到位
                }
            } else {
                stall_counter_ = 0; // 速度恢复，重新计数
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        belt_target_vel_ = 0.0; // 安全归零
    }

private:
    double& belt_target_vel_;
    const double& left_belt_vel_;
    const double& right_belt_vel_;

    double command_vel_;
    uint64_t timeout_ticks_;
    double stall_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t min_running_ticks_;

    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
