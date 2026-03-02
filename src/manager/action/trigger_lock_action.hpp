#pragma once

#include "action.hpp"

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// TriggerLockAction
//   将扳机舵机旋转到锁定角度，等待舵机物理到位后返回 SUCCESS。
//
//   完成判定逻辑：
//     on_enter() 立即设置目标角度，之后等待 settle_ticks_ 帧让舵机物理到位。
//
//   退出时保持锁定角度不变——扳机应在发射前始终保持锁定状态，
//   直到后续的发射动作显式释放。
// ─────────────────────────────────────────────────────────────────────────────
class TriggerLockAction : public IAction {
public:
    /// @param trigger_target_angle  【输出引用】扳机目标角度
    /// @param lock_angle            锁定角度（弧度）
    /// @param settle_ticks          舵机到位等待帧数
    TriggerLockAction(
        double& trigger_target_angle,
        double lock_angle,
        uint64_t settle_ticks = 50)
        : IAction("trigger_lock")
        , trigger_target_angle_(trigger_target_angle)
        , lock_angle_(lock_angle)
        , settle_ticks_(settle_ticks) {}

    void on_enter() override {
        trigger_target_angle_ = lock_angle_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= settle_ticks_) {
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 保持锁定角度，不重置
        // 扳机在整个发射流程中应保持锁定，直到发射动作显式释放
    }

private:
    double& trigger_target_angle_;
    double lock_angle_;
    uint64_t settle_ticks_;
};

} // namespace rmcs_dart_guidance::manager
