#pragma once

#include "action.hpp"

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

class TriggerLockAction : public IAction {
public:
    TriggerLockAction(double& trigger_target_angle, double lock_angle, uint64_t settle_ticks = 50)
        : IAction("trigger_lock")
        , trigger_target_angle_(trigger_target_angle)
        , lock_angle_(lock_angle)
        , settle_ticks_(settle_ticks) {}

    void on_enter() override { trigger_target_angle_ = lock_angle_; }

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
