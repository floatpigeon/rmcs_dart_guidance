#pragma once

#include "manager/core/runtime/action.hpp"

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

class TriggerControlAction : public IAction {
public:
    TriggerControlAction(
        bool& trigger_lock_enable, bool lock_enable, uint64_t settle_ticks = 50)
        : IAction("trigger_lock")
        , trigger_lock_enable_(trigger_lock_enable)
        , lock_enable_(lock_enable)
        , settle_ticks_(settle_ticks) {}

    void on_enter() override { trigger_lock_enable_ = lock_enable_; }

    ActionStatus update() override {
        if (elapsed_ticks() >= settle_ticks_) {
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 保持锁定或释放状态，不重置
        // 扳机在整个发射流程中应保持状态，直到显式改变
    }

private:
    bool& trigger_lock_enable_;
    bool lock_enable_;
    uint64_t settle_ticks_;
};

} // namespace rmcs_dart_guidance::manager
