#pragma once

#include "action.hpp"

#include <cstdint>

namespace rmcs_dart_guidance::manager {

// 填装限位舵机动作：先放开限位，等待 fill_ticks，再锁回。
class LimitingFillAction : public IAction {
public:
    LimitingFillAction(
        uint16_t& limiting_control_angle,
        uint16_t trigger_angle,
        uint16_t lock_angle,
        uint64_t fill_ticks)
        : IAction("limiting_fill")
        , limiting_control_angle_(limiting_control_angle)
        , trigger_angle_(trigger_angle)
        , lock_angle_(lock_angle)
        , fill_ticks_(fill_ticks) {}

    void on_enter() override {
        limiting_control_angle_ = trigger_angle_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= fill_ticks_) {
            limiting_control_angle_ = lock_angle_;
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        limiting_control_angle_ = lock_angle_;
    }

private:
    uint16_t& limiting_control_angle_;

    uint16_t trigger_angle_;
    uint16_t lock_angle_;

    uint64_t fill_ticks_;
};

} // namespace rmcs_dart_guidance::manager