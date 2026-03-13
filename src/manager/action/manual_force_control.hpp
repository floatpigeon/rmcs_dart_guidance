#pragma once

#include "action.hpp"

#include <cstdint>
#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

// 手动力控：右摇杆控制拉力电机速度。
// 当右摇杆持续归零超过 1 秒（1000 tick @ 1kHz）时返回 SUCCESS。
class DartManualForceControlAction : public IAction {
public:
    DartManualForceControlAction(
        double& force_control_velocity,
        const Eigen::Vector2d& joystick_right,
        double max_transform_rate,
        double manual_force_scale = 5.0)
        : IAction("dart_manual_force_control")
        , force_control_velocity_(force_control_velocity)
        , joystick_right_(joystick_right)
        , max_transform_rate_(max_transform_rate)
        , manual_force_scale_(manual_force_scale) {}

    void on_enter() override { idle_ticks_ = 0; }

    ActionStatus update() override {
        force_control_velocity_ =
            joystick_right_.x() * max_transform_rate_ * manual_force_scale_;

        if (joystick_right_.norm() < kIdleThreshold) {
            if (++idle_ticks_ >= kIdleTicksForSuccess)
                return ActionStatus::SUCCESS;
        } else {
            idle_ticks_ = 0;
        }
        return ActionStatus::RUNNING;
    }

private:
    static constexpr double   kIdleThreshold      = 0.02;
    static constexpr uint32_t kIdleTicksForSuccess = 1000;

    double& force_control_velocity_;
    const Eigen::Vector2d& joystick_right_;

    double   max_transform_rate_;
    double   manual_force_scale_;
    uint32_t idle_ticks_{0};
};

} // namespace rmcs_dart_guidance::manager
