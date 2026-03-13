#pragma once

#include "action.hpp"

#include <cstdint>
#include <eigen3/Eigen/Dense>

namespace rmcs_dart_guidance::manager {

// 手动角度控制：左摇杆控制 yaw，右摇杆控制 pitch。
// 当两个摇杆持续归零超过 1 秒（1000 tick @ 1kHz）时返回 SUCCESS。
class DartManualAngleControlAction : public IAction {
public:
    DartManualAngleControlAction(
        double& yaw_control_velocity,
        double& pitch_control_velocity,
        const Eigen::Vector2d& joystick_left,
        const Eigen::Vector2d& joystick_right,
        double max_transform_rate)
        : IAction("dart_manual_angle_control")
        , yaw_control_velocity_(yaw_control_velocity)
        , pitch_control_velocity_(pitch_control_velocity)
        , joystick_left_(joystick_left)
        , joystick_right_(joystick_right)
        , max_transform_rate_(max_transform_rate) {}

    void on_enter() override { idle_ticks_ = 0; }

    ActionStatus update() override {
        yaw_control_velocity_   = joystick_left_.y()  * max_transform_rate_;
        pitch_control_velocity_ = joystick_right_.x() * max_transform_rate_;

        if (joystick_left_.norm() < kIdleThreshold && joystick_right_.norm() < kIdleThreshold) {
            if (++idle_ticks_ >= kIdleTicksForSuccess)
                return ActionStatus::SUCCESS;
        } else {
            idle_ticks_ = 0;
        }
        return ActionStatus::RUNNING;
    }

private:
    static constexpr double   kIdleThreshold       = 0.02;
    static constexpr uint32_t kIdleTicksForSuccess  = 1000;

    double& yaw_control_velocity_;
    double& pitch_control_velocity_;

    const Eigen::Vector2d& joystick_left_;
    const Eigen::Vector2d& joystick_right_;

    double   max_transform_rate_;
    uint32_t idle_ticks_{0};
};

} // namespace rmcs_dart_guidance::manager
