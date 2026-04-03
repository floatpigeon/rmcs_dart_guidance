#pragma once

#include "manager/core/runtime/task.hpp"

#include <cstdint>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>

#include <rmcs_msgs/dart_limiting_servo_status.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

struct ManagerTaskContext {
    rmcs_msgs::DartSliderStatus& belt_command;
    double& belt_target_velocity;
    double& belt_torque_limit;
    double& belt_hold_torque;
    bool& belt_wait_zero_velocity;

    const double& left_belt_velocity;
    const double& right_belt_velocity;
    const double& left_belt_torque;
    const double& right_belt_torque;

    bool& trigger_lock_enable;

    Eigen::Vector2d& yaw_pitch_control_velocity;
    double& force_control_velocity;
    const Eigen::Vector2d& joystick_left;
    const Eigen::Vector2d& joystick_right;

    rmcs_msgs::DartSliderStatus& lifting_command;
    const double& lifting_left_vel_fb;
    const double& lifting_right_vel_fb;

    rmcs_msgs::DartLimitingServoStatus& limiting_command;

    double max_transform_rate;
    double manual_force_scale;
    uint64_t limiting_fill_ticks;

    double lifting_stall_threshold;
    uint64_t lifting_stall_confirm_ticks;
    uint64_t lifting_stall_min_run_ticks;
    uint64_t lifting_stall_timeout_ticks;
};

std::shared_ptr<Task> make_slider_init_task(const ManagerTaskContext& context);
std::shared_ptr<Task> make_task(const std::string& cmd, const ManagerTaskContext& context);

} // namespace rmcs_dart_guidance::manager
