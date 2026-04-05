#pragma once

#include <cstdint>
#include <string>

#include <eigen3/Eigen/Dense>

#include <rmcs_msgs/dart_limiting_servo_status.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

enum class ManagerLifecycleState : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    ERROR = 2,
};

enum class ManagerTaskSlot : uint8_t {
    PRIMARY = 0,
    AUXILIARY_VISION = 1,
};

inline const char* to_string(ManagerTaskSlot slot) {
    switch (slot) {
    case ManagerTaskSlot::PRIMARY: return "primary";
    case ManagerTaskSlot::AUXILIARY_VISION: return "auxiliary_vision";
    }
    return "unknown";
}

struct ManagerInputContext {
    const double& left_belt_velocity;
    const double& right_belt_velocity;
    const double& left_belt_torque;
    const double& right_belt_torque;

    const Eigen::Vector2d& joystick_left;
    const Eigen::Vector2d& joystick_right;

    const double& lifting_left_vel_fb;
    const double& lifting_right_vel_fb;
};

struct ManagerOutputContext {
    rmcs_msgs::DartMotorStatus& belt_command;
    double& belt_target_velocity;
    double& belt_torque_limit;
    double& belt_hold_torque;
    bool& belt_wait_zero_velocity;

    bool& trigger_lock_enable;

    Eigen::Vector2d& yaw_pitch_control_velocity;
    double& force_control_velocity;

    rmcs_msgs::DartMotorStatus& lifting_command;
    rmcs_msgs::DartServoStatus& limiting_command;
};

struct ManagerSettings {
    double max_transform_rate;
    double manual_force_scale;
    uint64_t limiting_fill_ticks;

    double lifting_stall_threshold;
    uint64_t lifting_stall_confirm_ticks;
    uint64_t lifting_stall_min_run_ticks;
    uint64_t lifting_stall_timeout_ticks;
};

struct ManagerRuntimeState {
    uint32_t fire_count{0};
    ManagerLifecycleState lifecycle_state{ManagerLifecycleState::IDLE};
    std::string last_command;
};

} // namespace rmcs_dart_guidance::manager
