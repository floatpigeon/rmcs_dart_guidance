#pragma once

#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_servo_command.hpp>

namespace rmcs_dart_guidance::manager {

enum class ManagerLifecycleState : uint8_t {
    IDLE = 0,
    RUNNING = 1,
    ERROR = 2,
};

inline const char* to_string(ManagerLifecycleState state) {
    switch (state) {
    case ManagerLifecycleState::IDLE: return "IDLE";
    case ManagerLifecycleState::RUNNING: return "RUNNING";
    case ManagerLifecycleState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

struct ManagerInputContext {
    const double& left_belt_velocity;
    const double& right_belt_velocity;
    const double& left_belt_torque;
    const double& right_belt_torque;

    const double& lifting_left_vel_fb;
    const double& lifting_right_vel_fb;
};

struct ManagerOutputContext {
    rmcs_msgs::DartMechanismCommand& belt_command;
    double& belt_target_velocity;
    double& belt_torque_limit;
    double& belt_hold_torque;
    bool& belt_wait_zero_velocity;

    bool& trigger_lock_enable;

    rmcs_msgs::DartMechanismCommand& lifting_command;
    rmcs_msgs::DartServoCommand& limiting_command;
};

struct ManagerSettings {
    uint64_t limiting_fill_ticks;

    double lifting_stall_threshold;
    uint64_t lifting_stall_confirm_ticks;
    uint64_t lifting_stall_min_run_ticks;
    uint64_t lifting_stall_timeout_ticks;
};

struct ManagerRuntimeState {
    uint32_t fire_count{0};
    ManagerLifecycleState lifecycle_state{ManagerLifecycleState::IDLE};
};

} // namespace rmcs_dart_guidance::manager
