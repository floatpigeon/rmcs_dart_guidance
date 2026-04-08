#pragma once

#include <cstdint>

#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

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
    // belt
    const bool& belt_arrive_flag;

    // lift
    const bool& lift_arrive_flag;

    // trigger

    // limit servo
};

struct ManagerOutputContext {
    // belt
    rmcs_msgs::DartMechanismCommand& belt_command;
    double& belt_target_velocity;
    rmcs_msgs::ExitMode& belt_exit_mode;

    // lift
    rmcs_msgs::DartMechanismCommand& lifting_command;
    double& lift_target_velocity;
    rmcs_msgs::ExitMode& lift_exit_mode;

    // trigger
    rmcs_msgs::DartServoCommand& trigger_command;

    // limit servo
    rmcs_msgs::DartServoCommand& limiting_command;
};

struct ManagerSettings {
    // belt
    double belt_down_target_velocity;
    double belt_up_target_velocity;

    // lift
    double lift_target_velocity;

    // trigger

    // limit servo
    uint64_t limiting_fill_ticks;
};

struct ManagerRuntimeState {
    uint32_t fire_count{0};
    ManagerLifecycleState lifecycle_state{ManagerLifecycleState::IDLE};
};

} // namespace rmcs_dart_guidance::manager
