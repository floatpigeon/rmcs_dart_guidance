#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include "Eigen/src/Core/Matrix.h"
#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"
#include "rmcs_msgs/switch.hpp"

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
    const double& belt_left_angle;
    const double& belt_left_velocity;
    const double& belt_left_torque;
    const double& belt_right_angle;
    const double& belt_right_velocity;
    const double& belt_right_torque;

    // lift
    const double& lift_left_velocity;
    const double& lift_left_torque;
    const double& lift_right_velocity;
    const double& lift_right_torque;

    // trigger

    // limit servo

    // yaw pitch force
    const int32_t& force_sensor_ch1;
    const int32_t& force_sensor_ch2;

    // remote control
    const rmcs_msgs::Switch& remote_left_switch;
    const rmcs_msgs::Switch& remote_right_switch;
    const rmcs_msgs::Switch& remote_rotary_knob_switch;
    const Eigen::Vector2d& remote_left_joystick;
    const Eigen::Vector2d& remote_right_joystick;

    // host manual control
    const int32_t& host_manual_belt_direction;
    const int32_t& host_manual_lift_direction;
    const int32_t& host_manual_yaw_direction;
    const int32_t& host_manual_pitch_direction;
    const rmcs_msgs::DartServoCommand& host_manual_trigger_command;
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

    // yaw pitch force
    int32_t& force_error;
    Eigen::Vector2d& angle_error_vector;
};

struct ManagerSettings {
    // belt
    double belt_down_setting_velocity;
    double belt_up_setting_velocity;
    double belt_stall_velocity_threshold;
    double belt_stall_torque_threshold;
    uint64_t belt_stall_confirm_ticks;
    double belt_manual_setting_velocity;

    // lift
    double lift_target_velocity;
    double lift_stall_velocity_threshold;
    double lift_stall_torque_threshold;
    uint64_t lift_stall_confirm_ticks;

    // trigger

    // limit servo
    uint64_t limiting_fill_ticks;

    // yaw pitch force
    int32_t force_setpoint;
    int32_t force_allowable_error;
    double manual_angle_max_error;
    int32_t manual_force_max_error;
};

struct ManagerRuntimeState {
    uint32_t fire_count{0};
    ManagerLifecycleState lifecycle_state{ManagerLifecycleState::IDLE};
    bool host_manual_control_active{false};
};

struct ManagerQueuedTaskInfo {
    std::string task_name;
    std::string display_name;

    friend bool operator==(const ManagerQueuedTaskInfo&, const ManagerQueuedTaskInfo&) = default;
};

struct ManagerLastErrorInfo {
    std::string task_name;
    std::string action_name;
    ActionFailureReason reason{ActionFailureReason::NONE};
    int64_t timestamp_ms{0};

    friend bool operator==(const ManagerLastErrorInfo&, const ManagerLastErrorInfo&) = default;
};

} // namespace rmcs_dart_guidance::manager
