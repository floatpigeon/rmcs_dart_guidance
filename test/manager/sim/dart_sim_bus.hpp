#pragma once

#include <cstdint>

#include <rclcpp/qos.hpp>
#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_servo_command.hpp>

namespace rmcs_dart_guidance::manager::sim::bus {

inline constexpr char kCmdBeltCommand[] = "/dart/sim_bus/cmd/belt_command";
inline constexpr char kCmdBeltTargetVelocity[] = "/dart/sim_bus/cmd/belt_target_velocity";
inline constexpr char kCmdBeltTorqueLimit[] = "/dart/sim_bus/cmd/belt_torque_limit";
inline constexpr char kCmdBeltHoldTorque[] = "/dart/sim_bus/cmd/belt_hold_torque";
inline constexpr char kCmdBeltWaitZeroVelocity[] = "/dart/sim_bus/cmd/belt_wait_zero_velocity";
inline constexpr char kCmdTriggerLockEnable[] = "/dart/sim_bus/cmd/trigger_lock_enable";
inline constexpr char kCmdLiftingCommand[] = "/dart/sim_bus/cmd/lifting_command";
inline constexpr char kCmdLimitingCommand[] = "/dart/sim_bus/cmd/limiting_command";
inline constexpr char kTick[] = "/dart/sim_bus/tick";

inline constexpr char kFbLeftBeltVelocity[] = "/dart/sim_bus/fb/drive_belt_left_velocity";
inline constexpr char kFbRightBeltVelocity[] = "/dart/sim_bus/fb/drive_belt_right_velocity";
inline constexpr char kFbLeftBeltTorque[] = "/dart/sim_bus/fb/drive_belt_left_torque";
inline constexpr char kFbRightBeltTorque[] = "/dart/sim_bus/fb/drive_belt_right_torque";
inline constexpr char kFbLeftLiftVelocity[] = "/dart/sim_bus/fb/lifting_left_velocity";
inline constexpr char kFbRightLiftVelocity[] = "/dart/sim_bus/fb/lifting_right_velocity";
inline constexpr char kFbBeltPosition[] = "/dart/sim_bus/fb/belt_position";
inline constexpr char kFbLiftPosition[] = "/dart/sim_bus/fb/lift_position";
inline constexpr char kFbTriggerLocked[] = "/dart/sim_bus/fb/trigger_locked";
inline constexpr char kFbLimitingStatus[] = "/dart/sim_bus/fb/limiting_status";

inline rclcpp::QoS reliable_qos() { return rclcpp::QoS(1).reliable(); }

inline uint8_t encode(rmcs_msgs::DartMechanismCommand command) {
    return static_cast<uint8_t>(command);
}

inline uint8_t encode(rmcs_msgs::DartServoCommand command) { return static_cast<uint8_t>(command); }

inline rmcs_msgs::DartMechanismCommand decode_motor_command(uint8_t value) {
    switch (value) {
    case 0: return rmcs_msgs::DartMechanismCommand::UP;
    case 1: return rmcs_msgs::DartMechanismCommand::DOWN;
    case 2: return rmcs_msgs::DartMechanismCommand::WAIT;
    default: return rmcs_msgs::DartMechanismCommand::WAIT;
    }
}

inline rmcs_msgs::DartServoCommand decode_servo_command(uint8_t value) {
    switch (value) {
    case 0: return rmcs_msgs::DartServoCommand::FREE;
    case 1: return rmcs_msgs::DartServoCommand::LOCK;
    case 2: return rmcs_msgs::DartServoCommand::WAIT;
    default: return rmcs_msgs::DartServoCommand::LOCK;
    }
}

} // namespace rmcs_dart_guidance::manager::sim::bus
