#pragma once

#include <cstdint>
#include <string>

#include <eigen3/Eigen/Dense>

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

namespace rmcs_dart_guidance::manager {

class HostManualControlAction : public IAction {
public:
    HostManualControlAction(
        std::string name,
        const int32_t& belt_direction,
        const int32_t& lift_direction,
        const int32_t& yaw_direction,
        const int32_t& pitch_direction,
        const rmcs_msgs::DartServoCommand& trigger_command,
        rmcs_msgs::DartMechanismCommand& belt_command,
        double& belt_target_velocity,
        rmcs_msgs::ExitMode& belt_exit_mode,
        rmcs_msgs::DartMechanismCommand& lift_command,
        double& lift_target_velocity,
        rmcs_msgs::ExitMode& lift_exit_mode,
        rmcs_msgs::DartServoCommand& trigger_output_command,
        int32_t& force_error,
        Eigen::Vector2d& angle_error_vector,
        double angle_max_error,
        double belt_velocity,
        double lift_velocity)
        : IAction(std::move(name))
        , belt_direction_(belt_direction)
        , lift_direction_(lift_direction)
        , yaw_direction_(yaw_direction)
        , pitch_direction_(pitch_direction)
        , trigger_command_(trigger_command)
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_exit_mode_(belt_exit_mode)
        , lift_command_(lift_command)
        , lift_target_velocity_(lift_target_velocity)
        , lift_exit_mode_(lift_exit_mode)
        , trigger_output_command_(trigger_output_command)
        , force_error_(force_error)
        , angle_error_vector_(angle_error_vector)
        , angle_max_error_(angle_max_error)
        , belt_velocity_(belt_velocity)
        , lift_velocity_(lift_velocity) {}

    void on_enter() override { apply_neutral_outputs(); }

    ActionStatus update() override {
        apply_belt_command();
        apply_lift_command();
        apply_angle_command();
        apply_trigger_command();
        force_error_ = 0;
        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        apply_neutral_outputs();
        trigger_output_command_ = rmcs_msgs::DartServoCommand::WAIT;
    }

private:
    void apply_neutral_outputs() {
        belt_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_ = 0.0;
        belt_exit_mode_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;

        lift_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        lift_target_velocity_ = 0.0;
        lift_exit_mode_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;

        force_error_ = 0;
        angle_error_vector_ = Eigen::Vector2d::Zero();
    }

    void apply_belt_command() {
        belt_exit_mode_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        belt_target_velocity_ = 0.0;

        if (belt_direction_ > 0) {
            belt_command_ = rmcs_msgs::DartMechanismCommand::UP;
            belt_target_velocity_ = belt_velocity_;
            return;
        }

        if (belt_direction_ < 0) {
            belt_command_ = rmcs_msgs::DartMechanismCommand::DOWN;
            belt_target_velocity_ = belt_velocity_;
            return;
        }

        belt_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
    }

    void apply_lift_command() {
        lift_exit_mode_ = rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY;
        lift_target_velocity_ = 0.0;

        if (lift_direction_ > 0) {
            lift_command_ = rmcs_msgs::DartMechanismCommand::UP;
            lift_target_velocity_ = lift_velocity_;
            return;
        }

        if (lift_direction_ < 0) {
            lift_command_ = rmcs_msgs::DartMechanismCommand::DOWN;
            lift_target_velocity_ = lift_velocity_;
            return;
        }

        lift_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
    }

    void apply_angle_command() {
        angle_error_vector_ = Eigen::Vector2d{
            static_cast<double>(yaw_direction_) * angle_max_error_,
            static_cast<double>(pitch_direction_) * angle_max_error_};
    }

    void apply_trigger_command() { trigger_output_command_ = trigger_command_; }

    const int32_t& belt_direction_;
    const int32_t& lift_direction_;
    const int32_t& yaw_direction_;
    const int32_t& pitch_direction_;
    const rmcs_msgs::DartServoCommand& trigger_command_;

    rmcs_msgs::DartMechanismCommand& belt_command_;
    double& belt_target_velocity_;
    rmcs_msgs::ExitMode& belt_exit_mode_;

    rmcs_msgs::DartMechanismCommand& lift_command_;
    double& lift_target_velocity_;
    rmcs_msgs::ExitMode& lift_exit_mode_;

    rmcs_msgs::DartServoCommand& trigger_output_command_;
    int32_t& force_error_;
    Eigen::Vector2d& angle_error_vector_;

    double angle_max_error_;
    double belt_velocity_;
    double lift_velocity_;
};

} // namespace rmcs_dart_guidance::manager
