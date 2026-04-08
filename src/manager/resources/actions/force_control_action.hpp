#pragma once

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <utility>

namespace rmcs_dart_guidance::manager {

class ForceControlAction : public IAction {
public:
    ForceControlAction(
        std::string name,                                        //
        rmcs_msgs::DartMechanismCommand& belt_command_interface, //
        double& target_velocity_interface,                       //
        double& force_feedback_ch1_interface,                    //
        double& force_feedback_ch2_interface,                    //
        rmcs_msgs::ExitMode& exit_mode_interface,                //
        rmcs_msgs::DartMechanismCommand command_setting,         //
        double allowable_error,                                  //
        double force_setting,                                    //
        rmcs_msgs::ExitMode exit_mode_setting,                   //
        uint64_t timeout_ticks_setting                           //
        )
        : IAction(std::move(name))
        , force_command_output_interface_(belt_command_interface)
        , force_error_output_interface_(target_velocity_interface)
        , belt_exit_mode_output_interface_(exit_mode_interface)
        , force_feedback_ch1_input_interface_(force_feedback_ch1_interface)
        , force_feedback_ch2_input_interface_(force_feedback_ch2_interface)
        , command_(command_setting)
        , allowable_error_(allowable_error)
        , target_force_(force_setting)
        , exit_mode_(exit_mode_setting)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        force_command_output_interface_ = command_;
        force_error_output_interface_ = target_force_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        double avg_force_feedback =
            (force_feedback_ch1_input_interface_ + force_feedback_ch2_input_interface_) / 2;

        if (abs(avg_force_feedback - target_force_) < allowable_error_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        force_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        force_error_output_interface_ = 0.0;
        belt_exit_mode_output_interface_ = exit_mode_;
    }

private:
    rmcs_msgs::DartMechanismCommand& force_command_output_interface_;
    double& force_error_output_interface_;
    rmcs_msgs::ExitMode& belt_exit_mode_output_interface_;
    double& force_feedback_ch1_input_interface_;
    double& force_feedback_ch2_input_interface_;

    rmcs_msgs::DartMechanismCommand command_;
    double allowable_error_;
    double target_force_;
    rmcs_msgs::ExitMode exit_mode_;
    uint64_t timeout_ticks_;
};

} // namespace rmcs_dart_guidance::manager
