#pragma once

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <utility>

namespace rmcs_dart_guidance::manager {

class BeltControlAction : public IAction {
public:
    BeltControlAction(
        std::string name,                                        //
        rmcs_msgs::DartMechanismCommand& belt_command_interface, //
        double& target_velocity_interface,                       //
        rmcs_msgs::ExitMode& exit_mode_interface,                //
        const bool& block_flag_interface,                        //
        rmcs_msgs::DartMechanismCommand command_setting,         //
        double velocity_setting,                                 //
        rmcs_msgs::ExitMode exit_mode_setting,                   //
        uint64_t timeout_ticks_setting                           //
        )
        : IAction(std::move(name))
        , belt_command_output_interface_(belt_command_interface)
        , belt_target_velocity_output_interface_(target_velocity_interface)
        , belt_exit_mode_input_interface_(exit_mode_interface)
        , block_flag_input_interface_(block_flag_interface)
        , command_(command_setting)
        , target_velocity_(velocity_setting)
        , exit_mode_(exit_mode_setting)
        , timeout_ticks_(timeout_ticks_setting) {}

    void on_enter() override {
        belt_command_output_interface_ = command_;
        belt_target_velocity_output_interface_ = target_velocity_;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return fail(ActionFailureReason::TIMEOUT);
        }

        if (block_flag_input_interface_) {
            return ActionStatus::SUCCESS;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        belt_command_output_interface_ = rmcs_msgs::DartMechanismCommand::WAIT;
        belt_target_velocity_output_interface_ = 0.0;
        belt_exit_mode_input_interface_ = exit_mode_;
    }

private:
    rmcs_msgs::DartMechanismCommand& belt_command_output_interface_;
    double& belt_target_velocity_output_interface_;
    rmcs_msgs::ExitMode& belt_exit_mode_input_interface_;
    const bool& block_flag_input_interface_;

    rmcs_msgs::DartMechanismCommand command_;
    double target_velocity_;
    rmcs_msgs::ExitMode exit_mode_;
    uint64_t timeout_ticks_;
};

} // namespace rmcs_dart_guidance::manager
