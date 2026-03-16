#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class BeltMoveAction : public IAction {
public:
    BeltMoveAction(
        std::string name, rmcs_msgs::DartSliderStatus& belt_command,
        double& belt_target_velocity, double& belt_torque_limit, double& belt_hold_torque,
        const double& left_belt_velocity, const double& right_belt_velocity,
        rmcs_msgs::DartSliderStatus command, double velocity, double torque_limit, double hold_torque,
        uint64_t timeout_ticks, double stall_velocity_threshold = 1.0,
        uint64_t stall_confirm_ticks = 20, uint64_t min_running_ticks = 50)
        : IAction(std::move(name))
        , belt_command_(belt_command)
        , belt_target_velocity_(belt_target_velocity)
        , belt_torque_limit_(belt_torque_limit)
        , belt_hold_torque_(belt_hold_torque)
        , left_belt_vel_(left_belt_velocity)
        , right_belt_vel_(right_belt_velocity)
        , command_(command)
        , velocity_(velocity)
        , torque_limit_(torque_limit)
        , hold_torque_(hold_torque)
        , timeout_ticks_(timeout_ticks)
        , stall_threshold_(stall_velocity_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , min_running_ticks_(min_running_ticks) {}

    void on_enter() override {
        belt_command_ = command_;
        belt_target_velocity_ = velocity_;
        belt_torque_limit_ = torque_limit_;
        stall_counter_ = 0;
    }

    ActionStatus update() override {
        if (elapsed_ticks() >= timeout_ticks_) {
            return ActionStatus::FAILURE;
        }

        if (elapsed_ticks() > min_running_ticks_) {
            double avg_vel = (std::abs(left_belt_vel_) + std::abs(right_belt_vel_)) / 2.0;

            if (avg_vel < stall_threshold_) {
                ++stall_counter_;
                if (stall_counter_ >= stall_confirm_ticks_) {
                    return ActionStatus::SUCCESS;
                }
            } else {
                stall_counter_ = 0;
            }
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override { 
        belt_command_ = rmcs_msgs::DartSliderStatus::WAIT; 
        belt_hold_torque_ = hold_torque_;
    }

private:
    rmcs_msgs::DartSliderStatus& belt_command_;
    double& belt_target_velocity_;
    double& belt_torque_limit_;
    double& belt_hold_torque_;
    const double& left_belt_vel_;
    const double& right_belt_vel_;

    rmcs_msgs::DartSliderStatus command_;
    double velocity_;
    double torque_limit_;
    double hold_torque_;
    uint64_t timeout_ticks_;
    double stall_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t min_running_ticks_;

    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
