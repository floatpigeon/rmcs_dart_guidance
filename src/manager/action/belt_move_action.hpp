#pragma once

#include "action.hpp"

#include <cmath>
#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

class BeltMoveAction : public IAction {
public:
    BeltMoveAction(
        std::string name, double& belt_target_velocity, const double& left_belt_velocity,
        const double& right_belt_velocity, double command_velocity, uint64_t timeout_ticks,
        double stall_velocity_threshold = 1.0, uint64_t stall_confirm_ticks = 20,
        uint64_t min_running_ticks = 50)
        : IAction(std::move(name))
        , belt_target_vel_(belt_target_velocity)
        , left_belt_vel_(left_belt_velocity)
        , right_belt_vel_(right_belt_velocity)
        , command_vel_(command_velocity)
        , timeout_ticks_(timeout_ticks)
        , stall_threshold_(stall_velocity_threshold)
        , stall_confirm_ticks_(stall_confirm_ticks)
        , min_running_ticks_(min_running_ticks) {}

    void on_enter() override {
        belt_target_vel_ = command_vel_;
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

    void on_exit() override { belt_target_vel_ = 0.0; }

private:
    double& belt_target_vel_;
    const double& left_belt_vel_;
    const double& right_belt_vel_;

    double command_vel_;
    uint64_t timeout_ticks_;
    double stall_threshold_;
    uint64_t stall_confirm_ticks_;
    uint64_t min_running_ticks_;

    uint64_t stall_counter_{0};
};

} // namespace rmcs_dart_guidance::manager
