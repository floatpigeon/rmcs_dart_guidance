#include "manager/sim/dart_sim_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace rmcs_dart_guidance::manager::sim {

namespace {

constexpr double kLowerLimit = 0.0;
constexpr double kUpperLimit = 1.0;
constexpr double kPositionEpsilon = 1e-6;

bool is_near(double lhs, double rhs) { return std::abs(lhs - rhs) <= kPositionEpsilon; }

double clamp_position(double position) { return std::clamp(position, kLowerLimit, kUpperLimit); }

} // namespace

DartSimCore::DartSimCore(DartSimConfig config)
    : config_(std::move(config)) {
    reset();
}

void DartSimCore::reset() {
    state_ = DartSimPlantState{};
    state_.belt_position = clamp_position(config_.initial_belt_position);
    state_.lift_position = clamp_position(config_.initial_lift_position);
    state_.trigger_locked = config_.initial_trigger_locked;
    state_.limiting_status = config_.initial_limiting_status;
}

void DartSimCore::step(const DartSimControlInput& input) {
    state_.trigger_locked = input.trigger_lock_enable;
    if (input.limiting_command != rmcs_msgs::DartServoCommand::WAIT) {
        state_.limiting_status = input.limiting_command;
    }

    step_belt(input);
    step_lift(input);
}

void DartSimCore::step_belt(const DartSimControlInput& input) {
    const double torque_limit = std::abs(input.belt_torque_limit);

    if (input.belt_command == rmcs_msgs::DartMechanismCommand::WAIT) {
        state_.left_belt_velocity = 0.0;
        state_.right_belt_velocity = 0.0;

        const double hold_torque =
            input.belt_wait_zero_velocity ? 0.0 : std::abs(input.belt_hold_torque);
        state_.left_belt_torque = hold_torque;
        state_.right_belt_torque = hold_torque;
        return;
    }

    const double direction =
        input.belt_command == rmcs_msgs::DartMechanismCommand::UP ? +1.0 : -1.0;
    const double velocity = std::abs(input.belt_target_velocity);
    const double next_position = clamp_position(
        state_.belt_position + direction * velocity * config_.belt_position_per_velocity_tick);

    const bool reached_upper = direction > 0.0 && is_near(next_position, kUpperLimit);
    const bool reached_lower = direction < 0.0 && is_near(next_position, kLowerLimit);
    const bool hit_limit = reached_upper || reached_lower;

    state_.belt_position = next_position;
    if (hit_limit) {
        const double stall_torque =
            torque_limit > 0.0 ? torque_limit : config_.belt_limit_torque_fallback;
        state_.left_belt_velocity = 0.0;
        state_.right_belt_velocity = 0.0;
        state_.left_belt_torque = stall_torque;
        state_.right_belt_torque = stall_torque;
        return;
    }

    const double move_torque = torque_limit * config_.belt_move_torque_ratio;
    state_.left_belt_velocity = direction * velocity;
    state_.right_belt_velocity = direction * velocity;
    state_.left_belt_torque = move_torque;
    state_.right_belt_torque = move_torque;
}

void DartSimCore::step_lift(const DartSimControlInput& input) {
    if (input.lifting_command == rmcs_msgs::DartMechanismCommand::WAIT) {
        state_.left_lift_velocity = 0.0;
        state_.right_lift_velocity = 0.0;
        return;
    }

    const double direction =
        input.lifting_command == rmcs_msgs::DartMechanismCommand::UP ? +1.0 : -1.0;
    const double next_position =
        clamp_position(state_.lift_position + direction * config_.lift_position_per_tick);

    const bool reached_upper = direction > 0.0 && is_near(next_position, kUpperLimit);
    const bool reached_lower = direction < 0.0 && is_near(next_position, kLowerLimit);
    const bool hit_limit = reached_upper || reached_lower;

    state_.lift_position = next_position;
    if (hit_limit) {
        state_.left_lift_velocity = 0.0;
        state_.right_lift_velocity = 0.0;
        return;
    }

    const double velocity = config_.lift_feedback_velocity;
    if (direction > 0.0) {
        state_.left_lift_velocity = -velocity;
        state_.right_lift_velocity = +velocity;
    } else {
        state_.left_lift_velocity = +velocity;
        state_.right_lift_velocity = -velocity;
    }
}

} // namespace rmcs_dart_guidance::manager::sim
