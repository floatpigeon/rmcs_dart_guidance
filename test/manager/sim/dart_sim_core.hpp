#pragma once

#include <rmcs_msgs/dart_mechanism_command.hpp>
#include <rmcs_msgs/dart_servo_command.hpp>

namespace rmcs_dart_guidance::manager::sim {

struct DartSimConfig {
    double initial_belt_position{0.0};
    double initial_lift_position{1.0};
    bool initial_trigger_locked{false};
    rmcs_msgs::DartServoCommand initial_limiting_status{rmcs_msgs::DartServoCommand::LOCK};

    double belt_position_per_velocity_tick{0.001};
    double belt_limit_torque_fallback{1.0};
    double belt_move_torque_ratio{0.25};

    double lift_position_per_tick{0.002};
    double lift_feedback_velocity{2.0};
};

struct DartSimControlInput {
    rmcs_msgs::DartMechanismCommand belt_command{rmcs_msgs::DartMechanismCommand::WAIT};
    double belt_target_velocity{0.0};
    double belt_torque_limit{0.0};
    double belt_hold_torque{0.0};
    bool belt_wait_zero_velocity{false};

    bool trigger_lock_enable{false};

    rmcs_msgs::DartMechanismCommand lifting_command{rmcs_msgs::DartMechanismCommand::WAIT};
    rmcs_msgs::DartServoCommand limiting_command{rmcs_msgs::DartServoCommand::LOCK};
};

struct DartSimPlantState {
    double belt_position{0.0};
    double lift_position{1.0};

    double left_belt_velocity{0.0};
    double right_belt_velocity{0.0};
    double left_belt_torque{0.0};
    double right_belt_torque{0.0};

    double left_lift_velocity{0.0};
    double right_lift_velocity{0.0};

    bool trigger_locked{false};
    rmcs_msgs::DartServoCommand limiting_status{rmcs_msgs::DartServoCommand::LOCK};
};

class DartSimCore {
public:
    explicit DartSimCore(DartSimConfig config = {});

    void reset();
    void step(const DartSimControlInput& input);

    const DartSimPlantState& state() const { return state_; }

private:
    void step_belt(const DartSimControlInput& input);
    void step_lift(const DartSimControlInput& input);

    DartSimConfig config_;
    DartSimPlantState state_;
};

} // namespace rmcs_dart_guidance::manager::sim
