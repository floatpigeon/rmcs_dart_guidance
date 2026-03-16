#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        rmcs_msgs::DartSliderStatus& belt_command,
        const double& left_belt_velocity, const double& right_belt_velocity,
        const double& left_belt_torque,   const double& right_belt_torque,
        bool& trigger_lock_enable)
        : Task("cancel_launch", "取消发射") {

        then(std::make_shared<BeltMoveAction>(
            "belt_move_down",
            belt_command,
            left_belt_velocity, right_belt_velocity,
            left_belt_torque,   right_belt_torque,
            rmcs_msgs::DartSliderStatus::DOWN,
            7500, 1.0, 0.5, 100, 50));

        then(std::make_shared<DelayAction>("belt_wait", 50));

        then(std::make_shared<TriggerControlAction>(trigger_lock_enable, false, 1000));

        then(std::make_shared<BeltMoveAction>(
            "belt_reset",
            belt_command,
            left_belt_velocity, right_belt_velocity,
            left_belt_torque,   right_belt_torque,
            rmcs_msgs::DartSliderStatus::UP,
            7500, 1.0, 0.5, 100, 50));
    }
};

} // namespace rmcs_dart_guidance::manager
