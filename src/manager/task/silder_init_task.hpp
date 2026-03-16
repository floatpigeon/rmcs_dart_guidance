#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// SliderInitTask — 上电复位任务，对标 V1 的 DISABLE→RESETTING 逻辑：
//   1. 解锁板机（安全态）
//   2. 传送带 UP 归零（homing_mode=true → 扭矩限制到10%，防机械限位过热）
//   on_enter 置 homing_mode=true，on_exit 清 homing_mode=false
class SliderInitTask : public Task {
public:
    SliderInitTask(
        rmcs_msgs::DartSliderStatus& belt_command,
        double& belt_target_velocity, double& belt_torque_limit, double& belt_hold_torque,
        const double& left_belt_velocity,  const double& right_belt_velocity,
        const double& left_belt_torque,    const double& right_belt_torque,
        bool& trigger_lock_enable,
        bool& belt_homing_mode)
        : Task("slider_init", "传送带上电复位")
        , homing_mode_(belt_homing_mode) {

        then(std::make_shared<TriggerControlAction>(trigger_lock_enable, false, 100));

        then(std::make_shared<BeltMoveAction>(
            "belt_home",
            belt_command,
            belt_target_velocity, belt_torque_limit, belt_hold_torque,
            left_belt_velocity,   right_belt_velocity,
            left_belt_torque,     right_belt_torque,
            rmcs_msgs::DartSliderStatus::UP,
            100.0, 5.0, 1.0,  // velocity, torque_limit, hold_torque
            4000, 0.3, 0.1, 200, 100));
    }

    void on_enter() override {
        homing_mode_ = true;
        Task::on_enter();
    }

    void on_exit() override {
        homing_mode_ = false;
        Task::on_exit();
    }

private:
    bool& homing_mode_;
};

} // namespace rmcs_dart_guidance::manager
