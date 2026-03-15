#pragma once

#include "manager/action/action_set.hpp"
#include "manager/action/belt_move_action.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/lifting_lk_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// 第2~4发准备任务：
//   1. 传送带下行堵转
//   2. 短暂延迟
//   3. 并行：升降平台下降（等堵转）+ 扳机锁定
//   4. 传送带上行复位
class LaunchPreparationTask2 : public Task {
public:
    LaunchPreparationTask2(
        rmcs_msgs::DartSliderStatus& belt_command,
        const double& left_belt_velocity,
        const double& right_belt_velocity,
        bool& trigger_lock_enable,
        rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb,
        double lifting_stall_threshold,
        uint64_t lifting_stall_confirm_ticks,
        uint64_t lifting_stall_min_run_ticks,
        uint64_t lifting_stall_timeout_ticks)
        : Task("launch_preparation_t2f", "滑块发射准备（2~4发）") {

        then(std::make_shared<BeltMoveAction>(
            "belt_move_down",
            belt_command, left_belt_velocity, right_belt_velocity,
            rmcs_msgs::DartSliderStatus::DOWN,
            5000, 1.0, 100, 50));

        then(std::make_shared<DelayAction>("belt_wait", 50));

        auto parallel =
            std::make_shared<ActionSet>("parallel_prep", ActionSet::Policy::ALL_SUCCESS);
        parallel
            ->also(std::make_shared<LiftingLkAction>(
                "lifting_down",
                lifting_command, rmcs_msgs::DartSliderStatus::DOWN,
                lifting_left_vel_fb, lifting_right_vel_fb,
                lifting_stall_threshold, lifting_stall_confirm_ticks,
                lifting_stall_min_run_ticks, lifting_stall_timeout_ticks))
            .also(std::make_shared<TriggerControlAction>(trigger_lock_enable, true, 1000));
        then(parallel);

        then(std::make_shared<BeltMoveAction>(
            "belt_reset",
            belt_command, left_belt_velocity, right_belt_velocity,
            rmcs_msgs::DartSliderStatus::UP,
            5000, 1.0, 100, 50));
    }
};

} // namespace rmcs_dart_guidance::manager
