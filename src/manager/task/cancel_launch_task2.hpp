#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/lifting_lk_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// 第2~4发取消任务：
//   1. 传送带下行堵转
//   2. 升降平台上升（等堵转）
//   3. 扳机解锁
//   4. 传送带上行复位
class CancelLaunchTask2 : public Task {
public:
    CancelLaunchTask2(
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
        : Task("cancel_launch", "取消发射（2~4发）") {

        then(std::make_shared<BeltMoveAction>(
            "belt_move_down",
            belt_command, 
            left_belt_velocity, 
            right_belt_velocity,
            rmcs_msgs::DartSliderStatus::DOWN,
            5000, 
            1.0, 
            100, 
            50
            ));

        then(
            std::make_shared<DelayAction>(
                "belt_wait",                       // 动作名称
                100                                 // 等待帧数 (让同步带速度闭环到0)
            ));
            
        then(std::make_shared<LiftingLkAction>(
            "lifting_up",
            lifting_command, rmcs_msgs::DartSliderStatus::UP,
            lifting_left_vel_fb, lifting_right_vel_fb,
            lifting_stall_threshold, lifting_stall_confirm_ticks,
            lifting_stall_min_run_ticks, lifting_stall_timeout_ticks));

        then(std::make_shared<TriggerControlAction>(trigger_lock_enable, false, 1000));

        then(std::make_shared<BeltMoveAction>(
            "belt_reset",
            belt_command, left_belt_velocity, right_belt_velocity,
            rmcs_msgs::DartSliderStatus::UP,
            5000, 1.0, 100, 50));
    }
};

} // namespace rmcs_dart_guidance::manager
