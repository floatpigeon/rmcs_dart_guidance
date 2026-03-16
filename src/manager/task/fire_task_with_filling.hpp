#pragma once

#include "manager/action/filling_action.hpp"
#include "manager/action/lifting_lk_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// 第2~4发发射任务：
//   1. 扳机释放
//   2. 升降平台上升（等堵转）
//   3. 限位舵机开→等待→关（让下一发飞镖滑到平台）
class FireTask2 : public Task {
public:
    FireTask2(
        bool& trigger_lock_enable,
        rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb,
        double lifting_stall_threshold,
        uint64_t lifting_stall_confirm_ticks,
        uint64_t lifting_stall_min_run_ticks,
        uint64_t lifting_stall_timeout_ticks,
        uint16_t& limiting_servo_angle,
        uint16_t limiting_open_angle,
        uint16_t limiting_close_angle,
        uint64_t limiting_fill_ticks)
        : Task("fire", "发射（2~4发）") {

        then(std::make_shared<TriggerControlAction>(trigger_lock_enable, false, 1000));

        then(std::make_shared<LiftingLkAction>(
            "lifting_up",
            lifting_command, rmcs_msgs::DartSliderStatus::UP,
            lifting_left_vel_fb, lifting_right_vel_fb,
            lifting_stall_threshold, lifting_stall_confirm_ticks,
            lifting_stall_min_run_ticks, lifting_stall_timeout_ticks));

        then(std::make_shared<LimitingFillAction>(
            limiting_servo_angle,
            limiting_open_angle,
            limiting_close_angle,
            limiting_fill_ticks));
    }
};

} // namespace rmcs_dart_guidance::manager
