#pragma once

#include "manager/action/filling_action.hpp"
#include "manager/action/lifting_lk_action.hpp"
#include "manager/task/task.hpp"
#include "manager/action/delay_action.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// FillingTestTask — 单独测试填装序列：升降UP堵转 → 限位舵机开→等待→关
// 无传送带/板机控制，专用于调试验证。
class FillingTestTask : public Task {
public:
    FillingTestTask(
        rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb,
        const double& lifting_right_vel_fb,
        double lifting_stall_threshold,
        uint64_t lifting_stall_confirm_ticks,
        uint64_t lifting_stall_min_run_ticks,
        uint64_t lifting_stall_timeout_ticks,
        uint16_t& limiting_servo_angle,
        uint16_t open_angle,
        uint16_t close_angle,
        uint64_t fill_ticks)
        : Task("fill_test", "填装测试（升降+限位舵机）") {

        then(std::make_shared<LiftingLkAction>(
            "lifting_down",
            lifting_command, rmcs_msgs::DartSliderStatus::DOWN,
            lifting_left_vel_fb, lifting_right_vel_fb,
            lifting_stall_threshold, lifting_stall_confirm_ticks,
            lifting_stall_min_run_ticks, lifting_stall_timeout_ticks));

        then(std::make_shared<DelayAction>("wait", 50));

        then(std::make_shared<LiftingLkAction>(
            "lifting_up",
            lifting_command, rmcs_msgs::DartSliderStatus::UP,
            lifting_left_vel_fb, lifting_right_vel_fb,
            lifting_stall_threshold, lifting_stall_confirm_ticks,
            lifting_stall_min_run_ticks, lifting_stall_timeout_ticks));

        then(std::make_shared<LimitingFillAction>(
            limiting_servo_angle,
            open_angle,
            close_angle,
            fill_ticks));
    }
};

} // namespace rmcs_dart_guidance::manager
