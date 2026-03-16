#pragma once

#include "manager/action/filling_limit_servo_action.hpp"
#include "manager/action/filling_lift_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class FireAndPreloadTask : public Task {
public:
    FireAndPreloadTask(
        bool& trigger_lock_enable, rmcs_msgs::DartSliderStatus& lifting_command,
        const double& lifting_left_vel_fb, const double& lifting_right_vel_fb,
        double lifting_stall_threshold, uint64_t lifting_stall_confirm_ticks,
        uint64_t lifting_stall_min_run_ticks, uint64_t lifting_stall_timeout_ticks,
        uint16_t& limiting_servo_angle, uint16_t limiting_release_angle,
        uint16_t limiting_lock_angle, uint64_t preload_fill_ticks)
        : Task("fire", "发射并预装填") {

        then(
            std::make_shared<TriggerControlAction>(
                trigger_lock_enable, // 扳机锁定使能（输出）
                false,               // 解锁（false）
                1000                 // 等待释放完成帧数
                ));

        then(std::make_shared<FillingLiftAction>(
            "filling_lift_up", lifting_command, rmcs_msgs::DartSliderStatus::UP,
            lifting_left_vel_fb, lifting_right_vel_fb, lifting_stall_threshold,
            lifting_stall_confirm_ticks, lifting_stall_min_run_ticks,
            lifting_stall_timeout_ticks));

        then(std::make_shared<FillingLimitServoAction>(
            limiting_servo_angle, limiting_release_angle, limiting_lock_angle, preload_fill_ticks));
    }
};

} // namespace rmcs_dart_guidance::manager
