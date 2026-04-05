#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/filling_limit_servo_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// FireAndPreloadTask — 执行一次发射并完成预装填：
//   1. 解锁扳机
//   2. 根据 fire_count 判断是否执行预装填动作
class FireAndPreloadTask : public Task {
public:
    FireAndPreloadTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : Task("fire_preload", "发射并预装填") {

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                           // 动作名称
                output.trigger_lock_enable, false, 1000));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_up",                    // 动作名称
                    output.lifting_command,               // 升降指令（输出）
                    rmcs_msgs::DartMotorStatus::UP,       // 指令状态
                    input.lifting_left_vel_fb,            // 左升降电机速度反馈（输入）
                    input.lifting_right_vel_fb,           // 右升降电机速度反馈（输入）
                    settings.lifting_stall_threshold,     // 堵转速度阈值
                    settings.lifting_stall_confirm_ticks, // 堵转确认帧数
                    settings.lifting_stall_min_run_ticks, // 最短运行帧数
                    settings.lifting_stall_timeout_ticks  // 超时帧数
                    ));

            then(
                std::make_shared<FillingLimitServoAction>(
                    "filling_limit_servo",                // 动作名称
                    output.limiting_command,              // 限位舵机状态（输出）
                    rmcs_msgs::DartServoStatus::FREE,     // 先释放
                    rmcs_msgs::DartServoStatus::LOCK,     // 再锁回
                    settings.limiting_fill_ticks          // 预装填持续帧数
                    ));
        }
    }
};

} // namespace rmcs_dart_guidance::manager
