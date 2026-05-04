#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/delay_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/filling_limit_servo_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

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
        then(std::make_shared<DelayAction>("fire_delay", 500));
        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                              // 动作名称
                output.trigger_command,                      //
                rmcs_msgs::DartServoCommand::FREE,           //
                1000                                         //
                ));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_up",                       // 动作名称
                    output.lifting_command,                  // 升降命令接口
                    output.lift_target_velocity,             // 升降目标速度接口
                    output.lift_exit_mode,                   // 电机退出状态接口
                    input.lift_left_velocity,                // 左电机速度反馈
                    input.lift_left_torque,                  // 左电机力矩反馈
                    input.lift_right_velocity,               // 右电机速度反馈
                    input.lift_right_torque,                 // 右电机力矩反馈
                    rmcs_msgs::DartMechanismCommand::UP,     // 升降命令设置
                    settings.lift_target_velocity,           // 同步带目标速度设置
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, // 电机退出模式设置
                    settings.lift_stall_velocity_threshold,  // 堵转速度阈值
                    settings.lift_stall_torque_threshold,    // 堵转力矩阈值
                    settings.lift_stall_confirm_ticks,       // 堵转确认帧数
                    20000                                    // 超时时间 ms
                    ));

            then(
                std::make_shared<FillingLimitServoAction>(
                    "filling_limit_servo",                   // 动作名称
                    output.limiting_command,                 // 限位舵机状态（输出）
                    rmcs_msgs::DartServoCommand::FREE,       // 先释放
                    rmcs_msgs::DartServoCommand::LOCK,       // 再锁回
                    settings.limiting_fill_ticks             // 预装填持续帧数
                    ));
        }
    }
};

} // namespace rmcs_dart_guidance::manager
