#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_move_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// LaunchPreparationTask — 将机构切换到发射准备态：
//   1. 同步带下行
//   2. 通过 belt_down 的退出态保持滑块压在底部
//   3. 扳机锁定
//   4. 根据 fire_count 判断是否执行填装升降下行
//   5. 同步带上行复位
class LaunchPreparationTask : public Task {
public:
    LaunchPreparationTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare", "滑块发射准备") {

        // 到底后借助机械限位和保持力矩继续把滑块压在底部，直到后续动作完成。

        then(
            std::make_shared<BeltMoveAction>(
                "belt_down",                              // 动作名称
                output.belt_command,                      // 同步带目标状态（输出）
                output.belt_target_velocity,              // 同步带目标速度（输出）
                output.belt_torque_limit,                 // 同步带力矩限制（输出）
                output.belt_hold_torque,                  // 同步带保持力矩（输出）
                output.belt_wait_zero_velocity,           // Wait 时使用零速闭环还是保留力矩
                input.left_belt_velocity,                 // 左同步带反馈（输入）
                input.right_belt_velocity,                // 右同步带反馈（输入）
                input.left_belt_torque,                   // 左同步带力矩（输入）
                input.right_belt_torque,                  // 右同步带力矩（输入）
                rmcs_msgs::DartMotorStatus::DOWN,         // 指令状态
                10.0,                                     // 设定速度
                5.0,                                      // 设定力矩限制
                5.0,                                      // 设定保持力矩
                10000,                                    // 超时帧数
                1.0,                                      // 堵转速度阈值
                0.5,                                      // 堵转力矩阈值
                100,                                      // 堵转确认帧数
                50,                                       // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_HOLD_TORQUE));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock", output.trigger_lock_enable, true, 1000));
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_down",                  // 动作名称
                    output.lifting_command,               // 升降指令（输出）
                    rmcs_msgs::DartMotorStatus::DOWN,     // 指令状态
                    input.lifting_left_vel_fb,            // 左升降电机速度反馈（输入）
                    input.lifting_right_vel_fb,           // 右升降电机速度反馈（输入）
                    settings.lifting_stall_threshold,     // 堵转速度阈值
                    settings.lifting_stall_confirm_ticks, // 堵转确认帧数
                    settings.lifting_stall_min_run_ticks, // 最短运行帧数
                    settings.lifting_stall_timeout_ticks  // 超时帧数
                    ));
        } else {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock", output.trigger_lock_enable, true, 1000));
        }

        then(
            std::make_shared<BeltMoveAction>(
                "belt_up",                                // 动作名称
                output.belt_command,                      // 同步带目标状态（输出）
                output.belt_target_velocity,              // 同步带目标速度（输出）
                output.belt_torque_limit,                 // 同步带力矩限制（输出）
                output.belt_hold_torque,                  // 同步带保持力矩（输出）
                output.belt_wait_zero_velocity,           // Wait 时使用零速闭环还是保留力矩
                input.left_belt_velocity,                 // 左同步带反馈（输入）
                input.right_belt_velocity,                // 右同步带反馈（输入）
                input.left_belt_torque,                   // 左同步带力矩（输入）
                input.right_belt_torque,                  // 右同步带力矩（输入）
                rmcs_msgs::DartMotorStatus::UP,           // 指令状态
                20.0,                                     // 设定速度
                5.0,                                      // 设定力矩限制
                1.0,                                      // 设定保持力矩
                5000,                                     // 超时帧数
                0.5,                                      // 堵转速度阈值
                0.5,                                      // 堵转力矩阈值
                200,                                      // 堵转确认帧数
                50,                                       // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY));
    }
};

} // namespace rmcs_dart_guidance::manager
