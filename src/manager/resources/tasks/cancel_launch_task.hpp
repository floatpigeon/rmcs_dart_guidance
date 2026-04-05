#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_move_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// CancelLaunchTask — 取消当前发射流程并回到安全待机位：
//   1. 同步带下行到卸载位
//   2. 通过 belt_down 的退出态保持滑块压在底部
//   3. 解锁扳机
//   4. 同步带上行复位
class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        const ManagerInputContext& input, ManagerOutputContext& output, const ManagerSettings&)
        : Task("launch_cancel", "取消发射") {

        // 在任务内部定义相关物理参数，避免从外部传参，让结构更整洁
        double down_velocity = 15.0;
        double up_velocity = 10.0;
        double hold_torque = 1.0;                 // Wait 时的保持力矩

        then(
            std::make_shared<BeltMoveAction>(
                "belt_down",                      // 动作名称
                output.belt_command,              // 同步带目标状态（输出）
                output.belt_target_velocity,      // 同步带目标速度（输出）
                output.belt_torque_limit,         // 同步带力矩限制（输出）
                output.belt_hold_torque,          // 同步带保持力矩（输出）
                output.belt_wait_zero_velocity,   // Wait 时使用零速闭环还是保留力矩
                input.left_belt_velocity,         // 左同步带反馈（输入）
                input.right_belt_velocity,        // 右同步带反馈（输入）
                input.left_belt_torque,           // 左同步带力矩（输入）
                input.right_belt_torque,          // 右同步带力矩（输入）
                rmcs_msgs::DartMotorStatus::DOWN, // 指令状态
                down_velocity,                    // 设定速度
                5.0,                              // 设定力矩限制
                hold_torque,                      // 设定保持力矩
                10000,                            // 超时帧数
                1.0,                              // 堵转速度阈值
                0.5,                              // 堵转力矩阈值
                100,                              // 堵转确认帧数
                50,                               // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_HOLD_TORQUE));

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                   // 动作名称
                output.trigger_lock_enable,       // 扳机锁定使能（输出）
                false,                            // 解锁（false）
                1000                              // 等待释放完成帧数
                ));

        then(
            std::make_shared<BeltMoveAction>(
                "belt_up",                        // 动作名称
                output.belt_command,              // 同步带目标状态（输出）
                output.belt_target_velocity,      // 同步带目标速度（输出）
                output.belt_torque_limit,         // 同步带力矩限制（输出）
                output.belt_hold_torque,          // 同步带保持力矩（输出）
                output.belt_wait_zero_velocity,   // Wait 时使用零速闭环还是保留力矩
                input.left_belt_velocity,         // 左同步带反馈（输入）
                input.right_belt_velocity,        // 右同步带反馈（输入）
                input.left_belt_torque,           // 左同步带力矩（输入）
                input.right_belt_torque,          // 右同步带力矩（输入）
                rmcs_msgs::DartMotorStatus::UP,   // 指令状态
                up_velocity,                      // 设定速度
                1.0,                              // 设定力矩限制
                hold_torque,                      // 设定保持力矩
                10000,                            // 超时帧数
                1.0,                              // 堵转速度阈值
                0.5,                              // 堵转力矩阈值
                100,                              // 堵转确认帧数
                50,                               // 最短运行帧数
                BeltMoveAction::ExitMode::WAIT_ZERO_VELOCITY));
    }
};

} // namespace rmcs_dart_guidance::manager
