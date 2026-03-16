#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/delay_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        rmcs_msgs::DartSliderStatus& belt_command,
        double& belt_target_velocity, double& belt_torque_limit, double& belt_hold_torque,
        const double& left_belt_velocity, const double& right_belt_velocity,
        bool& trigger_lock_enable)
        : Task("cancel_launch", "取消发射") {

        // 在任务内部定义相关物理参数，避免从外部传参，让结构更整洁
        double velocity = 50.0;
        double torque_limit = 3.0;
        double hold_torque = 1.0; // Wait 时的保持力矩

        then(
            std::make_shared<BeltMoveAction>(
                "belt_move_down",                  // 动作名称
                belt_command,                      // 同步带目标状态（输出）
                belt_target_velocity,              // 同步带目标速度（输出）
                belt_torque_limit,                 // 同步带力矩限制（输出）
                belt_hold_torque,                  // 同步带保持力矩（输出）
                left_belt_velocity,                // 左同步带反馈（输入）
                right_belt_velocity,               // 右同步带反馈（输入）
                rmcs_msgs::DartSliderStatus::DOWN, // 指令状态
                velocity,                          // 设定速度
                torque_limit,                      // 设定力矩限制
                hold_torque,                       // 设定保持力矩
                5000,                              // 超时帧数
                1.0,                               // 堵转阈值
                100,                               // 堵转确认帧数
                50                                 // 最短运行帧数
                ));

        then(
            std::make_shared<DelayAction>(
                "belt_wait",                       // 动作名称
                50                                 // 等待帧数
                ));

        then(
            std::make_shared<TriggerControlAction>(
                trigger_lock_enable,               // 扳机锁定使能（输出）
                false,                             // 解锁（false）
                1000                               // 舵机到位等待帧数
                ));

        then(
            std::make_shared<BeltMoveAction>(
                "belt_reset",                      // 动作名称
                belt_command,                      // 同步带目标状态（输出）
                belt_target_velocity,              // 同步带目标速度（输出）
                belt_torque_limit,                 // 同步带力矩限制（输出）
                belt_hold_torque,                  // 同步带保持力矩（输出）
                left_belt_velocity,                // 左同步带反馈（输入）
                right_belt_velocity,               // 右同步带反馈（输入）
                rmcs_msgs::DartSliderStatus::UP,   // 指令状态
                velocity,                          // 设定速度
                torque_limit,                      // 设定力矩限制
                hold_torque,                       // 设定保持力矩
                5000,                              // 超时帧数
                1.0,                               // 堵转阈值
                100,                               // 堵转确认帧数
                50                                 // 最短运行帧数
                ));
    }
};

} // namespace rmcs_dart_guidance::manager