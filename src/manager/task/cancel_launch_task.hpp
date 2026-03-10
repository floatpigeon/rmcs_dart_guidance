#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        double& belt_target_velocity, const double& left_belt_velocity,
        const double& right_belt_velocity, double& trigger_target_angle)
        : Task("cancel_launch", "取消发射") {

        then(
            std::make_shared<BeltMoveAction>(
                "belt_move_down",     // 动作名称
                belt_target_velocity, // 同步带目标速度（输出）
                left_belt_velocity,   // 左同步带反馈（输入）
                right_belt_velocity,  // 右同步带反馈（输入）
                -100.0,               // 指令速度（负值 = 下行）
                1000,                 // 超时帧数
                1.0,                  // 堵转阈值
                20,                   // 堵转确认帧数
                50                    // 最短运行帧数
                ));

        then(
            std::make_shared<TriggerControlAction>(
                trigger_target_angle, // 扳机目标角度（输出）
                0.0,                  // 解锁角度（弧度）
                50                    // 舵机到位等待帧数
                ));

        then(
            std::make_shared<BeltMoveAction>(
                "belt_reset",         // 动作名称
                belt_target_velocity, // 同步带目标速度（输出）
                left_belt_velocity,   // 左同步带反馈（输入）
                right_belt_velocity,  // 右同步带反馈（输入）
                100.0,                // 指令速度（正值 = 上行复位）
                1000,                 // 超时帧数
                1.0,                  // 堵转阈值
                20,                   // 堵转确认帧数
                50                    // 最短运行帧数
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
