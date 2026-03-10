#pragma once

#include "manager/action/belt_move_action.hpp"
#include "manager/action/trigger_lock_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// CancelLaunchTask
//   取消发射任务 / 退膛任务。
//
//   顺序执行：同步带下行 → 扳机解锁 → 同步带复位
//
//   流程说明：
//     1. 同步带下行：驱动滑块到底部填装位，堵转检测判定到位
//     2. 扳机解锁：舵机旋转到解锁角度，等待物理到位
//     3. 同步带复位：驱动滑块回到顶部初始位，堵转检测判定到位
// ─────────────────────────────────────────────────────────────────────────────
class CancelLaunchTask : public Task {
public:
    /// @param belt_target_velocity   【输出引用】同步带目标速度
    /// @param left_belt_velocity     【输入引用】左同步带实际速度
    /// @param right_belt_velocity    【输入引用】右同步带实际速度
    /// @param trigger_target_angle   【输出引用】扳机目标角度
    CancelLaunchTask(
        double& belt_target_velocity, const double& left_belt_velocity,
        const double& right_belt_velocity, double& trigger_target_angle)
        : Task("cancel_launch", "取消发射") {

        // Step 1: 同步带下行 —— 驱动滑块到底部填装位
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

        // Step 2: 扳机解锁 —— 舵机旋转到解锁位
        then(
            std::make_shared<TriggerLockAction>(
                trigger_target_angle, // 扳机目标角度（输出）
                0.0,                  // 解锁角度（弧度）
                50                    // 舵机到位等待帧数
                ));

        // Step 3: 同步带复位 —— 驱动滑块回到顶部初始位
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
