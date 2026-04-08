#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/manual_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class ManualControlTask : public Task {
public:
    ManualControlTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("manual_control", "手动控制") {

        then(
            std::make_shared<ManualControlAction>(
                "manual_control",                      // 动作名称
                input.remote_left_switch,              // 左拨杆状态
                input.remote_right_switch,             // 右拨杆状态
                input.remote_left_joystick,            // 左摇杆输入
                input.remote_right_joystick,           // 右摇杆输入
                output.belt_command,                   // 同步带命令接口
                output.belt_target_velocity,           // 同步带目标速度接口
                output.belt_exit_mode,                 // 同步带退出模式接口
                output.force_control_velocity,         // 力度控制速度接口
                output.angle_control_vector,           // 姿态控制速度接口
                settings.manual_angle_target_velocity, // 手动角度最大速度
                settings.manual_force_target_velocity, // 手动力度最大速度
                settings.manual_belt_target_velocity   // 手动同步带最大速度
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
