#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class CancelLaunchTask : public Task {
public:
    CancelLaunchTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("launch_cancel", "取消发射") {

        then(
            std::make_shared<BeltControlAction>(
                "belt_up",                               // 动作名称
                output.belt_command,                     // 同步带命令接口
                output.belt_target_velocity,             // 同步带目标速度接口
                output.belt_exit_mode,                   // 电机退出状态接口
                input.belt_arrive_flag,                  // 电机堵转标志接口
                rmcs_msgs::DartMechanismCommand::UP,     // 同步带命令设置
                settings.belt_up_target_velocity,        // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, // 电机退出模式设置
                20000                                    // 超时时间 ms
                ));

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",                          // 动作名称
                output.trigger_command,                  //
                rmcs_msgs::DartServoCommand::FREE,       //
                100                                      //
                ));

        then(
            std::make_shared<BeltControlAction>(
                "belt_down",                             // 动作名称
                output.belt_command,                     // 同步带命令接口
                output.belt_target_velocity,             // 同步带目标速度接口
                output.belt_exit_mode,                   // 电机退出状态接口
                input.belt_arrive_flag,                  // 电机堵转标志接口
                rmcs_msgs::DartMechanismCommand::DOWN,   // 同步带命令设置
                settings.belt_down_target_velocity,      // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,   // 电机退出模式设置
                20000                                    // 超时时间 ms
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
