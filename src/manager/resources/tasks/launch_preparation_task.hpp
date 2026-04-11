#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/belt_control_action.hpp"
#include "manager/resources/actions/filling_lift_action.hpp"
#include "manager/resources/actions/force_control_action.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"
#include "rmcs_msgs/dart_mechanism_command.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class LaunchPreparationTask : public Task {
public:
    LaunchPreparationTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings, const ManagerRuntimeState& runtime_state)
        : Task("launch_prepare", "滑块发射准备") {

        then(
            std::make_shared<BeltControlAction>(
                "belt_down",                                 // 动作名称
                output.belt_command,                         // 同步带命令接口
                output.belt_target_velocity,                 // 同步带目标速度接口
                output.belt_exit_mode,                       // 电机退出状态接口
                input.belt_arrive_flag,                      // 电机堵转标志接口
                rmcs_msgs::DartMechanismCommand::DOWN,       // 同步带命令设置
                settings.belt_down_target_velocity,          // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_HOLD_TORQUE,       // 电机退出模式设置
                20000                                        // 超时时间 ms
                ));

        if (runtime_state.fire_count > 0) {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock",                          //
                    output.trigger_command,                  //
                    rmcs_msgs::DartServoCommand::LOCK,       //
                    100                                      //
                    ));
            then(
                std::make_shared<FillingLiftAction>(
                    "filling_lift_down",                     // 动作名称
                    output.lifting_command,                  // 升降命令接口
                    output.lift_target_velocity,             // 升降目标速度接口
                    output.lift_exit_mode,                   // 电机退出状态接口
                    input.lift_arrive_flag,                  // 电机堵转标志接口
                    rmcs_msgs::DartMechanismCommand::DOWN,   // 升降命令设置
                    settings.lift_target_velocity,           // 同步带目标速度设置
                    rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY, // 电机退出模式设置
                    20000                                    // 超时时间 ms
                    ));
        } else {
            then(
                std::make_shared<TriggerControlAction>(
                    "trigger_lock",                          //
                    output.trigger_command,                  //
                    rmcs_msgs::DartServoCommand::LOCK,       //
                    100                                      //
                    ));
        }

        then(
            std::make_shared<BeltControlAction>(
                "belt_up",                                   // 动作名称
                output.belt_command,                         // 同步带命令接口
                output.belt_target_velocity,                 // 同步带目标速度接口
                output.belt_exit_mode,                       // 电机退出状态接口
                input.belt_arrive_flag,                      // 电机堵转标志接口
                rmcs_msgs::DartMechanismCommand::UP,         // 同步带命令设置
                settings.belt_up_target_velocity,            // 同步带目标速度设置
                rmcs_msgs::ExitMode::WAIT_ZERO_VELOCITY,     // 电机退出模式设置
                20000                                        // 超时时间 ms
                ));

        then(
            std::make_shared<ForceControlAction>(
                "force_close_loop",                          //
                output.force_error,                          //
                input.force_sensor_ch1,                      //
                input.force_sensor_ch2,                      //
                settings.force_setpoint,                     //
                settings.force_allowable_error,              //
                20000                                        //
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
