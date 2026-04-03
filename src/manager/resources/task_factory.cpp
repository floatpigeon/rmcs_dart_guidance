#include "manager/resources/task_factory.hpp"

#include "manager/resources/actions/manual_angle_control_action.hpp"
#include "manager/resources/actions/manual_force_control_action.hpp"
#include "manager/resources/tasks/cancel_launch_task.hpp"
#include "manager/resources/tasks/fire_and_preload_task.hpp"
#include "manager/resources/tasks/launch_preparation_task.hpp"
#include "manager/resources/tasks/slider_init_task.hpp"

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_slider_init_task(const ManagerTaskContext& context) {
    return std::make_shared<SliderInitTask>(
        context.belt_command,
        context.belt_target_velocity,
        context.belt_torque_limit,
        context.belt_hold_torque,
        context.belt_wait_zero_velocity,
        context.left_belt_velocity,
        context.right_belt_velocity,
        context.left_belt_torque,
        context.right_belt_torque);
}

std::shared_ptr<Task> make_task(const std::string& cmd, const ManagerTaskContext& context) {
    if (cmd == "launch_prepare" || cmd == "launch-prepare") {
        return std::make_shared<LaunchPreparationTask>(
            context.belt_command,
            context.belt_target_velocity,
            context.belt_torque_limit,
            context.belt_hold_torque,
            context.belt_wait_zero_velocity,
            context.left_belt_velocity,
            context.right_belt_velocity,
            context.left_belt_torque,
            context.right_belt_torque,
            context.trigger_lock_enable,
            context.lifting_command,
            context.lifting_left_vel_fb,
            context.lifting_right_vel_fb,
            context.lifting_stall_threshold,
            context.lifting_stall_confirm_ticks,
            context.lifting_stall_min_run_ticks,
            context.lifting_stall_timeout_ticks);
    }

    if (cmd == "unload" || cmd == "cancel_launch") {
        return std::make_shared<CancelLaunchTask>(
            context.belt_command,
            context.belt_target_velocity,
            context.belt_torque_limit,
            context.belt_hold_torque,
            context.belt_wait_zero_velocity,
            context.left_belt_velocity,
            context.right_belt_velocity,
            context.left_belt_torque,
            context.right_belt_torque,
            context.trigger_lock_enable,
            context.lifting_command,
            context.lifting_left_vel_fb,
            context.lifting_right_vel_fb,
            context.lifting_stall_threshold,
            context.lifting_stall_confirm_ticks,
            context.lifting_stall_min_run_ticks,
            context.lifting_stall_timeout_ticks);
    }

    if (cmd == "fire") {
        return std::make_shared<FireAndPreloadTask>(
            context.trigger_lock_enable,
            context.lifting_command,
            context.lifting_left_vel_fb,
            context.lifting_right_vel_fb,
            context.lifting_stall_threshold,
            context.lifting_stall_confirm_ticks,
            context.lifting_stall_min_run_ticks,
            context.lifting_stall_timeout_ticks,
            context.limiting_command,
            context.limiting_fill_ticks);
    }

    if (cmd == "manual_angle") {
        auto task = std::make_shared<Task>("manual_angle", "手动 yaw/pitch 调整");
        task->then(std::make_shared<DartManualAngleControlAction>(
            context.yaw_pitch_control_velocity[0],
            context.yaw_pitch_control_velocity[1],
            context.joystick_left,
            context.joystick_right,
            context.max_transform_rate));
        return task;
    }

    if (cmd == "manual_force") {
        auto task = std::make_shared<Task>("manual_force", "手动力丝杆速度调整");
        task->then(std::make_shared<DartManualForceControlAction>(
            context.force_control_velocity,
            context.joystick_right,
            context.max_transform_rate,
            context.manual_force_scale));
        return task;
    }

    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
