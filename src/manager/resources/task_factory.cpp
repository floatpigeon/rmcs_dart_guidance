#include "manager/resources/task_factory.hpp"

#include "manager/resources/actions/manual_angle_control_action.hpp"
#include "manager/resources/actions/manual_force_control_action.hpp"
#include "manager/resources/tasks/cancel_launch_task.hpp"
#include "manager/resources/tasks/fire_and_preload_task.hpp"
#include "manager/resources/tasks/launch_preparation_task.hpp"
#include "manager/resources/tasks/slider_init_task.hpp"

namespace rmcs_dart_guidance::manager {

ManagerTaskSpec make_slider_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings) {
    return ManagerTaskSpec{
        std::make_shared<SliderInitTask>(input, output, settings),
        ManagerTaskSlot::PRIMARY,
    };
}

std::optional<ManagerTaskSpec> make_task(
    const std::string& cmd, const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings, const ManagerRuntimeState& runtime_state) {
    if (cmd == "launch_prepare" || cmd == "launch-prepare") {
        return ManagerTaskSpec{
            std::make_shared<LaunchPreparationTask>(input, output, settings, runtime_state),
            ManagerTaskSlot::PRIMARY,
        };
    }

    if (cmd == "launch_cancel" || cmd == "cancel_launch" || cmd == "unload") {
        return ManagerTaskSpec{
            std::make_shared<CancelLaunchTask>(input, output, settings),
            ManagerTaskSlot::PRIMARY,
        };
    }

    if (cmd == "fire_preload" || cmd == "fire") {
        return ManagerTaskSpec{
            std::make_shared<FireAndPreloadTask>(input, output, settings, runtime_state),
            ManagerTaskSlot::PRIMARY,
        };
    }

    if (cmd == "manual_angle") {
        auto task = std::make_shared<Task>("manual_angle", "手动 yaw/pitch 调整");
        task->then(
            std::make_shared<DartManualAngleControlAction>(
                output.yaw_pitch_control_velocity[0], output.yaw_pitch_control_velocity[1],
                input.joystick_left, input.joystick_right, settings.max_transform_rate));
        return ManagerTaskSpec{std::move(task), ManagerTaskSlot::PRIMARY};
    }

    if (cmd == "manual_force") {
        auto task = std::make_shared<Task>("manual_force", "手动力丝杆速度调整");
        task->then(
            std::make_shared<DartManualForceControlAction>(
                output.force_control_velocity, input.joystick_right, settings.max_transform_rate,
                settings.manual_force_scale));
        return ManagerTaskSpec{std::move(task), ManagerTaskSlot::PRIMARY};
    }

    return std::nullopt;
}

} // namespace rmcs_dart_guidance::manager
