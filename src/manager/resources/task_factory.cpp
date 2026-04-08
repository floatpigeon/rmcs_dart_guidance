#include "manager/resources/task_factory.hpp"

#include "manager/resources/tasks/cancel_launch_task.hpp"
#include "manager/resources/tasks/fire_and_preload_task.hpp"
#include "manager/resources/tasks/launch_preparation_task.hpp"
#include "manager/resources/tasks/manual_control_task.hpp"
#include "manager/resources/tasks/slider_init_task.hpp"

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_slider_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings) {
    return std::make_shared<SliderInitTask>(input, output, settings);
}

std::shared_ptr<Task> make_task(
    const std::string& cmd, const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings, const ManagerRuntimeState& runtime_state) {
    if (cmd == "launch_prepare" || cmd == "launch-prepare") {
        return std::make_shared<LaunchPreparationTask>(input, output, settings, runtime_state);
    }

    if (cmd == "launch_cancel" || cmd == "cancel_launch" || cmd == "unload") {
        return std::make_shared<CancelLaunchTask>(input, output, settings);
    }

    if (cmd == "fire_preload" || cmd == "fire") {
        return std::make_shared<FireAndPreloadTask>(input, output, settings, runtime_state);
    }

    if (cmd == "manual_control" || cmd == "manual-control" || cmd == "manual") {
        return std::make_shared<ManualControlTask>(input, output, settings);
    }

    return nullptr;
}

} // namespace rmcs_dart_guidance::manager
