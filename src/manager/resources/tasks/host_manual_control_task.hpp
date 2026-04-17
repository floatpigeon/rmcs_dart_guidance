#pragma once

#include <memory>

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/host_manual_control_action.hpp"

namespace rmcs_dart_guidance::manager {

class HostManualControlTask : public Task {
public:
    HostManualControlTask(
        const ManagerInputContext& input, ManagerOutputContext& output,
        const ManagerSettings& settings)
        : Task("host_manual_control", "手动接管") {
        then(
            std::make_shared<HostManualControlAction>(
                "host_manual_control",
                input.host_manual_belt_direction,
                input.host_manual_lift_direction,
                input.host_manual_yaw_direction,
                input.host_manual_pitch_direction,
                input.host_manual_trigger_command,
                output.belt_command,
                output.belt_target_velocity,
                output.belt_exit_mode,
                output.lifting_command,
                output.lift_target_velocity,
                output.lift_exit_mode,
                output.trigger_command,
                output.force_error,
                output.angle_error_vector,
                settings.manual_angle_max_error,
                settings.belt_manual_setting_velocity,
                settings.lift_target_velocity));
    }
};

} // namespace rmcs_dart_guidance::manager
