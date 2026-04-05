#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"

#include <memory>
#include <optional>
#include <string>

namespace rmcs_dart_guidance::manager {

struct ManagerTaskSpec {
    std::shared_ptr<Task> task;
    ManagerTaskSlot slot{ManagerTaskSlot::PRIMARY};
};

ManagerTaskSpec make_slider_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings);

std::optional<ManagerTaskSpec> make_task(
    const std::string& cmd, const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings, const ManagerRuntimeState& runtime_state);

} // namespace rmcs_dart_guidance::manager
