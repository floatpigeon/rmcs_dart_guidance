#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"

#include <memory>
#include <string>

namespace rmcs_dart_guidance::manager {

std::shared_ptr<Task> make_slider_init_task(
    const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings);

std::shared_ptr<Task> make_task(
    const std::string& cmd, const ManagerInputContext& input, ManagerOutputContext& output,
    const ManagerSettings& settings, const ManagerRuntimeState& runtime_state);

} // namespace rmcs_dart_guidance::manager
