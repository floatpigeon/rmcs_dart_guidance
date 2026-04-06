#include "manager/core/components/gui_bridge_command_port.hpp"

#include <mutex>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

GuiBridgeCommandPort::GuiBridgeCommandPort(std::shared_ptr<GuiBridgeRuntime> runtime)
    : runtime_(std::move(runtime)) {
    register_output("/dart/manager/gui_command", command_output_, std::string{});
}

void GuiBridgeCommandPort::update() {
    if (clear_on_next_tick_) {
        *command_output_ = std::string{};
        clear_on_next_tick_ = false;
        return;
    }

    std::string next_command;
    {
        std::scoped_lock lock(runtime_->command_mutex);
        if (runtime_->pending_commands.empty()) {
            return;
        }

        next_command = std::move(runtime_->pending_commands.front());
        runtime_->pending_commands.pop_front();
    }

    *command_output_ = next_command;
    clear_on_next_tick_ = true;
}

} // namespace rmcs_dart_guidance::manager
