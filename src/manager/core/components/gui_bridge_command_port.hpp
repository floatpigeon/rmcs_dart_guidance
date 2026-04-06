#pragma once

#include "manager/gui_bridge_runtime.hpp"

#include <memory>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class GuiBridgeCommandPort : public rmcs_executor::Component {
public:
    explicit GuiBridgeCommandPort(std::shared_ptr<GuiBridgeRuntime> runtime);

    void update() override;

private:
    std::shared_ptr<GuiBridgeRuntime> runtime_;
    OutputInterface<std::string> command_output_;
    bool clear_on_next_tick_{false};
};

} // namespace rmcs_dart_guidance::manager
