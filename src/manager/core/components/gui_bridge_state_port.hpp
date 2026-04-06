#pragma once

#include "manager/gui_bridge_runtime.hpp"

#include <chrono>
#include <memory>
#include <string>

#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class GuiBridgeStatePort : public rmcs_executor::Component {
public:
    explicit GuiBridgeStatePort(std::shared_ptr<GuiBridgeRuntime> runtime);

    void update() override;

private:
    std::string build_snapshot_json() const;
    static std::string wrap_object_with_type(const std::string& type, const std::string& object_json);

    std::shared_ptr<GuiBridgeRuntime> runtime_;

    InputInterface<std::string> lifecycle_state_input_;
    InputInterface<std::string> current_task_input_;
    InputInterface<std::string> current_action_input_;
    InputInterface<uint32_t> fire_count_input_;
    InputInterface<std::string> queue_json_input_;
    InputInterface<std::string> last_error_json_input_;
    InputInterface<double> belt_left_velocity_input_;
    InputInterface<double> belt_right_velocity_input_;
    InputInterface<double> belt_left_torque_input_;
    InputInterface<double> belt_right_torque_input_;
    InputInterface<double> lift_left_velocity_input_;
    InputInterface<double> lift_right_velocity_input_;

    std::string last_snapshot_json_;
    std::string last_error_json_cache_;
    std::chrono::steady_clock::time_point last_publish_time_;
};

} // namespace rmcs_dart_guidance::manager
