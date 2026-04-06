#include "manager/core/components/gui_bridge_state_port.hpp"

#include "manager/gui_bridge_json_utils.hpp"

#include <chrono>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::manager {

GuiBridgeStatePort::GuiBridgeStatePort(std::shared_ptr<GuiBridgeRuntime> runtime)
    : runtime_(std::move(runtime)) {
    register_input("/dart/manager/debug/lifecycle_state", lifecycle_state_input_);
    register_input("/dart/manager/debug/current_task", current_task_input_);
    register_input("/dart/manager/debug/current_action", current_action_input_);
    register_input("/dart/manager/fire_count", fire_count_input_);
    register_input("/dart/manager/debug/queue_json", queue_json_input_);
    register_input("/dart/manager/debug/last_error_json", last_error_json_input_);
    register_input("/dart/drive_belt/left/velocity", belt_left_velocity_input_);
    register_input("/dart/drive_belt/right/velocity", belt_right_velocity_input_);
    register_input("/dart/drive_belt/left/torque", belt_left_torque_input_);
    register_input("/dart/drive_belt/right/torque", belt_right_torque_input_);
    register_input("/dart/lifting_left/velocity", lift_left_velocity_input_);
    register_input("/dart/lifting_right/velocity", lift_right_velocity_input_);

    last_publish_time_ = std::chrono::steady_clock::now();
}

void GuiBridgeStatePort::update() {
    const std::string snapshot = build_snapshot_json();
    const std::string current_error_json = *last_error_json_input_;
    const auto now = std::chrono::steady_clock::now();
    const bool due = now - last_publish_time_ >= std::chrono::milliseconds(100);

    std::scoped_lock lock(runtime_->state_mutex);

    if (current_error_json != last_error_json_cache_) {
        if (!current_error_json.empty()) {
            runtime_->pending_events.push_back(
                wrap_object_with_type("fault.event", current_error_json));
        } else if (!last_error_json_cache_.empty()) {
            runtime_->pending_events.push_back(
                "{\"type\":\"fault.cleared\",\"timestamp_ms\":"
                + std::to_string(current_system_timestamp_ms()) + '}');
        }

        last_error_json_cache_ = current_error_json;
    }

    if (snapshot != last_snapshot_json_ || due) {
        runtime_->latest_snapshot_json = snapshot;
        runtime_->snapshot_version += 1;
        last_snapshot_json_ = snapshot;
        last_publish_time_ = now;
    }
}

std::string GuiBridgeStatePort::build_snapshot_json() const {
    std::ostringstream builder;
    builder << "{\"type\":\"state.snapshot\",\"manager\":{\"lifecycle_state\":\""
            << escape_json_string(*lifecycle_state_input_) << '"'
            << ",\"current_task\":\"" << escape_json_string(*current_task_input_) << '"'
            << ",\"current_action\":\"" << escape_json_string(*current_action_input_) << '"'
            << ",\"fire_count\":" << *fire_count_input_
            << ",\"queue\":";

    if ((*queue_json_input_).empty()) {
        builder << "[]";
    } else {
        builder << *queue_json_input_;
    }

    builder << ",\"last_error\":";
    if ((*last_error_json_input_).empty()) {
        builder << "null";
    } else {
        builder << *last_error_json_input_;
    }

    builder << "},\"feedback\":{\"belt\":{\"left_velocity\":" << *belt_left_velocity_input_
            << ",\"right_velocity\":" << *belt_right_velocity_input_
            << ",\"left_torque\":" << *belt_left_torque_input_
            << ",\"right_torque\":" << *belt_right_torque_input_
            << "},\"lift\":{\"left_velocity\":" << *lift_left_velocity_input_
            << ",\"right_velocity\":" << *lift_right_velocity_input_ << "}}}";
    return builder.str();
}

std::string GuiBridgeStatePort::wrap_object_with_type(
    const std::string& type, const std::string& object_json) {
    if (object_json.size() < 2 || object_json.front() != '{' || object_json.back() != '}') {
        return "{\"type\":\"" + escape_json_string(type) + "\"}";
    }

    const std::string inner = object_json.substr(1, object_json.size() - 2);
    if (inner.empty()) {
        return "{\"type\":\"" + escape_json_string(type) + "\"}";
    }

    return "{\"type\":\"" + escape_json_string(type) + "\"," + inner + '}';
}

} // namespace rmcs_dart_guidance::manager
