#pragma once

#include "manager/host_bridge_json_utils.hpp"
#include "manager/host_bridge_runtime.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>

#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class HostCommandBridge : public rmcs_executor::Component {
public:
    explicit HostCommandBridge(std::shared_ptr<HostBridgeRuntime> runtime)
        : runtime_(std::move(runtime)) {
        register_output("/dart/source/host_command", command_output_, std::string{});
    }

    void update() override {
        if (clear_on_next_tick_) {
            *command_output_ = std::string{};
            clear_on_next_tick_ = false;
            return;
        }

        std::string next_command;
        {
            std::scoped_lock lock(runtime_->command_mutex);
            if (runtime_->pending_host_commands.empty()) {
                return;
            }

            next_command = std::move(runtime_->pending_host_commands.front());
            runtime_->pending_host_commands.pop_front();
        }

        *command_output_ = next_command;
        clear_on_next_tick_ = true;
    }

private:
    std::shared_ptr<HostBridgeRuntime> runtime_;
    OutputInterface<std::string> command_output_;
    bool clear_on_next_tick_{false};
};

class HostBridgeStatePort : public rmcs_executor::Component {
public:
    explicit HostBridgeStatePort(std::shared_ptr<HostBridgeRuntime> runtime)
        : runtime_(std::move(runtime))
        , last_publish_time_(std::chrono::steady_clock::now()) {
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
    }

    void update() override {
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

private:
    std::string build_snapshot_json() const {
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

    static std::string wrap_object_with_type(
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

    std::shared_ptr<HostBridgeRuntime> runtime_;

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
