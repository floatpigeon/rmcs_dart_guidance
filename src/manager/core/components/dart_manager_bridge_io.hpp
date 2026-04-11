#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"

#include <chrono>
#include <deque>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class DartManagerBridgeIo {
public:
    void register_interfaces(rmcs_executor::Component& component) {
        component.register_input("/dart/manager/command", command_input_, false);

        component.register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});
        component.register_output(
            "/dart/manager/debug/lifecycle_state", debug_lifecycle_state_output_,
            std::string{to_string(ManagerLifecycleState::IDLE)});
        component.register_output(
            "/dart/manager/debug/current_task", debug_current_task_output_, std::string{});
        component.register_output(
            "/dart/manager/debug/current_action", debug_current_action_output_, std::string{});
        component.register_output(
            "/dart/manager/debug/queue_json", debug_queue_json_output_, std::string{"[]"});
        component.register_output(
            "/dart/manager/debug/last_error_json", debug_last_error_json_output_, std::string{});
    }

    void bind_optional_inputs(const rclcpp::Logger& logger) {
        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(logger, "Failed to fetch \"/dart/manager/command\". Set to empty string.");
        }
    }

    std::string poll_command() {
        std::string cmd = command_input_.ready() ? *command_input_ : std::string{};

        if (cmd.empty()) {
            last_command_.clear();
            return {};
        }

        if (cmd == last_command_) {
            return {};
        }

        last_command_ = cmd;
        return cmd;
    }

    void clear_last_error() { last_error_json_.clear(); }
    void set_last_error(
        const std::string& task_name, const std::string& action_name, ActionFailureReason reason,
        const std::string& description) {
        last_error_json_ = build_last_error_json(task_name, action_name, reason, description);
    }

    void sync_debug_outputs(
        const ManagerRuntimeState& runtime_state, const std::string& current_task_name,
        const std::string& current_action_name,
        const std::deque<std::shared_ptr<Task>>& task_queue) {
        *fire_count_output_ = runtime_state.fire_count;
        *debug_lifecycle_state_output_ = to_string(runtime_state.lifecycle_state);
        *debug_current_task_output_ = current_task_name;
        *debug_current_action_output_ = current_action_name;
        *debug_queue_json_output_ = build_queue_json(task_queue);
        *debug_last_error_json_output_ = last_error_json_;
    }

private:
    static std::string escape_json_string(std::string_view input) {
        std::string output;
        output.reserve(input.size() + 8);

        for (const char c : input) {
            switch (c) {
            case '\\': output += "\\\\"; break;
            case '"': output += "\\\""; break;
            case '\b': output += "\\b"; break;
            case '\f': output += "\\f"; break;
            case '\n': output += "\\n"; break;
            case '\r': output += "\\r"; break;
            case '\t': output += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    output += '?';
                } else {
                    output.push_back(c);
                }
                break;
            }
        }

        return output;
    }

    static uint64_t current_system_timestamp_ms() {
        return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                         std::chrono::system_clock::now().time_since_epoch())
                                         .count());
    }

    static std::string build_queue_json(const std::deque<std::shared_ptr<Task>>& task_queue) {
        std::ostringstream builder;
        builder << '[';

        bool first = true;
        for (const auto& task : task_queue) {
            if (!task) {
                continue;
            }

            if (!first) {
                builder << ',';
            }

            const std::string display_name =
                task->description().empty() ? task->name() : task->description();
            builder << "{\"task_name\":\"" << escape_json_string(task->name()) << '"'
                    << ",\"display_name\":\"" << escape_json_string(display_name) << '"'
                    << ",\"status\":\"queued\"}";
            first = false;
        }

        builder << ']';
        return builder.str();
    }

    static std::string build_last_error_json(
        const std::string& task_name, const std::string& action_name, ActionFailureReason reason,
        const std::string& description) {
        const std::string message = (description.empty() ? task_name : description) + "执行失败";

        std::ostringstream builder;
        builder << "{\"task_name\":\"" << escape_json_string(task_name) << '"'
                << ",\"action_name\":\"" << escape_json_string(action_name) << '"'
                << ",\"reason\":\"" << escape_json_string(to_string(reason)) << '"'
                << ",\"message\":\"" << escape_json_string(message) << '"'
                << ",\"timestamp_ms\":" << current_system_timestamp_ms() << '}';
        return builder.str();
    }

    rmcs_executor::Component::InputInterface<std::string> command_input_;

    rmcs_executor::Component::OutputInterface<uint32_t> fire_count_output_;
    rmcs_executor::Component::OutputInterface<std::string> debug_lifecycle_state_output_;
    rmcs_executor::Component::OutputInterface<std::string> debug_current_task_output_;
    rmcs_executor::Component::OutputInterface<std::string> debug_current_action_output_;
    rmcs_executor::Component::OutputInterface<std::string> debug_queue_json_output_;
    rmcs_executor::Component::OutputInterface<std::string> debug_last_error_json_output_;

    std::string last_command_;
    std::string last_error_json_;
};

} // namespace rmcs_dart_guidance::manager
