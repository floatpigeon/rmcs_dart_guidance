#include "manager/core/runtime/task.hpp"
#include "manager/gui_bridge_json_utils.hpp"
#include "manager/resources/task_factory.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class DartManager
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManager()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart/drive_belt/left/velocity", belt_left_velocity_);
        register_input("/dart/drive_belt/right/velocity", belt_right_velocity_);
        register_input("/dart/drive_belt/left/torque", belt_left_torque_);
        register_input("/dart/drive_belt/right/torque", belt_right_torque_);
        register_input("/dart/lifting_left/velocity", lifting_left_velocity_);
        register_input("/dart/lifting_right/velocity", lifting_right_velocity_);

        register_input("/dart/manager/command", remote_command_input_, false);
        register_input("/dart/manager/gui_command", gui_command_input_, false);

        register_output(
            "/dart/manager/belt/command", belt_command_, rmcs_msgs::DartMechanismCommand::WAIT);
        register_output("/dart/manager/belt/target_velocity", belt_target_velocity_, 0.0);
        register_output("/dart/manager/belt/torque_limit", belt_torque_limit_, 0.0);
        register_output("/dart/manager/belt/hold_torque", belt_hold_torque_, 0.0);
        register_output("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_output("/dart/manager/trigger/lock_enable", trigger_lock_enable_, false);
        register_output(
            "/dart/manager/lifting/command", lifting_command_,
            rmcs_msgs::DartMechanismCommand::WAIT);
        register_output(
            "/dart/manager/limiting/command", limiting_command_, rmcs_msgs::DartServoCommand::LOCK);
        register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});
        register_output(
            "/dart/manager/debug/lifecycle_state", debug_lifecycle_state_output_,
            std::string{to_string(ManagerLifecycleState::IDLE)});
        register_output(
            "/dart/manager/debug/current_task", debug_current_task_output_, std::string{});
        register_output("/dart/manager/debug/current_action", debug_current_action_output_, std::string{});
        register_output("/dart/manager/debug/queue_json", debug_queue_json_output_, std::string{"[]"});
        register_output(
            "/dart/manager/debug/last_error_json", debug_last_error_json_output_, std::string{});

        limiting_fill_ticks_ = (uint64_t)get_parameter("limiting_fill_ticks").as_int();
        lifting_stall_threshold_ = get_parameter("lifting_stall_threshold").as_double();
        lifting_stall_confirm_ticks_ =
            (uint64_t)get_parameter("lifting_stall_confirm_ticks").as_int();
        lifting_stall_min_run_ticks_ =
            (uint64_t)get_parameter("lifting_stall_min_run_ticks").as_int();
        lifting_stall_timeout_ticks_ =
            (uint64_t)get_parameter("lifting_stall_timeout_ticks").as_int();

        reset_fire_count();
        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] initialized");
    }

    void before_updating() override {
        if (!remote_command_input_.ready()) {
            remote_command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(logger_, "Failed to fetch \"/dart/manager/command\". Set to empty string.");
        }
        if (!gui_command_input_.ready()) {
            gui_command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/dart/manager/gui_command\". Set to empty string.");
        }

        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        submit_task(make_slider_init_task(input, output, manager_settings));
        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] queued SliderInitTask on startup");
    }

    void update() override {
        poll_command();

        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            sync_debug_outputs();
            return;
        }

        while (runtime_state_.lifecycle_state != ManagerLifecycleState::ERROR) {
            if (!task_state_.current_task) {
                if (task_state_.task_queue.empty()) {
                    if (runtime_state_.lifecycle_state != ManagerLifecycleState::IDLE) {
                        transition_to(ManagerLifecycleState::IDLE);
                    }
                    break;
                }
                dispatch_next_task();
            }

            if (!task_state_.current_task) {
                break;
            }

            const ActionStatus status = tick_current_task();
            if (status == ActionStatus::RUNNING
                || runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
                break;
            }
        }

        sync_debug_outputs();
    }

private:
    struct TaskState {
        std::shared_ptr<Task> current_task;
        std::deque<std::shared_ptr<Task>> task_queue;
        bool first_tick_of_task{true};
    };

    void poll_command() {
        const std::string remote_cmd =
            remote_command_input_.ready() ? *remote_command_input_ : std::string{};
        const std::string gui_cmd = gui_command_input_.ready() ? *gui_command_input_ : std::string{};
        const std::string& cmd = remote_cmd.empty() ? gui_cmd : remote_cmd;

        if (cmd.empty()) {
            runtime_state_.last_command.clear();
            return;
        }

        if (cmd == runtime_state_.last_command) {
            return;
        }

        if (cmd == "cancel") {
            cancel_all();
        } else if (cmd == "recover") {
            recover();
        } else {
            auto input = input_context();
            auto output = output_context();
            auto manager_settings = settings();
            auto task = make_task(cmd, input, output, manager_settings, runtime_state_);

            RCLCPP_INFO(logger_, "[DartManager] received command: '%s'", cmd.c_str());
            if (task) {
                submit_task(std::move(task));
            } else {
                RCLCPP_WARN(logger_, "[DartManager] unknown command: '%s'", cmd.c_str());
            }
        }

        runtime_state_.last_command = cmd;
    }

    void cancel_all() {
        cancel_task_state(task_state_, ActionCancelReason::EXTERNAL_CANCEL);
        enter_belt_wait_zero_velocity_mode();
        *lifting_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        *limiting_command_ = rmcs_msgs::DartServoCommand::LOCK;

        RCLCPP_WARN(logger_, "[DartManager] all tasks cancelled");
        transition_to(ManagerLifecycleState::IDLE);
    }

    void recover() {
        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            reset_task_state(task_state_);
            RCLCPP_INFO(logger_, "[DartManager] recovered from ERROR, state=IDLE");
            transition_to(ManagerLifecycleState::IDLE);
        }

        last_error_json_.clear();
        reset_fire_count();
        *limiting_command_ = rmcs_msgs::DartServoCommand::LOCK;

        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        submit_task(make_slider_init_task(input, output, manager_settings));
        RCLCPP_INFO(logger_, "[DartManager] queued SliderInitTask for recovery");
    }

    void submit_task(std::shared_ptr<Task> task) {
        if (!task) {
            return;
        }

        task_state_.task_queue.push_back(std::move(task));
        RCLCPP_INFO(
            logger_, "[DartManager] task queued: %s (queue size=%zu)",
            task_state_.task_queue.back()->name().c_str(), task_state_.task_queue.size());
    }

    void dispatch_next_task() {
        if (task_state_.task_queue.empty()) {
            return;
        }

        task_state_.current_task = std::move(task_state_.task_queue.front());
        task_state_.task_queue.pop_front();
        task_state_.current_task->bind_runtime_context(
            ActionRuntimeContext{task_state_.current_task->name(), &logger_});

        RCLCPP_INFO(
            logger_, "[DartManager] dispatching task: '%s'",
            task_state_.current_task->name().c_str());
        task_state_.first_tick_of_task = true;
        if (runtime_state_.lifecycle_state != ManagerLifecycleState::RUNNING) {
            transition_to(ManagerLifecycleState::RUNNING);
        }
    }

    ActionStatus tick_current_task() {
        if (!task_state_.current_task) {
            return ActionStatus::SUCCESS;
        }

        const ActionStatus status = task_state_.first_tick_of_task
                                      ? task_state_.current_task->tick_first()
                                      : task_state_.current_task->tick();
        task_state_.first_tick_of_task = false;

        if (status == ActionStatus::SUCCESS) {
            if (task_state_.current_task->name() == "fire_preload") {
                increment_fire_count();
            }
            task_state_.current_task->finish_success();
            RCLCPP_INFO(
                logger_, "[DartManager] task '%s' SUCCESS",
                task_state_.current_task->name().c_str());
            task_state_.current_task.reset();
            if (task_state_.task_queue.empty()) {
                transition_to(ManagerLifecycleState::IDLE);
            }
        } else if (status == ActionStatus::FAILURE) {
            const auto failure = task_state_.current_task->failure_info();
            const std::string failed_action = failure.action_name.empty()
                                                ? task_state_.current_task->name()
                                                : failure.action_name;
            RCLCPP_ERROR(
                logger_,
                "[DartManager] task '%s' FAILURE at action '%s' reason='%s' -> state=ERROR",
                task_state_.current_task->name().c_str(), failed_action.c_str(),
                to_string(failure.reason));
            last_error_json_ = build_last_error_json(
                task_state_.current_task->name(), failed_action, failure.reason,
                task_state_.current_task->description());
            on_task_failure();
        }

        return status;
    }

    void on_task_failure() {
        if (!task_state_.current_task) {
            return;
        }

        task_state_.current_task->finish_failure();
        reset_task_state(task_state_);

        enter_belt_wait_zero_velocity_mode();
        *lifting_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        *limiting_command_ = rmcs_msgs::DartServoCommand::LOCK;

        transition_to(ManagerLifecycleState::ERROR);
    }

    void enter_belt_wait_zero_velocity_mode() {
        *belt_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
        *belt_target_velocity_ = 0.0;
        *belt_hold_torque_ = 0.0;
        *belt_wait_zero_velocity_ = true;
    }

    static void reset_task_state(TaskState& task_state) {
        task_state.current_task.reset();
        task_state.task_queue.clear();
        task_state.first_tick_of_task = true;
    }

    static void cancel_task_state(TaskState& task_state, ActionCancelReason reason) {
        task_state.task_queue.clear();
        if (task_state.current_task) {
            task_state.current_task->cancel(reason);
            task_state.current_task.reset();
        }
        task_state.first_tick_of_task = true;
    }

    void transition_to(ManagerLifecycleState new_state) {
        runtime_state_.lifecycle_state = new_state;
    }

    std::string active_task_name() const {
        return task_state_.current_task ? task_state_.current_task->name() : std::string{};
    }

    std::string active_action_name() const {
        if (!task_state_.current_task) {
            return {};
        }

        return task_state_.current_task->current_action_name();
    }

    std::string build_queue_json() const {
        std::ostringstream builder;
        builder << '[';

        bool first = true;
        for (const auto& task : task_state_.task_queue) {
            if (!task) {
                continue;
            }

            if (!first) {
                builder << ',';
            }

            const std::string display_name = task->description().empty() ? task->name() : task->description();
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

    void sync_debug_outputs() {
        *debug_lifecycle_state_output_ = to_string(runtime_state_.lifecycle_state);
        *debug_current_task_output_ = active_task_name();
        *debug_current_action_output_ = active_action_name();
        *debug_queue_json_output_ = build_queue_json();
        *debug_last_error_json_output_ = last_error_json_;
    }

    void reset_fire_count() {
        runtime_state_.fire_count = 0;
        *fire_count_output_ = runtime_state_.fire_count;
    }

    void increment_fire_count() {
        ++runtime_state_.fire_count;
        *fire_count_output_ = runtime_state_.fire_count;
        RCLCPP_INFO(logger_, "[DartManager] fire_count=%u", runtime_state_.fire_count);
    }

    ManagerInputContext input_context() {
        return ManagerInputContext{
            *belt_left_velocity_, *belt_right_velocity_,   *belt_left_torque_,
            *belt_right_torque_,  *lifting_left_velocity_, *lifting_right_velocity_,
        };
    }

    ManagerOutputContext output_context() {
        return ManagerOutputContext{
            *belt_command_,     *belt_target_velocity_,    *belt_torque_limit_,
            *belt_hold_torque_, *belt_wait_zero_velocity_, *trigger_lock_enable_,
            *lifting_command_,  *limiting_command_,
        };
    }

    ManagerSettings settings() const {
        return ManagerSettings{
            limiting_fill_ticks_,         lifting_stall_threshold_,
            lifting_stall_confirm_ticks_, lifting_stall_min_run_ticks_,
            lifting_stall_timeout_ticks_,
        };
    }

    rclcpp::Logger logger_;

    InputInterface<double> belt_left_velocity_;
    InputInterface<double> belt_right_velocity_;
    InputInterface<double> belt_left_torque_;
    InputInterface<double> belt_right_torque_;
    InputInterface<double> lifting_left_velocity_;
    InputInterface<double> lifting_right_velocity_;
    InputInterface<std::string> remote_command_input_;
    InputInterface<std::string> gui_command_input_;

    OutputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<double> belt_torque_limit_;
    OutputInterface<double> belt_hold_torque_;
    OutputInterface<bool> belt_wait_zero_velocity_;
    OutputInterface<bool> trigger_lock_enable_;
    OutputInterface<rmcs_msgs::DartMechanismCommand> lifting_command_;
    OutputInterface<rmcs_msgs::DartServoCommand> limiting_command_;
    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> debug_lifecycle_state_output_;
    OutputInterface<std::string> debug_current_task_output_;
    OutputInterface<std::string> debug_current_action_output_;
    OutputInterface<std::string> debug_queue_json_output_;
    OutputInterface<std::string> debug_last_error_json_output_;

    uint64_t limiting_fill_ticks_{500};
    double lifting_stall_threshold_{0.5};
    uint64_t lifting_stall_confirm_ticks_{100};
    uint64_t lifting_stall_min_run_ticks_{500};
    uint64_t lifting_stall_timeout_ticks_{5000};

    ManagerRuntimeState runtime_state_{};
    TaskState task_state_{};
    std::string last_error_json_;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
