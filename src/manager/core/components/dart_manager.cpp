#include "manager/core/components/dart_manager_bridge_io.hpp"
#include "manager/core/runtime/task.hpp"
#include "manager/resources/task_factory.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"

#include <cstdint>
#include <deque>
#include <memory>
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

        // belt
        register_output("/dart_manager/belt/command", belt_command_);
        register_output("/dart_manager/belt/target_velocity", belt_target_velocity_);
        register_output("/dart_manager/belt/exit_mode", belt_exit_mode_);

        register_input("/dart_manager/belt/arrive_flag", belt_arrive_flag_);

        belt_down_velocity_ = get_parameter("belt_down_velocity").as_double();
        belt_up_velocity_ = get_parameter("belt_up_velocity").as_double();

        // lift
        register_output("/dart_manager/belt/command", belt_command_);
        register_output("/dart_manager/belt/target_velocity", belt_target_velocity_);
        register_output("/dart_manager/belt/exit_mode", belt_exit_mode_);

        register_input("/dart_manager/belt/arrive_flag", belt_arrive_flag_);

        lift_velocity_ = get_parameter("lift_velocity").as_double();

        // trigger
        register_output("/dart_manager/trigger/command", trigger_command_);

        // limit servo
        register_output("/dart_manager/limit_servo/command", limiting_command_);

        limiting_fill_ticks_ = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        bridge_io_.register_interfaces(*this);

        reset_fire_count();
        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] initialized");
    }

    void before_updating() override {
        bridge_io_.bind_optional_inputs(logger_);

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
        const std::string cmd = bridge_io_.poll_command();
        if (cmd.empty()) {
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

        bridge_io_.clear_last_error();
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
            bridge_io_.set_last_error(
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

    void sync_debug_outputs() {
        bridge_io_.sync_debug_outputs(
            runtime_state_, active_task_name(), active_action_name(), task_state_.task_queue);
    }

    void reset_fire_count() { runtime_state_.fire_count = 0; }

    void increment_fire_count() {
        ++runtime_state_.fire_count;
        RCLCPP_INFO(logger_, "[DartManager] fire_count=%u", runtime_state_.fire_count);
    }

    ManagerInputContext input_context() {
        return ManagerInputContext{
            *belt_arrive_flag_,     //
            *lift_arrive_flag,      //
        };
    }

    ManagerOutputContext output_context() {
        return ManagerOutputContext{
            *belt_command_,         //
            *belt_target_velocity_, //
            *belt_exit_mode_,       //
            *lifting_command_,      //
            *lift_target_velocity_, //
            *lift_exit_mode_,       //
            *trigger_command_,      //
            *limiting_command_,     //
        };
    }

    ManagerSettings settings() const {
        return ManagerSettings{
            belt_down_velocity_,    //
            belt_up_velocity_,      //
            lift_velocity_,         //
            limiting_fill_ticks_,   //
        };
    }

    rclcpp::Logger logger_;

    // belt
    OutputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<rmcs_msgs::ExitMode> belt_exit_mode_;

    InputInterface<bool> belt_arrive_flag_;

    double belt_down_velocity_;
    double belt_up_velocity_;

    // lift
    OutputInterface<rmcs_msgs::DartMechanismCommand> lifting_command_;
    OutputInterface<double> lift_target_velocity_;
    OutputInterface<rmcs_msgs::ExitMode> lift_exit_mode_;

    InputInterface<bool> lift_arrive_flag;

    double lift_velocity_;

    // trigger
    OutputInterface<rmcs_msgs::DartServoCommand> trigger_command_;

    // limit servo
    OutputInterface<rmcs_msgs::DartServoCommand> limiting_command_;

    uint64_t limiting_fill_ticks_;

    ManagerRuntimeState runtime_state_{};
    TaskState task_state_{};

    DartManagerBridgeIo bridge_io_;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
