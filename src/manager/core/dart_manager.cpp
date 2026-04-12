#include "manager/core/runtime/task.hpp"
#include "manager/resources/task_factory.hpp"
#include "rmcs_msgs/dart_motor_exit_mode.hpp"

#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>
#include <sys/types.h>

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
        register_input("/dart/drive_belt/left/velocity", belt_left_velocity_);
        register_input("/dart/drive_belt/left/torque", belt_left_torque_);
        register_input("/dart/drive_belt/right/velocity", belt_right_velocity_);
        register_input("/dart/drive_belt/right/torque", belt_right_torque_);

        belt_down_velocity_ = get_parameter("belt_down_velocity").as_double();
        belt_up_velocity_ = get_parameter("belt_up_velocity").as_double();
        manual_belt_velocity_ = get_parameter("manual_max_velocity").as_double();
        belt_stall_velocity_threshold_ = get_parameter("belt_stall_velocity_threshold").as_double();
        belt_stall_torque_threshold_ = get_parameter("belt_stall_torque_threshold").as_double();
        belt_stall_confirm_ticks_ =
            static_cast<uint64_t>(get_parameter("belt_stall_confirm_ticks").as_int());

        // lift
        register_output("/dart_manager/lift/command", lift_command_);
        register_output("/dart_manager/lift/target_velocity", lift_target_velocity_);
        register_output("/dart_manager/lift/exit_mode", lift_exit_mode_);
        register_input("/dart/lifting_left/velocity", lift_left_velocity_);
        register_input("/dart/lifting_left/torque", lift_left_torque_);
        register_input("/dart/lifting_right/velocity", lift_right_velocity_);
        register_input("/dart/lifting_right/torque", lift_right_torque_);

        lift_velocity_ = get_parameter("lift_velocity").as_double();
        lift_stall_velocity_threshold_ = get_parameter("lifting_stall_threshold").as_double();
        lift_stall_torque_threshold_ = get_parameter("lifting_stall_torque_threshold").as_double();
        lift_stall_confirm_ticks_ =
            static_cast<uint64_t>(get_parameter("lifting_stall_confirm_ticks").as_int());
        lift_stall_min_run_ticks_ =
            static_cast<uint64_t>(get_parameter("lifting_stall_min_run_ticks").as_int());

        // trigger
        register_output("/dart_manager/trigger/command", trigger_command_);

        // limit servo
        register_output("/dart_manager/limit_servo/command", limiting_command_);

        limiting_fill_ticks_ = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        // yaw pitch force
        register_output("/dart_manager/force/error", force_error_, int32_t{0});
        register_output(
            "/dart_manager/angle/error_vector", angle_error_vector_, Eigen::Vector2d::Zero());

        register_input("/force_sensor/channel_1/weight", force_sensor_ch1_);
        register_input("/force_sensor/channel_2/weight", force_sensor_ch2_);

        force_setpoint_ = static_cast<int32_t>(get_parameter("force_setpoint").as_int());
        force_allowable_error_ =
            static_cast<int32_t>(get_parameter("force_allowable_error").as_int());

        // manual control
        register_input("/remote/switch/left", remote_left_switch_, false);
        register_input("/remote/switch/right", remote_right_switch_, false);
        register_input("/remote/joystick/left", remote_left_joystick_, false);
        register_input("/remote/joystick/right", remote_right_joystick_, false);

        manual_angle_max_error_ = get_parameter("max_transform_rate").as_double();
        manual_force_max_error_ =
            static_cast<int32_t>(std::lround(get_parameter("manual_force_scale").as_double()));

        // command/debug io
        register_input("/dart/manager/command", command_input_, false);
        register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});
        register_output(
            "/dart/manager/debug/lifecycle_state", debug_lifecycle_state_output_,
            std::string{to_string(ManagerLifecycleState::IDLE)});
        register_output(
            "/dart/manager/debug/current_task", debug_current_task_output_, std::string{});
        register_output(
            "/dart/manager/debug/current_action", debug_current_action_output_, std::string{});

        reset_fire_count();
        sync_debug_outputs();
        RCLCPP_INFO(logger_, "[DartManager] initialized");
    }

    void before_updating() override {
        bind_optional_command_input();
        bind_optional_manual_inputs();

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

    void bind_optional_command_input() {
        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(logger_, "Failed to fetch \"/dart/manager/command\". Set to empty string.");
        }
    }

    void bind_optional_manual_inputs() {
        if (!remote_left_switch_.ready()) {
            remote_left_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }

        if (!remote_right_switch_.ready()) {
            remote_right_switch_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }

        if (!remote_left_joystick_.ready()) {
            remote_left_joystick_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/joystick/left\". Set to zero.");
        }

        if (!remote_right_joystick_.ready()) {
            remote_right_joystick_.make_and_bind_directly(Eigen::Vector2d::Zero());
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/joystick/right\". Set to zero.");
        }
    }

    void poll_command() {
        const std::string cmd = poll_new_command();
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

    std::string poll_new_command() {
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

    void cancel_all() {
        cancel_task_state(task_state_, ActionCancelReason::EXTERNAL_CANCEL);
        enter_belt_wait_zero_velocity_mode();
        *lift_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
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
        *lift_command_ = rmcs_msgs::DartMechanismCommand::WAIT;
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
        *fire_count_output_ = runtime_state_.fire_count;
        *debug_lifecycle_state_output_ = to_string(runtime_state_.lifecycle_state);
        *debug_current_task_output_ = active_task_name();
        *debug_current_action_output_ = active_action_name();
    }

    void reset_fire_count() { runtime_state_.fire_count = 0; }

    void increment_fire_count() {
        ++runtime_state_.fire_count;
        RCLCPP_INFO(logger_, "[DartManager] fire_count=%u", runtime_state_.fire_count);
    }

    ManagerInputContext input_context() {
        return ManagerInputContext{
            *belt_left_velocity_,           //
            *belt_left_torque_,             //
            *belt_right_velocity_,          //
            *belt_right_torque_,            //
            *lift_left_velocity_,           //
            *lift_left_torque_,             //
            *lift_right_velocity_,          //
            *lift_right_torque_,            //
            *force_sensor_ch1_,             //
            *force_sensor_ch2_,             //
            *remote_left_switch_,           //
            *remote_right_switch_,          //
            *remote_left_joystick_,         //
            *remote_right_joystick_,        //
        };
    }

    ManagerOutputContext output_context() {
        return ManagerOutputContext{
            *belt_command_,                 //
            *belt_target_velocity_,         //
            *belt_exit_mode_,               //
            *lift_command_,                 //
            *lift_target_velocity_,         //
            *lift_exit_mode_,               //
            *trigger_command_,              //
            *limiting_command_,             //
            *force_error_,                  //
            *angle_error_vector_,           //
        };
    }

    ManagerSettings settings() const {
        return ManagerSettings{
            belt_down_velocity_,            //
            belt_up_velocity_,              //
            belt_stall_velocity_threshold_, //
            belt_stall_torque_threshold_,   //
            belt_stall_confirm_ticks_,      //
            manual_belt_velocity_,          //
            lift_velocity_,                 //
            lift_stall_velocity_threshold_, //
            lift_stall_torque_threshold_,   //
            lift_stall_confirm_ticks_,      //
            limiting_fill_ticks_,           //
            force_setpoint_,                //
            force_allowable_error_,         //
            manual_angle_max_error_,        //
            manual_force_max_error_,        //
        };
    }

    rclcpp::Logger logger_;

    // belt
    OutputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<rmcs_msgs::ExitMode> belt_exit_mode_;
    InputInterface<double> belt_left_velocity_;
    InputInterface<double> belt_left_torque_;
    InputInterface<double> belt_right_velocity_;
    InputInterface<double> belt_right_torque_;

    double belt_down_velocity_;
    double belt_up_velocity_;
    double belt_stall_velocity_threshold_;
    double belt_stall_torque_threshold_;
    uint64_t belt_stall_confirm_ticks_;

    // lift
    OutputInterface<rmcs_msgs::DartMechanismCommand> lift_command_;
    OutputInterface<double> lift_target_velocity_;
    OutputInterface<rmcs_msgs::ExitMode> lift_exit_mode_;
    InputInterface<double> lift_left_velocity_;
    InputInterface<double> lift_left_torque_;
    InputInterface<double> lift_right_velocity_;
    InputInterface<double> lift_right_torque_;

    double lift_velocity_;
    double lift_stall_velocity_threshold_;
    double lift_stall_torque_threshold_;
    uint64_t lift_stall_confirm_ticks_;
    uint64_t lift_stall_min_run_ticks_;

    // trigger
    OutputInterface<rmcs_msgs::DartServoCommand> trigger_command_;

    // limit servo
    OutputInterface<rmcs_msgs::DartServoCommand> limiting_command_;

    uint64_t limiting_fill_ticks_;

    // yaw pitch force
    OutputInterface<int32_t> force_error_;
    OutputInterface<Eigen::Vector2d> angle_error_vector_;

    InputInterface<int32_t> force_sensor_ch1_;
    InputInterface<int32_t> force_sensor_ch2_;

    int32_t force_setpoint_;
    int32_t force_allowable_error_;

    // manual control
    InputInterface<rmcs_msgs::Switch> remote_left_switch_;
    InputInterface<rmcs_msgs::Switch> remote_right_switch_;
    InputInterface<Eigen::Vector2d> remote_left_joystick_;
    InputInterface<Eigen::Vector2d> remote_right_joystick_;

    double manual_belt_velocity_;
    int32_t manual_force_max_error_;
    double manual_angle_max_error_;

    // command & status
    InputInterface<std::string> command_input_;

    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> debug_lifecycle_state_output_;
    OutputInterface<std::string> debug_current_task_output_;
    OutputInterface<std::string> debug_current_action_output_;

    std::string last_command_;

    ManagerRuntimeState runtime_state_{};
    TaskState task_state_{};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
