#include "manager/core/runtime/task.hpp"
#include "manager/resources/task_factory.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include <rmcs_msgs/dart_limiting_servo_status.hpp>
#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class DartManagerV2
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManagerV2()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/dart/drive_belt/left/torque", left_belt_torque_);
        register_input("/dart/drive_belt/right/torque", right_belt_torque_);
        register_input("/dart/lifting_left/velocity", lifting_left_vel_fb_);
        register_input("/dart/lifting_right/velocity", lifting_right_vel_fb_);

        register_input("/dart/manager/command", remote_command_input_, false);

        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/joystick/right", joystick_right_, false);

        register_output(
            "/dart/manager/belt/command", belt_command_, rmcs_msgs::DartMotorStatus::WAIT);
        register_output("/dart/manager/belt/target_velocity", belt_target_velocity_, 0.0);
        register_output("/dart/manager/belt/torque_limit", belt_torque_limit_, 0.0);
        register_output("/dart/manager/belt/hold_torque", belt_hold_torque_, 0.0);
        register_output("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_output("/dart/manager/trigger/lock_enable", trigger_lock_enable_, false);

        register_output(
            "/pitch/control/velocity", yaw_pitch_control_velocity_, Eigen::Vector2d::Zero());
        register_output("/force/control/velocity", force_control_velocity_, 0.0);

        // 升降指令总线
        register_output(
            "/dart/manager/lifting/command", lifting_command_, rmcs_msgs::DartMotorStatus::WAIT);
        register_output(
            "/dart/manager/limiting/command", limiting_command_, rmcs_msgs::DartServoStatus::LOCK);
        register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});

        max_transform_rate_ = get_parameter("max_transform_rate").as_double();
        manual_force_scale_ = get_parameter("manual_force_scale").as_double();

        limiting_fill_ticks_ = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        lifting_stall_threshold_ = get_parameter("lifting_stall_threshold").as_double();
        lifting_stall_confirm_ticks_ =
            (uint64_t)get_parameter("lifting_stall_confirm_ticks").as_int();
        lifting_stall_min_run_ticks_ =
            (uint64_t)get_parameter("lifting_stall_min_run_ticks").as_int();
        lifting_stall_timeout_ticks_ =
            (uint64_t)get_parameter("lifting_stall_timeout_ticks").as_int();

        reset_fire_count();
        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        submit_task(make_slider_init_task(input, output, manager_settings));
        RCLCPP_INFO(logger_, "[DartManagerV2] initialized, queued SliderInitTask on startup");
    }

    void update() override {
        poll_command();

        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            return;
        }

        while (runtime_state_.lifecycle_state != ManagerLifecycleState::ERROR) {
            if (!task_slot_state_.current_task) {
                if (task_slot_state_.task_queue.empty()) {
                    if (runtime_state_.lifecycle_state != ManagerLifecycleState::IDLE) {
                        transition_to(ManagerLifecycleState::IDLE);
                    }
                    break;
                }
                dispatch_next_task(ManagerTaskSlot::PRIMARY);
            }

            if (!task_slot_state_.current_task) {
                break;
            }

            const ActionStatus status = tick_current_task(ManagerTaskSlot::PRIMARY);
            if (status == ActionStatus::RUNNING
                || runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
                break;
            }
        }
    }

private:
    void poll_command() {
        std::string cmd = *remote_command_input_;

        if (cmd.empty()) {
            runtime_state_.last_command.clear();
            return;
        }

        if (cmd == runtime_state_.last_command)
            return;

        if (cmd == "cancel") {
            cancel_all();
        } else if (cmd == "recover") {
            recover();
        } else {
            auto input = input_context();
            auto output = output_context();
            auto manager_settings = settings();
            auto task_spec = make_task(cmd, input, output, manager_settings, runtime_state_);
            RCLCPP_INFO(logger_, "[DartManagerV2] received command: '%s'", cmd.c_str());
            if (task_spec) {
                submit_task(std::move(*task_spec));
            } else {
                RCLCPP_WARN(logger_, "[DartManagerV2] unknown command: '%s'", cmd.c_str());
            }
        }

        runtime_state_.last_command = cmd;
    }

    void cancel_all() {
        task_slot_state_.task_queue.clear();
        if (task_slot_state_.current_task) {
            task_slot_state_.current_task->cancel(ActionCancelReason::EXTERNAL_CANCEL);
            task_slot_state_.current_task.reset();
            RCLCPP_WARN(logger_, "[DartManagerV2] all tasks cancelled");
        }
        task_slot_state_.first_tick_of_task = true;

        enter_belt_wait_zero_velocity_mode();
        *lifting_command_ = rmcs_msgs::DartMotorStatus::WAIT;
        *limiting_command_ = rmcs_msgs::DartServoStatus::LOCK;
        *yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        *force_control_velocity_ = 0.0;

        transition_to(ManagerLifecycleState::IDLE);
    }

    void recover() {
        if (runtime_state_.lifecycle_state == ManagerLifecycleState::ERROR) {
            task_slot_state_.current_task.reset();
            task_slot_state_.task_queue.clear();
            task_slot_state_.first_tick_of_task = true;
            RCLCPP_INFO(logger_, "[DartManagerV2] recovered from ERROR, state=IDLE");
            transition_to(ManagerLifecycleState::IDLE);
        }
        reset_fire_count();
        *limiting_command_ = rmcs_msgs::DartServoStatus::LOCK;
        // 无论 ERROR 还是 IDLE，都重新排队传送带复位
        auto input = input_context();
        auto output = output_context();
        auto manager_settings = settings();
        submit_task(make_slider_init_task(input, output, manager_settings));
        RCLCPP_INFO(logger_, "[DartManagerV2] queued SliderInitTask for recovery");
    }

    void submit_task(ManagerTaskSpec task_spec) {
        if (!task_spec.task) {
            return;
        }

        auto* slot_state = slot_state_ptr(task_spec.slot);
        if (slot_state == nullptr) {
            RCLCPP_WARN(
                logger_, "[DartManagerV2] task '%s' requested unsupported slot '%s'",
                task_spec.task->name().c_str(), to_string(task_spec.slot));
            return;
        }

        slot_state->task_queue.push_back(std::move(task_spec.task));
        RCLCPP_INFO(
            logger_, "[DartManagerV2] task queued: %s (slot=%s, queue size=%zu)",
            slot_state->task_queue.back()->name().c_str(), to_string(task_spec.slot),
            slot_state->task_queue.size());
    }

    void dispatch_next_task(ManagerTaskSlot slot) {
        auto* slot_state = slot_state_ptr(slot);
        if (slot_state == nullptr || slot_state->task_queue.empty())
            return;

        slot_state->current_task = std::move(slot_state->task_queue.front());
        slot_state->task_queue.pop_front();
        slot_state->current_task->bind_runtime_context(
            ActionRuntimeContext{slot_state->current_task->name(), &logger_});

        RCLCPP_INFO(
            logger_, "[DartManagerV2] dispatching task: '%s' (slot=%s)",
            slot_state->current_task->name().c_str(), to_string(slot));
        slot_state->first_tick_of_task = true;
        if (runtime_state_.lifecycle_state != ManagerLifecycleState::RUNNING) {
            transition_to(ManagerLifecycleState::RUNNING);
        }
    }

    ActionStatus tick_current_task(ManagerTaskSlot slot) {
        auto* slot_state = slot_state_ptr(slot);
        if (slot_state == nullptr || !slot_state->current_task)
            return ActionStatus::SUCCESS;

        const ActionStatus status = slot_state->first_tick_of_task
                                      ? slot_state->current_task->tick_first()
                                      : slot_state->current_task->tick();
        slot_state->first_tick_of_task = false;

        if (status == ActionStatus::SUCCESS) {
            if (slot_state->current_task->name() == "fire_preload") {
                increment_fire_count();
            }
            slot_state->current_task->finish_success();
            RCLCPP_INFO(
                logger_, "[DartManagerV2] task '%s' SUCCESS (slot=%s)",
                slot_state->current_task->name().c_str(), to_string(slot));
            slot_state->current_task.reset();
            if (slot == ManagerTaskSlot::PRIMARY && slot_state->task_queue.empty()) {
                transition_to(ManagerLifecycleState::IDLE);
            }
        } else if (status == ActionStatus::FAILURE) {
            const auto failure = slot_state->current_task->failure_info();
            const std::string failed_action = failure.action_name.empty()
                                                ? slot_state->current_task->name()
                                                : failure.action_name;
            RCLCPP_ERROR(
                logger_,
                "[DartManagerV2] task '%s' FAILURE at action '%s' reason='%s' (slot=%s) -> "
                "state=ERROR",
                slot_state->current_task->name().c_str(), failed_action.c_str(),
                to_string(failure.reason), to_string(slot));
            on_task_failure(slot);
        }

        return status;
    }

    void on_task_failure(ManagerTaskSlot slot) {
        auto* slot_state = slot_state_ptr(slot);
        if (slot_state == nullptr || !slot_state->current_task) {
            return;
        }

        slot_state->current_task->finish_failure();
        slot_state->current_task.reset();
        slot_state->task_queue.clear();
        slot_state->first_tick_of_task = true;

        enter_belt_wait_zero_velocity_mode();
        *lifting_command_ = rmcs_msgs::DartMotorStatus::WAIT;
        *limiting_command_ = rmcs_msgs::DartServoStatus::LOCK;
        *yaw_pitch_control_velocity_ = Eigen::Vector2d::Zero();
        *force_control_velocity_ = 0.0;

        transition_to(ManagerLifecycleState::ERROR);
    }

    void enter_belt_wait_zero_velocity_mode() {
        *belt_command_ = rmcs_msgs::DartMotorStatus::WAIT;
        *belt_target_velocity_ = 0.0;
        *belt_hold_torque_ = 0.0;
        *belt_wait_zero_velocity_ = true;
    }

    void transition_to(ManagerLifecycleState new_state) {
        runtime_state_.lifecycle_state = new_state;
    }

    void reset_fire_count() {
        runtime_state_.fire_count = 0;
        *fire_count_output_ = runtime_state_.fire_count;
    }

    void increment_fire_count() {
        ++runtime_state_.fire_count;
        *fire_count_output_ = runtime_state_.fire_count;
        RCLCPP_INFO(logger_, "[DartManagerV2] fire_count=%u", runtime_state_.fire_count);
    }

    ManagerInputContext input_context() {
        return ManagerInputContext{
            *left_belt_velocity_,  *right_belt_velocity_,  *left_belt_torque_,
            *right_belt_torque_,   *joystick_left_,        *joystick_right_,
            *lifting_left_vel_fb_, *lifting_right_vel_fb_,
        };
    }

    ManagerOutputContext output_context() {
        return ManagerOutputContext{
            *belt_command_,
            *belt_target_velocity_,
            *belt_torque_limit_,
            *belt_hold_torque_,
            *belt_wait_zero_velocity_,
            *trigger_lock_enable_,
            *yaw_pitch_control_velocity_,
            *force_control_velocity_,
            *lifting_command_,
            *limiting_command_,
        };
    }

    ManagerSettings settings() const {
        return ManagerSettings{
            max_transform_rate_,          manual_force_scale_,
            limiting_fill_ticks_,         lifting_stall_threshold_,
            lifting_stall_confirm_ticks_, lifting_stall_min_run_ticks_,
            lifting_stall_timeout_ticks_,
        };
    }

    struct TaskSlotState {
        std::shared_ptr<Task> current_task;
        std::deque<std::shared_ptr<Task>> task_queue;
        bool first_tick_of_task{true};
    };

    TaskSlotState* slot_state_ptr(ManagerTaskSlot slot) {
        if (slot == ManagerTaskSlot::PRIMARY) {
            return &task_slot_state_;
        }
        return nullptr;
    }

    rclcpp::Logger logger_;

    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<double> left_belt_torque_;
    InputInterface<double> right_belt_torque_;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    InputInterface<double> lifting_left_vel_fb_;
    InputInterface<double> lifting_right_vel_fb_;

    OutputInterface<rmcs_msgs::DartMotorStatus> belt_command_;
    OutputInterface<double> belt_target_velocity_;
    OutputInterface<double> belt_torque_limit_;
    OutputInterface<double> belt_hold_torque_;
    OutputInterface<bool> belt_wait_zero_velocity_;
    OutputInterface<bool> trigger_lock_enable_;

    OutputInterface<Eigen::Vector2d> yaw_pitch_control_velocity_;
    OutputInterface<double> force_control_velocity_;

    OutputInterface<rmcs_msgs::DartMotorStatus> lifting_command_;
    OutputInterface<rmcs_msgs::DartServoStatus> limiting_command_;
    OutputInterface<uint32_t> fire_count_output_;

    double max_transform_rate_{500.0};
    double manual_force_scale_{5.0};
    uint64_t limiting_fill_ticks_{500};

    double lifting_stall_threshold_{0.5};
    uint64_t lifting_stall_confirm_ticks_{100};
    uint64_t lifting_stall_min_run_ticks_{500};
    uint64_t lifting_stall_timeout_ticks_{5000};

    InputInterface<std::string> remote_command_input_;

    ManagerRuntimeState runtime_state_{};
    TaskSlotState task_slot_state_{};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManagerV2, rmcs_executor::Component)
