#include "manager/action/action.hpp"
#include "manager/action/manual_angle_control.hpp"
#include "manager/action/manual_force_control.hpp"
#include "manager/task/cancel_launch_task.hpp"
#include "manager/task/fire_task.hpp"
#include "manager/task/launch_preparation_task.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <rcl/publisher.h>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

class DartManager
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class State : uint8_t {
        IDLE = 0,
        RUNNING = 1,
        ERROR = 2,
    };

    DartManager()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        register_input("/dart/drive_belt/left/velocity",  left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/dart/drive_belt/left/torque",   left_belt_torque_);
        register_input("/dart/drive_belt/right/torque",  right_belt_torque_);
        register_input("/dart/force_screw/velocity", force_screw_velocity_);

        // 遥控器与 WebUI 命令输入
        register_input("/dart/manager/command", remote_command_input_, false);
        register_input("/dart/manager/web_command", web_command_input_, false);

        // 手动控制所需摇杆输入
        register_input("/remote/joystick/left",  joystick_left_,  false);
        register_input("/remote/joystick/right", joystick_right_, false);

        register_output("/dart/manager/belt/command", belt_command_, rmcs_msgs::DartSliderStatus::WAIT);
        register_output(
            "/dart/manager/force_screw/target_velocity", force_screw_target_velocity_, 0.0);
        register_output("/dart/manager/trigger/lock_enable", trigger_lock_enable_, false);

        // 手动角度 / 力螺母速度指令输出（供下游 PID controller 用作 setpoint）
        register_output("/pitch/control/velocity", yaw_pitch_control_velocity_);
        register_output("/force/control/velocity", force_control_velocity_, 0.0);

        // 从 yaml 读取手动控制参数
        try {
            max_transform_rate_ = get_parameter("max_transform_rate").as_double();
        } catch (...) {
            max_transform_rate_ = 500.0;
        }
        try {
            manual_force_scale_ = get_parameter("manual_force_scale").as_double();
        } catch (...) {
            manual_force_scale_ = 5.0;
        }

        state_pub_ = create_publisher<std_msgs::msg::UInt8>("/dart/manager/state", 10);

        RCLCPP_INFO(logger_, "[DartManager] initialized, state=IDLE");
    }

    void update() override {
        poll_command();

        switch (state_) {
        case State::IDLE: dispatch_next_task(); break;
        case State::RUNNING: tick_current_task(); break;
        case State::ERROR: break;
        }
    }

private:
    // 命令轮询
    void poll_command() {
        std::string cmd;

        // 优先读取 WebUI 命令，其次是遥控器命令
        if (web_command_input_.ready() && !web_command_input_->empty()) {
            cmd = *web_command_input_;
        } else if (remote_command_input_.ready()) {
            cmd = *remote_command_input_;
        }

        if (cmd.empty()) {
            last_command_.clear();
            return;
        }

        if (cmd == last_command_)
            return;

        last_command_ = cmd;

        if (cmd == "cancel") {
            cancel_all();
        } else if (cmd == "recover") {
            recover();
        } else {
            auto task = make_task(cmd);
            RCLCPP_INFO(logger_, "[DartManager] received command: '%s'", cmd.c_str());
            if (task) {
                submit_task(std::move(task));

            } else {
                RCLCPP_WARN(logger_, "[DartManager] unknown command: '%s'", cmd.c_str());
            }
        }
    }

    // 提交任务到队尾
    void submit_task(std::shared_ptr<Task> task) {
        task_queue_.push_back(std::move(task));
        RCLCPP_INFO(
            logger_, "[DartManager] task queued: %s (queue size=%zu)",
            task_queue_.back()->name().c_str(), task_queue_.size());
    }

    // 清空队列并取消当前任务
    void cancel_all() {
        task_queue_.clear();
        if (current_task_) {
            current_task_->cancel();
            current_task_.reset();
            RCLCPP_WARN(logger_, "[DartManager] all tasks cancelled");
        }

        *belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *force_screw_target_velocity_ = 0.0;

        transition_to(State::IDLE);
    }

    // 状态恢复
    void recover() {
        if (state_ == State::ERROR) {
            current_task_.reset();
            RCLCPP_INFO(logger_, "[DartManager] recovered from ERROR, state=IDLE");
            transition_to(State::IDLE);
        }
    }

    // 从队列取出下一个任务开始执行
    void dispatch_next_task() {
        if (task_queue_.empty())
            return;

        current_task_ = std::move(task_queue_.front());
        task_queue_.pop_front();

        RCLCPP_INFO(logger_, "[DartManager] dispatching task: '%s'", current_task_->name().c_str());
        transition_to(State::RUNNING);

        tick_current_task();
    }

    // tick 第一帧
    void tick_current_task() {
        if (!current_task_)
            return;

        // 区分首帧和后续帧
        ActionStatus status =
            first_tick_of_task_ ? current_task_->tick_first() : current_task_->tick();
        first_tick_of_task_ = false;

        if (status == ActionStatus::SUCCESS) {
            current_task_->on_exit();
            RCLCPP_INFO(logger_, "[DartManager] task '%s' SUCCESS", current_task_->name().c_str());
            current_task_.reset();
            transition_to(State::IDLE);

        } else if (status == ActionStatus::FAILURE) {
            RCLCPP_ERROR(
                logger_, "[DartManager] task '%s' FAILURE → state=ERROR",
                current_task_->name().c_str());
            on_task_failure();
        }
        // RUNNING: 什么都不做，等下一帧
    }

    // 失败处理
    void on_task_failure() {
        *belt_command_ = rmcs_msgs::DartSliderStatus::WAIT;
        *force_screw_target_velocity_ = 0.0;

        current_task_->on_exit();
        current_task_.reset();
        task_queue_.clear(); // 清空后续队列

        transition_to(State::ERROR);
    }

    // 状态转换
    void transition_to(State new_state) {
        state_ = new_state;
        first_tick_of_task_ = true;

        if (state_pub_) {
            std_msgs::msg::UInt8 msg;
            msg.data = static_cast<uint8_t>(new_state);
            state_pub_->publish(msg);
        }
    }

    // 任务工厂
    // 新增任务时新建独立的 Task 类 hpp 文件和 if-else
    std::shared_ptr<Task> make_task(const std::string& cmd) {
        if (cmd == "launch_prepare") {
            return std::make_shared<LaunchPreparationTask>(
                *belt_command_,
                *left_belt_velocity_,  *right_belt_velocity_,
                *left_belt_torque_,    *right_belt_torque_,
                *trigger_lock_enable_);
        }
        if (cmd == "unload" || cmd == "cancel_launch") {
            return std::make_shared<CancelLaunchTask>(
                *belt_command_,
                *left_belt_velocity_,  *right_belt_velocity_,
                *left_belt_torque_,    *right_belt_torque_,
                *trigger_lock_enable_);
        }
        if (cmd == "fire") {
            return std::make_shared<FireTask>(*trigger_lock_enable_);
        }
        if (cmd == "manual_angle") {
            auto task = std::make_shared<Task>("manual_angle", "手动 yaw/pitch 调整");
            task->then(std::make_shared<DartManualAngleControlAction>(
                (*yaw_pitch_control_velocity_)[0], (*yaw_pitch_control_velocity_)[1],
                *joystick_left_, *joystick_right_,
                max_transform_rate_));
            return task;
        }
        if (cmd == "manual_force") {
            auto task = std::make_shared<Task>("manual_force", "手动力丝杆速度调整");
            task->then(std::make_shared<DartManualForceControlAction>(
                *force_control_velocity_,
                *joystick_right_,
                max_transform_rate_,
                manual_force_scale_));
            return task;
        }
        return nullptr;
    }

    rclcpp::Logger logger_;

    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<double> left_belt_torque_;
    InputInterface<double> right_belt_torque_;
    InputInterface<double> force_screw_velocity_;

    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    OutputInterface<rmcs_msgs::DartSliderStatus> belt_command_;
    OutputInterface<double> force_screw_target_velocity_;
    OutputInterface<bool> trigger_lock_enable_;

    OutputInterface<Eigen::Vector2d> yaw_pitch_control_velocity_;
    OutputInterface<double> force_control_velocity_;

    double max_transform_rate_{500.0};
    double manual_force_scale_{5.0};

    InputInterface<std::string> remote_command_input_;
    InputInterface<std::string> web_command_input_;
    std::string last_command_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

    State state_{State::IDLE};

    std::shared_ptr<Task> current_task_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    bool first_tick_of_task_ = true;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
