#include "manager/action/action.hpp"
#include "manager/task/cancel_launch_task.hpp"
#include "manager/task/launch_preparation_task.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <rcl/publisher.h>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

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
        , logger_(get_logger())
        , last_heartbeat_time_(this->now() - rclcpp::Duration::from_seconds(10.0)) {

        register_input("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_input("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_input("/dart/force_screw/velocity", force_screw_velocity_);

        register_input("/dart/manager/command", command_input_, false);

        register_output("/dart/manager/belt/target_velocity", belt_target_velocity_, 0.0);
        register_output(
            "/dart/manager/force_screw/target_velocity", force_screw_target_velocity_, 0.0);
        register_output("/dart/manager/trigger/target_angle", trigger_target_angle_, 0.0);

        web_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/dart/manager/command", 10, [this](const std_msgs::msg::String::ConstSharedPtr msg) {
                std::lock_guard<std::mutex> lock(web_command_mutex_);
                web_command_ = msg->data;
            });

        web_heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/dart/manager/web_heartbeat", rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Empty::ConstSharedPtr /*msg*/) {
                std::lock_guard<std::mutex> lock(web_command_mutex_);
                last_heartbeat_time_ = this->now();
            });

        state_pub_ = create_publisher<std_msgs::msg::UInt8>("/dart/manager/state", 10);

        RCLCPP_INFO(logger_, "[DartManager] initialized, state=IDLE");
    }

    void update() override {
        check_web_connection();
        poll_command();

        switch (state_) {
        case State::IDLE: dispatch_next_task(); break;
        case State::RUNNING: tick_current_task(); break;
        case State::ERROR: break;
        }
    }

private:
    void check_web_connection() {
        std::lock_guard<std::mutex> lock(web_command_mutex_);
        if ((this->now() - last_heartbeat_time_).seconds() > 1.5) {
            if (web_command_enable_) {
                RCLCPP_WARN(logger_, "[DartManager] WebUI heartbeat timeout. Control revoked.");
                web_command_enable_ = false;
                web_command_.clear(); // 断开连接时清空遗留指令
            }
        } else {
            if (!web_command_enable_) {
                RCLCPP_INFO(logger_, "[DartManager] WebUI heartbeat restored. Control granted.");
                web_command_enable_ = true;
            }
        }
    }

    // 命令轮询
    void poll_command() {
        std::string cmd;

        {
            std::lock_guard<std::mutex> lock(web_command_mutex_);
            if (web_command_enable_ && !web_command_.empty()) {
                cmd = web_command_;
                web_command_.clear();
            }
        }

        if (cmd.empty() && command_input_.ready()) {
            cmd = *command_input_;
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

        *belt_target_velocity_ = 0.0;
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
        *belt_target_velocity_ = 0.0;
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
                *belt_target_velocity_, *left_belt_velocity_, *right_belt_velocity_,
                *trigger_target_angle_);
        }
        if (cmd == "unload" || cmd == "cancel_launch") {
            return std::make_shared<CancelLaunchTask>(
                *belt_target_velocity_, *left_belt_velocity_, *right_belt_velocity_,
                *trigger_target_angle_);
        }
        // if (cmd == "fire")   return std::make_shared<FireTask>(...);
        return nullptr;
    }

    rclcpp::Logger logger_;

    InputInterface<double> left_belt_velocity_;
    InputInterface<double> right_belt_velocity_;
    InputInterface<double> force_screw_velocity_;

    OutputInterface<double> belt_target_velocity_;
    OutputInterface<double> force_screw_target_velocity_;
    OutputInterface<double> trigger_target_angle_;

    InputInterface<std::string> command_input_;
    std::string last_command_;

    bool web_command_enable_ = false;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr web_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr web_heartbeat_sub_;
    std::string web_command_;
    std::mutex web_command_mutex_;
    rclcpp::Time last_heartbeat_time_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

    State state_{State::IDLE};

    std::shared_ptr<Task> current_task_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    bool first_tick_of_task_ = true;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManager, rmcs_executor::Component)
