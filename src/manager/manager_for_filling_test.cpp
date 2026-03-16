#include "manager/action/action.hpp"
#include "manager/task/filling_test_task.hpp"
#include "manager/task/task.hpp"

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <rmcs_msgs/dart_slider_status.hpp>

namespace rmcs_dart_guidance::manager {

// DartManagerV2 — 在 DartManager 基础上增加：
//   · shot_count_ (1~4) 发次计数，fire 成功后递增，超4归1
//   · /dart/manager/lifting/command 输出，供 DartLaunchSettingV2 转换为速度
//   · /dart/limiting_servo/control_angle 输出，由 FireTask2 直接写值
//   · 第2~4发使用 LaunchPreparationTask2 / CancelLaunchTask2 / FireTask2
//   · 升降堵转检测在 LiftingLkAction 内完成（直接读速度反馈，无循环依赖）
class DartManagerForFillingTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    enum class State : uint8_t {
        IDLE    = 0,
        RUNNING = 1,
        ERROR   = 2,
    };

    DartManagerForFillingTest()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/manager/command",     remote_command_input_, false);
        register_input("/dart/manager/web_command", web_command_input_,    false);

        register_input("/remote/joystick/left",  joystick_left_,  false);
        register_input("/remote/joystick/right", joystick_right_, false);

        // 升降电机速度反馈（LiftingLkAction 用于堵转检测）
        register_input("/dart/lifting_left/velocity",  lifting_left_vel_fb_);
        register_input("/dart/lifting_right/velocity", lifting_right_vel_fb_);

        // 升降指令总线
        register_output(
            "/dart/manager/lifting/command", lifting_command_,
            rmcs_msgs::DartSliderStatus::WAIT);

        register_output("/dart/limiting_servo/control_angle", limiting_servo_angle_, (uint16_t)0u);

        try {
            max_transform_rate_ = get_parameter("max_transform_rate").as_double();
        } catch (...) {
            max_transform_rate_ = 500.0;
        }


        lifting_stall_threshold_    = get_parameter("lifting_stall_threshold").as_double();
        lifting_stall_confirm_ticks_ =
            (uint64_t)get_parameter("lifting_stall_confirm_ticks").as_int();
        lifting_stall_min_run_ticks_ =
            (uint64_t)get_parameter("lifting_stall_min_run_ticks").as_int();
        lifting_stall_timeout_ticks_ =
            (uint64_t)get_parameter("lifting_stall_timeout_ticks").as_int();

        open_angle_ = (uint16_t)get_parameter("limiting_open_angle").as_int();
        close_angle_ = (uint16_t)get_parameter("limiting_close_angle").as_int();
        limiting_fill_ticks_ = (uint64_t)get_parameter("limiting_fill_ticks").as_int();

        // 初始化限位舵机为锁定态（close_angle），只在 FillingTestTask 中控制
        *limiting_servo_angle_ = close_angle_;

        state_pub_ = create_publisher<std_msgs::msg::UInt8>("/dart/manager/state", 10);
    }

    void update() override {
        poll_command();
        switch (state_) {
        case State::IDLE:    dispatch_next_task(); break;
        case State::RUNNING: tick_current_task();  break;
        case State::ERROR:   break;
        }
    }

private:
    void poll_command() {
        std::string cmd;

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
            RCLCPP_INFO(logger_, "[DartManagerV2] received command: '%s'", cmd.c_str());
            if (task) {
                submit_task(std::move(task));
            } else {
                RCLCPP_WARN(logger_, "[DartManagerV2] unknown command: '%s'", cmd.c_str());
            }
        }
    }

    void submit_task(std::shared_ptr<Task> task) {
        task_queue_.push_back(std::move(task));
        RCLCPP_INFO(
            logger_, "[DartManagerV2] task queued: %s (queue size=%zu)",
            task_queue_.back()->name().c_str(), task_queue_.size());
    }

    void cancel_all() {
        task_queue_.clear();
        if (current_task_) {
            current_task_->cancel();
            current_task_.reset();
            RCLCPP_WARN(logger_, "[DartManagerV2] all tasks cancelled");
        }

        transition_to(State::IDLE);
    }

    void recover() {
        if (state_ == State::ERROR) {
            current_task_.reset();
            task_queue_.clear();
            RCLCPP_INFO(logger_, "[DartManagerV2] recovered from ERROR, state=IDLE");
            transition_to(State::IDLE);
        }
        RCLCPP_INFO(logger_, "[DartManagerV2] queued SliderInitTask for recovery");
    }

    void dispatch_next_task() {
        if (task_queue_.empty())
            return;

        current_task_ = std::move(task_queue_.front());
        task_queue_.pop_front();

        RCLCPP_INFO(
            logger_, "[DartManagerV2] dispatching task: '%s'", current_task_->name().c_str());
        transition_to(State::RUNNING);

        tick_current_task();
    }

    void tick_current_task() {
        if (!current_task_)
            return;

        ActionStatus status =
            first_tick_of_task_ ? current_task_->tick_first() : current_task_->tick();
        first_tick_of_task_ = false;

        if (status == ActionStatus::SUCCESS) {
            // 发次计数：fire 任务成功后递增
            if (current_task_->name() == "fire") {
            }

            current_task_->on_exit();
            RCLCPP_INFO(
                logger_, "[DartManagerV2] task '%s' SUCCESS", current_task_->name().c_str());
            current_task_.reset();
            transition_to(State::IDLE);

        } else if (status == ActionStatus::FAILURE) {
            RCLCPP_ERROR(
                logger_, "[DartManagerV2] task '%s' FAILURE → state=ERROR",
                current_task_->name().c_str());
            on_task_failure();
        }
    }

    void on_task_failure() {
        current_task_->on_exit();
        current_task_.reset();
        task_queue_.clear();

        transition_to(State::ERROR);
    }

    void transition_to(State new_state) {
        state_             = new_state;
        first_tick_of_task_ = true;

        if (state_pub_) {
            std_msgs::msg::UInt8 msg;
            msg.data = static_cast<uint8_t>(new_state);
            state_pub_->publish(msg);
        }
    }

    // 升降堵转参数便捷透传
    void pass_lifting_params(
        rmcs_msgs::DartSliderStatus& cmd,
        const double*& lv, const double*& rv,
        double& thr, uint64_t& conf, uint64_t& minr, uint64_t& tout) {
        cmd  = *lifting_command_;  // unused, just sugar — not called
        lv   = &(*lifting_left_vel_fb_);
        rv   = &(*lifting_right_vel_fb_);
        thr  = lifting_stall_threshold_;
        conf = lifting_stall_confirm_ticks_;
        minr = lifting_stall_min_run_ticks_;
        tout = lifting_stall_timeout_ticks_;
    }

    std::shared_ptr<Task> make_task(const std::string& cmd) {
        
        if (cmd == "fill_test") {
            return std::make_shared<FillingTestTask>(
                *lifting_command_,
                *lifting_left_vel_fb_, *lifting_right_vel_fb_,
                lifting_stall_threshold_, lifting_stall_confirm_ticks_,
                lifting_stall_min_run_ticks_, lifting_stall_timeout_ticks_,
                *limiting_servo_angle_,
                open_angle_, close_angle_, limiting_fill_ticks_);
        }

        RCLCPP_INFO(logger_, "filling");
        return nullptr;
    }

    rclcpp::Logger logger_;


    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    InputInterface<double> lifting_left_vel_fb_;
    InputInterface<double> lifting_right_vel_fb_;

    OutputInterface<rmcs_msgs::DartSliderStatus> lifting_command_;
    OutputInterface<uint16_t>                    limiting_servo_angle_;

    double   max_transform_rate_{500.0};

    double   lifting_stall_threshold_{0.5};
    uint64_t lifting_stall_confirm_ticks_{100};
    uint64_t lifting_stall_min_run_ticks_{500};
    uint64_t lifting_stall_timeout_ticks_{5000};
    uint16_t open_angle_{500};
    uint16_t close_angle_{1000};
    uint64_t limiting_fill_ticks_{500};

    InputInterface<std::string> remote_command_input_;
    InputInterface<std::string> web_command_input_;
    std::string                 last_command_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_pub_;

    State state_{State::IDLE};

    std::shared_ptr<Task>             current_task_;
    std::deque<std::shared_ptr<Task>> task_queue_;
    bool first_tick_of_task_{true};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::DartManagerForFillingTest, rmcs_executor::Component)
