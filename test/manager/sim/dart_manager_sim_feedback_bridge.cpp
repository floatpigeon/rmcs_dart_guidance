#include "manager/sim/dart_sim_bus.hpp"

#include <mutex>

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace rmcs_dart_guidance::manager::sim {

class DartManagerSimFeedbackBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManagerSimFeedbackBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_output("/dart/drive_belt/left/velocity", left_belt_velocity_, feedback_.left_belt_velocity);
        register_output(
            "/dart/drive_belt/right/velocity", right_belt_velocity_, feedback_.right_belt_velocity);
        register_output("/dart/drive_belt/left/torque", left_belt_torque_, feedback_.left_belt_torque);
        register_output(
            "/dart/drive_belt/right/torque", right_belt_torque_, feedback_.right_belt_torque);
        register_output("/dart/lifting_left/velocity", lifting_left_velocity_, feedback_.left_lift_velocity);
        register_output(
            "/dart/lifting_right/velocity", lifting_right_velocity_, feedback_.right_lift_velocity);
        register_output("/dart/sim/belt/position", belt_position_output_, feedback_.belt_position);
        register_output("/dart/sim/lift/position", lift_position_output_, feedback_.lift_position);
        register_output(
            "/dart/sim/trigger/locked", trigger_locked_output_, feedback_.trigger_locked);
        register_output(
            "/dart/sim/limiting/status", limiting_status_output_, feedback_.limiting_status);

        const auto qos = bus::reliable_qos();
        left_belt_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbLeftBeltVelocity, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.left_belt_velocity = msg->data;
            });
        right_belt_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbRightBeltVelocity, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.right_belt_velocity = msg->data;
            });
        left_belt_torque_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbLeftBeltTorque, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.left_belt_torque = msg->data;
            });
        right_belt_torque_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbRightBeltTorque, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.right_belt_torque = msg->data;
            });
        left_lift_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbLeftLiftVelocity, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.left_lift_velocity = msg->data;
            });
        right_lift_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbRightLiftVelocity, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.right_lift_velocity = msg->data;
            });
        belt_position_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbBeltPosition, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.belt_position = msg->data;
            });
        lift_position_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kFbLiftPosition, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.lift_position = msg->data;
            });
        trigger_locked_sub_ = create_subscription<std_msgs::msg::Bool>(
            bus::kFbTriggerLocked, qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.trigger_locked = msg->data;
            });
        limiting_status_sub_ = create_subscription<std_msgs::msg::UInt8>(
            bus::kFbLimitingStatus, qos,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                std::scoped_lock lock(feedback_mutex_);
                feedback_.limiting_status = bus::decode_servo_command(msg->data);
            });

        write_outputs(feedback_);
        RCLCPP_INFO(logger_, "[DartManagerSimFeedbackBridge] initialized");
    }

    void update() override {
        Feedback feedback_snapshot;
        {
            std::scoped_lock lock(feedback_mutex_);
            feedback_snapshot = feedback_;
        }
        write_outputs(feedback_snapshot);
    }

private:
    struct Feedback {
        double left_belt_velocity{0.0};
        double right_belt_velocity{0.0};
        double left_belt_torque{0.0};
        double right_belt_torque{0.0};
        double left_lift_velocity{0.0};
        double right_lift_velocity{0.0};
        double belt_position{0.0};
        double lift_position{1.0};
        bool trigger_locked{false};
        rmcs_msgs::DartServoCommand limiting_status{rmcs_msgs::DartServoCommand::LOCK};
    };

    void write_outputs(const Feedback& feedback) {
        *left_belt_velocity_ = feedback.left_belt_velocity;
        *right_belt_velocity_ = feedback.right_belt_velocity;
        *left_belt_torque_ = feedback.left_belt_torque;
        *right_belt_torque_ = feedback.right_belt_torque;
        *lifting_left_velocity_ = feedback.left_lift_velocity;
        *lifting_right_velocity_ = feedback.right_lift_velocity;
        *belt_position_output_ = feedback.belt_position;
        *lift_position_output_ = feedback.lift_position;
        *trigger_locked_output_ = feedback.trigger_locked;
        *limiting_status_output_ = feedback.limiting_status;
    }

    rclcpp::Logger logger_;

    std::mutex feedback_mutex_;
    Feedback feedback_;

    OutputInterface<double> left_belt_velocity_;
    OutputInterface<double> right_belt_velocity_;
    OutputInterface<double> left_belt_torque_;
    OutputInterface<double> right_belt_torque_;
    OutputInterface<double> lifting_left_velocity_;
    OutputInterface<double> lifting_right_velocity_;
    OutputInterface<double> belt_position_output_;
    OutputInterface<double> lift_position_output_;
    OutputInterface<bool> trigger_locked_output_;
    OutputInterface<rmcs_msgs::DartServoCommand> limiting_status_output_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_belt_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_belt_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_belt_torque_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_belt_torque_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr left_lift_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr right_lift_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr belt_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_position_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_locked_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limiting_status_sub_;
};

} // namespace rmcs_dart_guidance::manager::sim

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_dart_guidance::manager::sim::DartManagerSimFeedbackBridge, rmcs_executor::Component)
