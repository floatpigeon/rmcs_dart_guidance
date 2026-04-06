#include "manager/sim/dart_sim_bus.hpp"
#include "manager/sim/dart_sim_core.hpp"

#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace rmcs_dart_guidance::manager::sim {

class DartManagerSimNode : public rclcpp::Node {
public:
    DartManagerSimNode()
        : Node("dart_manager_sim_node")
        , sim_(load_config()) {
        const auto qos = bus::reliable_qos();

        belt_command_sub_ = create_subscription<std_msgs::msg::UInt8>(
            bus::kCmdBeltCommand, qos,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.belt_command = bus::decode_motor_command(msg->data);
            });
        belt_target_velocity_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kCmdBeltTargetVelocity, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.belt_target_velocity = msg->data;
            });
        belt_torque_limit_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kCmdBeltTorqueLimit, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.belt_torque_limit = msg->data;
            });
        belt_hold_torque_sub_ = create_subscription<std_msgs::msg::Float64>(
            bus::kCmdBeltHoldTorque, qos,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.belt_hold_torque = msg->data;
            });
        belt_wait_zero_velocity_sub_ = create_subscription<std_msgs::msg::Bool>(
            bus::kCmdBeltWaitZeroVelocity, qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.belt_wait_zero_velocity = msg->data;
            });
        trigger_lock_enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            bus::kCmdTriggerLockEnable, qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.trigger_lock_enable = msg->data;
            });
        lifting_command_sub_ = create_subscription<std_msgs::msg::UInt8>(
            bus::kCmdLiftingCommand, qos,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.lifting_command = bus::decode_motor_command(msg->data);
            });
        limiting_command_sub_ = create_subscription<std_msgs::msg::UInt8>(
            bus::kCmdLimitingCommand, qos,
            [this](const std_msgs::msg::UInt8::SharedPtr msg) {
                std::scoped_lock lock(command_mutex_);
                command_.limiting_command = bus::decode_servo_command(msg->data);
            });
        tick_sub_ = create_subscription<std_msgs::msg::UInt64>(
            bus::kTick, qos,
            [this](const std_msgs::msg::UInt64::SharedPtr) {
                DartSimControlInput command_snapshot;
                {
                    std::scoped_lock lock(command_mutex_);
                    command_snapshot = command_;
                }
                sim_.step(command_snapshot);
                publish_state();
            });

        left_belt_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kFbLeftBeltVelocity, qos);
        right_belt_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kFbRightBeltVelocity, qos);
        left_belt_torque_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kFbLeftBeltTorque, qos);
        right_belt_torque_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kFbRightBeltTorque, qos);
        left_lift_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kFbLeftLiftVelocity, qos);
        right_lift_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kFbRightLiftVelocity, qos);
        belt_position_pub_ = create_publisher<std_msgs::msg::Float64>(bus::kFbBeltPosition, qos);
        lift_position_pub_ = create_publisher<std_msgs::msg::Float64>(bus::kFbLiftPosition, qos);
        trigger_locked_pub_ = create_publisher<std_msgs::msg::Bool>(bus::kFbTriggerLocked, qos);
        limiting_status_pub_ =
            create_publisher<std_msgs::msg::UInt8>(bus::kFbLimitingStatus, qos);

        publish_state();
        RCLCPP_INFO(get_logger(), "[DartManagerSimNode] initialized");
    }

private:
    DartSimConfig load_config() {
        DartSimConfig config;
        get_parameter_or("initial_belt_position", config.initial_belt_position, 0.0);
        get_parameter_or("initial_lift_position", config.initial_lift_position, 1.0);
        get_parameter_or("initial_trigger_locked", config.initial_trigger_locked, false);
        get_parameter_or(
            "belt_position_per_velocity_tick", config.belt_position_per_velocity_tick, 0.001);
        get_parameter_or("belt_limit_torque_fallback", config.belt_limit_torque_fallback, 1.0);
        get_parameter_or("belt_move_torque_ratio", config.belt_move_torque_ratio, 0.25);
        get_parameter_or("lift_position_per_tick", config.lift_position_per_tick, 0.002);
        get_parameter_or("lift_feedback_velocity", config.lift_feedback_velocity, 2.0);
        return config;
    }

    void publish_state() {
        const auto& state = sim_.state();

        std_msgs::msg::Float64 left_belt_velocity_msg;
        left_belt_velocity_msg.data = state.left_belt_velocity;
        left_belt_velocity_pub_->publish(left_belt_velocity_msg);

        std_msgs::msg::Float64 right_belt_velocity_msg;
        right_belt_velocity_msg.data = state.right_belt_velocity;
        right_belt_velocity_pub_->publish(right_belt_velocity_msg);

        std_msgs::msg::Float64 left_belt_torque_msg;
        left_belt_torque_msg.data = state.left_belt_torque;
        left_belt_torque_pub_->publish(left_belt_torque_msg);

        std_msgs::msg::Float64 right_belt_torque_msg;
        right_belt_torque_msg.data = state.right_belt_torque;
        right_belt_torque_pub_->publish(right_belt_torque_msg);

        std_msgs::msg::Float64 left_lift_velocity_msg;
        left_lift_velocity_msg.data = state.left_lift_velocity;
        left_lift_velocity_pub_->publish(left_lift_velocity_msg);

        std_msgs::msg::Float64 right_lift_velocity_msg;
        right_lift_velocity_msg.data = state.right_lift_velocity;
        right_lift_velocity_pub_->publish(right_lift_velocity_msg);

        std_msgs::msg::Float64 belt_position_msg;
        belt_position_msg.data = state.belt_position;
        belt_position_pub_->publish(belt_position_msg);

        std_msgs::msg::Float64 lift_position_msg;
        lift_position_msg.data = state.lift_position;
        lift_position_pub_->publish(lift_position_msg);

        std_msgs::msg::Bool trigger_locked_msg;
        trigger_locked_msg.data = state.trigger_locked;
        trigger_locked_pub_->publish(trigger_locked_msg);

        std_msgs::msg::UInt8 limiting_status_msg;
        limiting_status_msg.data = bus::encode(state.limiting_status);
        limiting_status_pub_->publish(limiting_status_msg);
    }

    std::mutex command_mutex_;
    DartSimControlInput command_{};
    DartSimCore sim_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr belt_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr belt_target_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr belt_torque_limit_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr belt_hold_torque_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr belt_wait_zero_velocity_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trigger_lock_enable_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr lifting_command_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr limiting_command_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr tick_sub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_belt_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_belt_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_belt_torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_belt_torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_lift_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_lift_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr belt_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lift_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_locked_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limiting_status_pub_;
};

} // namespace rmcs_dart_guidance::manager::sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rmcs_dart_guidance::manager::sim::DartManagerSimNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
