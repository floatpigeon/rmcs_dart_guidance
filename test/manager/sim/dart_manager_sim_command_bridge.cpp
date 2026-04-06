#include "manager/sim/dart_sim_bus.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace rmcs_dart_guidance::manager::sim {

class DartManagerSimCommandBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartManagerSimCommandBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/manager/belt/command", belt_command_);
        register_input("/dart/manager/belt/target_velocity", belt_target_velocity_, false);
        register_input("/dart/manager/belt/torque_limit", belt_torque_limit_, false);
        register_input("/dart/manager/belt/hold_torque", belt_hold_torque_, false);
        register_input("/dart/manager/belt/wait_zero_velocity", belt_wait_zero_velocity_, false);
        register_input("/dart/manager/trigger/lock_enable", trigger_lock_enable_);
        register_input("/dart/manager/lifting/command", lifting_command_);
        register_input("/dart/manager/limiting/command", limiting_command_);

        const auto qos = bus::reliable_qos();
        belt_command_pub_ = create_publisher<std_msgs::msg::UInt8>(bus::kCmdBeltCommand, qos);
        belt_target_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kCmdBeltTargetVelocity, qos);
        belt_torque_limit_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kCmdBeltTorqueLimit, qos);
        belt_hold_torque_pub_ =
            create_publisher<std_msgs::msg::Float64>(bus::kCmdBeltHoldTorque, qos);
        belt_wait_zero_velocity_pub_ =
            create_publisher<std_msgs::msg::Bool>(bus::kCmdBeltWaitZeroVelocity, qos);
        trigger_lock_enable_pub_ =
            create_publisher<std_msgs::msg::Bool>(bus::kCmdTriggerLockEnable, qos);
        lifting_command_pub_ = create_publisher<std_msgs::msg::UInt8>(bus::kCmdLiftingCommand, qos);
        limiting_command_pub_ =
            create_publisher<std_msgs::msg::UInt8>(bus::kCmdLimitingCommand, qos);
        tick_pub_ = create_publisher<std_msgs::msg::UInt64>(bus::kTick, qos);

        RCLCPP_INFO(logger_, "[DartManagerSimCommandBridge] initialized");
    }

    void update() override {
        std_msgs::msg::UInt8 belt_command_msg;
        belt_command_msg.data = bus::encode(*belt_command_);
        belt_command_pub_->publish(belt_command_msg);

        std_msgs::msg::Float64 belt_target_velocity_msg;
        belt_target_velocity_msg.data =
            belt_target_velocity_.ready() ? *belt_target_velocity_ : 0.0;
        belt_target_velocity_pub_->publish(belt_target_velocity_msg);

        std_msgs::msg::Float64 belt_torque_limit_msg;
        belt_torque_limit_msg.data = belt_torque_limit_.ready() ? *belt_torque_limit_ : 0.0;
        belt_torque_limit_pub_->publish(belt_torque_limit_msg);

        std_msgs::msg::Float64 belt_hold_torque_msg;
        belt_hold_torque_msg.data = belt_hold_torque_.ready() ? *belt_hold_torque_ : 0.0;
        belt_hold_torque_pub_->publish(belt_hold_torque_msg);

        std_msgs::msg::Bool belt_wait_zero_velocity_msg;
        belt_wait_zero_velocity_msg.data =
            belt_wait_zero_velocity_.ready() && *belt_wait_zero_velocity_;
        belt_wait_zero_velocity_pub_->publish(belt_wait_zero_velocity_msg);

        std_msgs::msg::Bool trigger_lock_enable_msg;
        trigger_lock_enable_msg.data = *trigger_lock_enable_;
        trigger_lock_enable_pub_->publish(trigger_lock_enable_msg);

        std_msgs::msg::UInt8 lifting_command_msg;
        lifting_command_msg.data = bus::encode(*lifting_command_);
        lifting_command_pub_->publish(lifting_command_msg);

        std_msgs::msg::UInt8 limiting_command_msg;
        limiting_command_msg.data = bus::encode(*limiting_command_);
        limiting_command_pub_->publish(limiting_command_msg);

        std_msgs::msg::UInt64 tick_msg;
        tick_msg.data = tick_counter_++;
        tick_pub_->publish(tick_msg);
    }

private:
    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::DartMechanismCommand> belt_command_;
    InputInterface<double> belt_target_velocity_;
    InputInterface<double> belt_torque_limit_;
    InputInterface<double> belt_hold_torque_;
    InputInterface<bool> belt_wait_zero_velocity_;
    InputInterface<bool> trigger_lock_enable_;
    InputInterface<rmcs_msgs::DartMechanismCommand> lifting_command_;
    InputInterface<rmcs_msgs::DartServoCommand> limiting_command_;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr belt_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr belt_target_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr belt_torque_limit_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr belt_hold_torque_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr belt_wait_zero_velocity_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trigger_lock_enable_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr lifting_command_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr limiting_command_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr tick_pub_;

    uint64_t tick_counter_{0};
};

} // namespace rmcs_dart_guidance::manager::sim

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_dart_guidance::manager::sim::DartManagerSimCommandBridge, rmcs_executor::Component)
