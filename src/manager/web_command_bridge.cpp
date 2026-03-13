#include <cstdint>
#include <mutex>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// WebCommandBridge
//   处理与 WebUI 的原生 ROS 通信，维护心跳机制，输出经过安全校验的命令。
//   在心跳断开时，自动切断 Web 控制权，防止旧命令意外触发。
// ─────────────────────────────────────────────────────────────────────────────
class WebCommandBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    WebCommandBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , last_heartbeat_time_(this->now() - rclcpp::Duration::from_seconds(10.0)) {

        // ── 给 DartManager 读取的输出 ──────────────────────────────────────────
        register_output("/dart/manager/web_command", web_command_output_, std::string{});

        // ── 原生 ROS 订阅 ──────────────────────────────────────────────────────
        // 这里订阅 WebUI 实际发送的主题。为了不和 DartManager 的内部接口冲突，我们订阅 "/dart/webui/command"
        web_cmd_sub_ = create_subscription<std_msgs::msg::String>(
            "/dart/webui/command", 10, [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
                std::lock_guard<std::mutex> lock(web_command_mutex_);
                web_command_buffer_ = msg->data;
            });

        web_heartbeat_sub_ = create_subscription<std_msgs::msg::Empty>(
            "/dart/webui/heartbeat", rclcpp::QoS(1).best_effort(),
            [this](const std_msgs::msg::Empty::ConstSharedPtr&) {
                std::lock_guard<std::mutex> lock(web_command_mutex_);
                last_heartbeat_time_ = this->now();
            });

        RCLCPP_INFO(logger_, "[WebCommandBridge] initialized");
    }

    void update() override {
        std::string cmd_to_emit = "";

        std::lock_guard<std::mutex> lock(web_command_mutex_);
        
        // 检查心跳超时
        if ((this->now() - last_heartbeat_time_).seconds() > 1.5) {
            if (web_command_enable_) {
                RCLCPP_WARN(logger_, "[WebCommandBridge] WebUI heartbeat timeout. Control revoked.");
                web_command_enable_ = false;
                web_command_buffer_.clear(); // 断开连接时清空遗留指令
            }
        } else {
            if (!web_command_enable_) {
                RCLCPP_INFO(logger_, "[WebCommandBridge] WebUI heartbeat restored. Control granted.");
                web_command_enable_ = true;
            }
        }

        // 如果启用，提取缓冲区的命令
        if (web_command_enable_ && !web_command_buffer_.empty()) {
            cmd_to_emit = web_command_buffer_;
            web_command_buffer_.clear();
        }

        *web_command_output_ = cmd_to_emit;
    }

private:
    rclcpp::Logger logger_;

    OutputInterface<std::string> web_command_output_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr web_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr web_heartbeat_sub_;

    std::mutex web_command_mutex_;
    std::string web_command_buffer_;
    rclcpp::Time last_heartbeat_time_;
    bool web_command_enable_ = false;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::WebCommandBridge, rmcs_executor::Component)
