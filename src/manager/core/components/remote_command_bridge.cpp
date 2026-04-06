#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::manager {

// RemoteCommandBridge
//   将遥控器 DR16 的拨杆信号翻译为 DartManager 可识别的离散命令。
//
//   键位映射：
//     ┌──────────────────┬──────────────────────────────────────────────────────┐
//     │ 左拨杆  右拨杆   │ 功能                                               │
//     ├──────────────────┼──────────────────────────────────────────────────────┤
//     │ DOWN    DOWN     │ 全部停止 -> "cancel"                                │
//     │ MIDDLE  MIDDLE   │ 初始状态 -> "recover"                               │
//     │ MIDDLE  DOWN→MID │ 切换上膛/退膛 -> "launch_prepare" / "launch_cancel" │
//     │ MIDDLE  UP       │ 处于上膛状态时发射 -> "fire_preload"                │
//     └──────────────────┴──────────────────────────────────────────────────────┘
class RemoteCommandBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    RemoteCommandBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        register_output("/dart/manager/command", command_output_, std::string{});

        RCLCPP_INFO(logger_, "[RemoteCommandBridge] initialized");
    }

    void update() override {
        using namespace rmcs_msgs;

        const auto left = *switch_left_;
        const auto right = *switch_right_;
        const bool toggle_triggered = detect_toggle(left, right);
        const bool recover_triggered = detect_recover(left, right) && !toggle_triggered;

        emit_command("");

        if (left == Switch::DOWN && right == Switch::DOWN) {
            emit_command("cancel");
            chambered_ = false;
            update_previous_switches(left, right);
            return;
        }

        if (left == Switch::MIDDLE) {
            if (toggle_triggered) {
                if (chambered_) {
                    emit_command("launch_cancel");
                    chambered_ = false;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] toggle -> launch_cancel");
                } else {
                    emit_command("launch_prepare");
                    chambered_ = true;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] toggle -> launch_prepare");
                }
                update_previous_switches(left, right);
                return;
            }

            if (recover_triggered) {
                emit_command("recover");
                chambered_ = false;
                update_previous_switches(left, right);
                return;
            }

            if (chambered_ && right == Switch::UP) {
                emit_command("fire_preload");
                chambered_ = false;
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] fire_preload!");
                update_previous_switches(left, right);
                return;
            }
        }

        update_previous_switches(left, right);
    }

private:
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    bool detect_toggle(rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE
            && prev_left_ == rmcs_msgs::Switch::MIDDLE
            && prev_right_ == rmcs_msgs::Switch::DOWN
            && current_right == rmcs_msgs::Switch::MIDDLE;
    }

    bool detect_recover(rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) const {
        return current_left == rmcs_msgs::Switch::MIDDLE
            && current_right == rmcs_msgs::Switch::MIDDLE
            && !(prev_left_ == rmcs_msgs::Switch::MIDDLE
                 && prev_right_ == rmcs_msgs::Switch::MIDDLE);
    }

    void update_previous_switches(rmcs_msgs::Switch current_left, rmcs_msgs::Switch current_right) {
        prev_left_ = current_left;
        prev_right_ = current_right;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<std::string> command_output_;

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    bool chambered_{false};
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
