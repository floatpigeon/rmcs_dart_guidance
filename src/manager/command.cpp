#include <cstdint>
#include <string>

#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// RemoteCommandBridge
//   将遥控器 DR16 的拨杆/摇杆信号翻译为 DartManager 可识别的命令字和控制量。
//
//   键位映射：
//     ┌──────────────────┬──────────────────────────────────────────────────────┐
//     │ 左拨杆  右拨杆   │ 功能                                               │
//     ├──────────────────┼──────────────────────────────────────────────────────┤
//     │ DOWN    DOWN     │ 全部停止 → "cancel"                                │
//     │ MIDDLE  MIDDLE   │ 初始状态 → "recover"                               │
//     │ MIDDLE  DOWN→MID │ 切换上膛/退膛 → "launch_prepare" / "unload"        │
//     │ MIDDLE  UP       │ 处于上膛状态时发射 → "fire"                        │
//     │ UP      MIDDLE   │ 设置模式：摇杆调整 yaw/pitch                       │
//     │ UP      DOWN     │ 设置模式：调整拉力                                 │
//     └──────────────────┴──────────────────────────────────────────────────────┘
//
//   所有信息通过 Interface 传递，不持有 DartManager 指针。
// ─────────────────────────────────────────────────────────────────────────────
class RemoteCommandBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    // ── 力控模式枚举 ──────────────────────────────────────────────────────────
    enum class ForceControlMode : uint8_t {
        MANUAL = 0,      // 手动控制（摇杆直接控制力螺母速度）
        CLOSED_LOOP = 1, // 力闭环模式
    };

    RemoteCommandBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {

        // ── 遥控器输入 ───────────────────────────────────────────────────────
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_input("/remote/joystick/left", joystick_left_, false);
        register_input("/remote/joystick/right", joystick_right_, false);

        // ── 命令输出（写给 DartManager 读取）─────────────────────────────────
        register_output("/dart/manager/command", command_output_, std::string{});

        // ── 设置模式输出 ─────────────────────────────────────────────────────
        register_output("/dart/command/yaw_delta", yaw_delta_output_, 0.0);
        register_output("/dart/command/pitch_delta", pitch_delta_output_, 0.0);
        register_output("/dart/command/force_screw_delta", force_screw_delta_output_, 0.0);
        register_output(
            "/dart/command/force_control_mode", force_control_mode_output_,
            static_cast<uint8_t>(ForceControlMode::MANUAL));

        // ── 从 yaml 读取力闭环模式配置 ───────────────────────────────────────
        try {
            auto mode_str = get_parameter("force_control_mode").as_string();
            if (mode_str == "closed_loop") {
                configured_force_mode_ = ForceControlMode::CLOSED_LOOP;
            } else {
                configured_force_mode_ = ForceControlMode::MANUAL;
            }
        } catch (...) {
            configured_force_mode_ = ForceControlMode::MANUAL;
        }

        // ── 从 yaml 读取摇杆灵敏度 ──────────────────────────────────────────
        try {
            joystick_sensitivity_ = get_parameter("joystick_sensitivity").as_double();
        } catch (...) {
            joystick_sensitivity_ = 0.01;
        }

        RCLCPP_INFO(logger_, "[RemoteCommandBridge] initialized");
    }

    void update() override {
        using S = rmcs_msgs::Switch;

        auto left = *switch_left_;
        auto right = *switch_right_;

        // 每帧先将设置模式的增量归零，只在对应分支中写入非零值
        *yaw_delta_output_ = 0.0;
        *pitch_delta_output_ = 0.0;
        *force_screw_delta_output_ = 0.0;

        // ── 最高优先级：双下 → 全部停止 ──────────────────────────────────────
        if (left == S::DOWN && right == S::DOWN) {
            emit_command("cancel");
            chambered_ = false;
            prev_right_ = right;
            return;
        }

        // ── 双中 → 初始状态（recover）────────────────────────────────────────
        if (left == S::MIDDLE && right == S::MIDDLE) {
            emit_command("recover");
            detect_toggle(right); // 更新边沿状态但不执行动作
            return;
        }

        // ── 左中：操控模式 ───────────────────────────────────────────────────
        if (left == S::MIDDLE) {
            // 右拨杆 DOWN→MIDDLE 边沿：切换上膛/退膛
            if (detect_toggle(right)) {
                if (chambered_) {
                    emit_command("unload");
                    chambered_ = false;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] toggle -> unload");
                } else {
                    emit_command("launch_prepare");
                    chambered_ = true;
                    RCLCPP_INFO(logger_, "[RemoteCommandBridge] toggle -> launch_prepare");
                }
                return;
            }

            // 处于上膛状态时，右拨杆上 → 发射
            if (chambered_ && right == S::UP) {
                emit_command("fire");
                chambered_ = false; // 发射后自动退出上膛状态
                RCLCPP_INFO(logger_, "[RemoteCommandBridge] fire!");
                return;
            }

            // 非触发帧：清空命令（允许 DartManager 重置去重状态）
            emit_command("");
            return;
        }

        // ── 左上：设置模式 ───────────────────────────────────────────────────
        if (left == S::UP) {
            if (right == S::MIDDLE) {
                // 右摇杆控制 yaw，左摇杆控制 pitch（分开防误触）
                *yaw_delta_output_ = joystick_right_->x() * joystick_sensitivity_;
                *pitch_delta_output_ = joystick_left_->y() * joystick_sensitivity_;
            } else if (right == S::DOWN) {
                // 调整拉力，模式由 yaml 配置决定
                *force_control_mode_output_ = static_cast<uint8_t>(configured_force_mode_);
                *force_screw_delta_output_ = joystick_right_->y() * joystick_sensitivity_;
            }
            // 右拨杆上：设置模式下无定义，增量保持已归零

            emit_command(""); // 设置模式下不发送任务命令
            prev_right_ = right;
            return;
        }

        // 未覆盖的组合（如 left==DOWN right!=DOWN）：安全清空
        emit_command("");
        prev_right_ = right;
    }

private:
    // ── 发送命令（写入 OutputInterface）───────────────────────────────────────
    void emit_command(const std::string& cmd) { *command_output_ = cmd; }

    // ── 边沿检测：右拨杆 DOWN→MIDDLE 触发一次 ────────────────────────────────
    //   返回 true 表示检测到一次 toggle 边沿
    bool detect_toggle(rmcs_msgs::Switch current_right) {
        bool triggered =
            (prev_right_ == rmcs_msgs::Switch::DOWN && current_right == rmcs_msgs::Switch::MIDDLE);
        prev_right_ = current_right;
        return triggered;
    }

    // ── 成员变量 ──────────────────────────────────────────────────────────────
    rclcpp::Logger logger_;

    // 遥控器输入
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    InputInterface<Eigen::Vector2d> joystick_left_;
    InputInterface<Eigen::Vector2d> joystick_right_;

    // 命令输出
    OutputInterface<std::string> command_output_;

    // 设置模式输出
    OutputInterface<double> yaw_delta_output_;
    OutputInterface<double> pitch_delta_output_;
    OutputInterface<double> force_screw_delta_output_;
    OutputInterface<uint8_t> force_control_mode_output_;

    // 配置
    ForceControlMode configured_force_mode_{ForceControlMode::MANUAL};
    double joystick_sensitivity_{0.01};

    // 内部状态
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    bool chambered_{false}; // 是否处于上膛状态
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::RemoteCommandBridge, rmcs_executor::Component)
