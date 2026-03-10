#include "rmcs_executor/component.hpp"
#include <rclcpp/node.hpp>
namespace rmcs_dart_guidance::manager::debug {

class FeedbackData
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    FeedbackData()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_output("/dart/drive_belt/left/velocity", left_belt_velocity_);
        register_output("/dart/drive_belt/right/velocity", right_belt_velocity_);
        register_output("/dart/force_screw/velocity", force_screw_velocity_);
    }

    void update() override {
        *left_belt_velocity_ = 0.0;
        *right_belt_velocity_ = 0.0;
        *force_screw_velocity_ = 0.0;
    }

private:
    OutputInterface<double> left_belt_velocity_;
    OutputInterface<double> right_belt_velocity_;
    OutputInterface<double> force_screw_velocity_;
};
} // namespace rmcs_dart_guidance::manager::debug

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::debug::FeedbackData, rmcs_executor::Component)
