#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::manager {

class ManagerCommandMux
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ManagerCommandMux()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_input("/dart/source/remote_command", remote_command_input_, false);
        register_input("/dart/source/gui_command", gui_command_input_, false);
        register_output("/dart/manager/command", command_output_, std::string{});

        RCLCPP_INFO(logger_, "[ManagerCommandMux] initialized");
    }

    void before_updating() override {
        if (!remote_command_input_.ready()) {
            remote_command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/dart/source/remote_command\". Set to empty string.");
        }

        if (!gui_command_input_.ready()) {
            gui_command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/dart/source/gui_command\". Set to empty string.");
        }
    }

    void update() override {
        const std::string remote_command =
            remote_command_input_.ready() ? *remote_command_input_ : std::string{};
        const std::string gui_command =
            gui_command_input_.ready() ? *gui_command_input_ : std::string{};

        *command_output_ = remote_command.empty() ? gui_command : remote_command;
    }

private:
    rclcpp::Logger logger_;

    InputInterface<std::string> remote_command_input_;
    InputInterface<std::string> gui_command_input_;
    OutputInterface<std::string> command_output_;
};

} // namespace rmcs_dart_guidance::manager

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::manager::ManagerCommandMux, rmcs_executor::Component)
