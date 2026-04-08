#pragma once

#include "manager/core/runtime/action.hpp"
#include "rmcs_msgs/dart_servo_command.hpp"

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

class TriggerControlAction : public IAction {
public:
    TriggerControlAction(
        std::string name,                                             //
        rmcs_msgs::DartServoCommand& trigger_servo_command_interface, //
        rmcs_msgs::DartServoCommand trigger_servo_command,            //
        uint64_t settle_ticks = 50                                    //
        )
        : IAction(std::move(name))
        , trigger_servo_command_output_interface_(trigger_servo_command_interface)
        , trigger_servo_command_(trigger_servo_command)
        , settle_ticks_(settle_ticks) {}

    void on_enter() override { trigger_servo_command_output_interface_ = trigger_servo_command_; }

    ActionStatus update() override {
        if (elapsed_ticks() >= settle_ticks_) {
            return ActionStatus::SUCCESS;
        }
        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 保持锁定或释放状态，不重置
        // 扳机在整个发射流程中应保持状态，直到显式改变
    }

private:
    rmcs_msgs::DartServoCommand& trigger_servo_command_output_interface_;
    rmcs_msgs::DartServoCommand trigger_servo_command_;
    uint64_t settle_ticks_;
};

} // namespace rmcs_dart_guidance::manager
