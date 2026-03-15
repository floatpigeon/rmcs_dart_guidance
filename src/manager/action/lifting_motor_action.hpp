#pragma once

#include "action.hpp"

#include <cstdint>

namespace rmcs_dart_guidance::manager {

// 升降舵机动作：on_enter 下发目标角度，等待 settle_ticks 后返回 SUCCESS。
class LiftingServoAction : public IAction {
public:
    /// @param name                         动作名称（如 "lifting_down" / "lifting_up"）
    /// @param left_target_angle            【输出引用】升降平台左电机目标角度
    /// @param right_target_angle           【输出引用】升降平台右电机目标角度
    /// @param command_left_angle           升降平台左电机命令角度
    /// @param command_right_angle          升降平台右电机命令角度
    /// @param settle_ticks                 电机到位等待帧数
    LiftingServoAction(
        const char* name,
        uint16_t& left_target_angle,
        uint16_t& right_target_angle,
        uint16_t command_left_angle,
        uint16_t command_right_angle,
        uint64_t settle_ticks = 1500)
        : IAction(name)
        , left_target_angle_(left_target_angle)
        , right_target_angle_(right_target_angle)
        , command_left_angle_(command_left_angle)
        , command_right_angle_(command_right_angle)
        , settle_ticks_(settle_ticks) {}

    void on_enter() override {
        left_target_angle_ = command_left_angle_;
        right_target_angle_ = command_right_angle_;
    }

    ActionStatus update() override {
        return (elapsed_ticks() >= settle_ticks_) ? ActionStatus::SUCCESS : ActionStatus::RUNNING;
    }

private:
    uint16_t& left_target_angle_;
    uint16_t& right_target_angle_;

    uint16_t command_left_angle_;
    uint16_t command_right_angle_;

    uint64_t settle_ticks_;
};

} // namespace rmcs_dart_guidance::manager