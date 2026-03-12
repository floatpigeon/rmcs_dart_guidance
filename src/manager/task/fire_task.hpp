#pragma once

#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class FireTask : public Task {
public:
    explicit FireTask(double& trigger_target_angle)
        : Task("fire", "发射") {

        then(
            std::make_shared<TriggerControlAction>(
                trigger_target_angle, // 扳机目标角度（输出）
                0.0,                  // 设置角度（弧度）
                100                   // 等待释放完成帧数
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
