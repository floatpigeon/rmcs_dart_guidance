#pragma once

#include "manager/action/trigger_control_action.hpp"
#include "manager/task/task.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class FireTask : public Task {
public:
    explicit FireTask(bool& trigger_lock_enable)
        : Task("fire", "发射") {

        then(
            std::make_shared<TriggerControlAction>(
                trigger_lock_enable, // 扳机锁定使能（输出）
                false,               // 解锁（false）
                1000                 // 等待释放完成帧数
                ));
    }
};

} // namespace rmcs_dart_guidance::manager
