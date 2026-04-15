#pragma once

#include "manager/core/runtime/task.hpp"
#include "manager/manager_types.hpp"
#include "manager/resources/actions/trigger_control_action.hpp"

#include <memory>

namespace rmcs_dart_guidance::manager {

class TriggerLockTask : public Task {
public:
    explicit TriggerLockTask(ManagerOutputContext& output)
        : Task("trigger_lock", "扳机锁定") {

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_lock",
                output.trigger_command,
                rmcs_msgs::DartServoCommand::LOCK,
                1000));
    }
};

class TriggerFreeTask : public Task {
public:
    explicit TriggerFreeTask(ManagerOutputContext& output)
        : Task("trigger_free", "扳机释放") {

        then(
            std::make_shared<TriggerControlAction>(
                "trigger_free",
                output.trigger_command,
                rmcs_msgs::DartServoCommand::FREE,
                100));
    }
};

} // namespace rmcs_dart_guidance::manager
