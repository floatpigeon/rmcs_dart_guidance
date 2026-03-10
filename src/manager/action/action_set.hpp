#pragma once

#include "action.hpp"

#include <cassert>
#include <memory>
#include <string>
#include <vector>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ActionSet
//   并行动作组：将若干 IAction 在同一帧同时推进。
//
//   策略（可在构造时选择）：
//     ALL_SUCCESS —— 所有子动作都 SUCCESS 才返回 SUCCESS；
//                    任意一个 FAILURE 则立刻取消其余并返回 FAILURE。
//     ANY_SUCCESS —— 任意一个 SUCCESS 即返回 SUCCESS，并取消其余；
//                    所有子动作均 FAILURE 才返回 FAILURE。
//
//   ActionSet 本身也是一个 IAction，可以嵌套进 ActionSequence。
// ─────────────────────────────────────────────────────────────────────────────
class ActionSet : public IAction {
public:
    enum class Policy {
        ALL_SUCCESS, // 所有成功才算成功
        ANY_SUCCESS, // 任意成功即算成功
    };

    explicit ActionSet(std::string name, Policy policy = Policy::ALL_SUCCESS)
        : IAction(std::move(name))
        , policy_(policy) {}

    // 追加动作（Builder 风格）
    ActionSet& also(std::shared_ptr<IAction> action) {
        assert(action && "ActionSet::also: action cannot be null");
        if (action) {
            entries_.push_back({std::move(action), false, ActionStatus::RUNNING});
        }
        return *this;
    }

    void on_enter() override {
        for (auto& e : entries_) {
            e.done = false;
            e.status = ActionStatus::RUNNING;
            if (!e.action) {
                e.done = true; // Skip null actions
            }
        }
        first_tick_ = true;
    }

    ActionStatus update() override {
        if (entries_.empty())
            return ActionStatus::SUCCESS;

        int running_count = 0;
        int success_count = 0;
        int failure_count = 0;

        for (auto& e : entries_) {
            if (e.done || !e.action)
                continue;

            ActionStatus s = first_tick_ ? e.action->tick_first() : e.action->tick();

            if (s == ActionStatus::SUCCESS) {
                e.action->on_exit();
                e.done = true;
                e.status = ActionStatus::SUCCESS;
                ++success_count;
                if (policy_ == Policy::ANY_SUCCESS && success_count > 0) {
                    cancel_running();
                    return ActionStatus::SUCCESS;
                }
            } else if (s == ActionStatus::FAILURE) {
                e.action->cancel();
                e.done = true;
                e.status = ActionStatus::FAILURE;
                ++failure_count;
                if (policy_ == Policy::ALL_SUCCESS && failure_count > 0) {
                    cancel_running();
                    return ActionStatus::FAILURE;
                }
            } else {
                ++running_count;
            }
        }
        first_tick_ = false;

        if (policy_ == Policy::ALL_SUCCESS) {
            return (running_count == 0) ? ActionStatus::SUCCESS : ActionStatus::RUNNING;
        } else { // ANY_SUCCESS
            return (running_count == 0) ? ActionStatus::FAILURE : ActionStatus::RUNNING;
        }
    }

    void on_exit() override { cancel_running(); }

private:
    struct Entry {
        std::shared_ptr<IAction> action;
        bool done;
        ActionStatus status;
    };

    void cancel_running() {
        for (auto& e : entries_) {
            if (!e.done && e.action) {
                e.action->cancel();
                e.done = true;
            }
        }
    }

    std::vector<Entry> entries_;
    Policy policy_;
    bool first_tick_{true};
};

} // namespace rmcs_dart_guidance::manager
