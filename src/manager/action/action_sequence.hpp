#pragma once

#include "action.hpp"

#include <memory>
#include <string>
#include <vector>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ActionSequence
//   顺序动作组：将若干 IAction 按加入顺序依次执行。
//
//   - 当前动作返回 SUCCESS 后自动推进到下一个动作
//   - 当前动作返回 FAILURE 时，整组立刻返回 FAILURE（并 cancel 当前动作）
//   - 所有动作均完成后返回 SUCCESS
//
//   ActionSequence 本身也是一个 IAction，因此可以嵌套。
// ─────────────────────────────────────────────────────────────────────────────
class ActionSequence : public IAction {
public:
    explicit ActionSequence(std::string name)
        : IAction(std::move(name)) {}

    // 追加动作（Builder 风格，支持链式调用）
    ActionSequence& then(std::shared_ptr<IAction> action) {
        actions_.push_back(std::move(action));
        return *this;
    }

    // ── IAction override ──────────────────────────────────────────────────────

    void on_enter() override {
        cursor_ = 0;
        if (!actions_.empty()) {
            // 第一个动作立刻进入
            first_tick_of_current_ = true;
        }
    }

    ActionStatus update() override {
        // 空序列视为立即成功
        if (actions_.empty())
            return ActionStatus::SUCCESS;

        if (cursor_ >= actions_.size())
            return ActionStatus::SUCCESS;

        auto& current = *actions_[cursor_];

        // 首帧调用 tick_first，后续调用 tick
        ActionStatus status = first_tick_of_current_ ? current.tick_first() : current.tick();
        first_tick_of_current_ = false;

        if (status == ActionStatus::SUCCESS) {
            current.on_exit();
            ++cursor_;
            if (cursor_ >= actions_.size()) {
                return ActionStatus::SUCCESS;
            }
            // 下一个动作首帧标记
            first_tick_of_current_ = true;
            return ActionStatus::RUNNING;
        }

        if (status == ActionStatus::FAILURE) {
            current.cancel();
            return ActionStatus::FAILURE;
        }

        return ActionStatus::RUNNING;
    }

    void on_exit() override {
        // 如果被外部取消时仍有动作在跑，清理它
        if (cursor_ < actions_.size()) {
            actions_[cursor_]->cancel();
        }
    }

    // 查询子动作数量
    std::size_t size() const { return actions_.size(); }

    // 查询当前执行到第几个动作（0-based）
    std::size_t cursor() const { return cursor_; }

private:
    std::vector<std::shared_ptr<IAction>> actions_;
    std::size_t cursor_{0};
    bool first_tick_of_current_{true};
};

} // namespace rmcs_dart_guidance::manager
