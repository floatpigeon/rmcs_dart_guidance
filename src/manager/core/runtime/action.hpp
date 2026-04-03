#pragma once

#include <cstdint>
#include <string>

namespace rmcs_dart_guidance::manager {

// ─────────────────────────────────────────────────────────────────────────────
// ActionStatus
//   动作的生命周期状态，由 Action::update() 每帧返回。
// ─────────────────────────────────────────────────────────────────────────────
enum class ActionStatus { RUNNING, SUCCESS, FAILURE };

// ─────────────────────────────────────────────────────────────────────────────
// IAction
//   所有动作的纯虚基类。
//
//   生命周期：
//     on_enter()  —— 第一次被调度时调用（仅一次）
//     update()    —— 每个控制周期调用，返回当前状态
//     on_exit()   —— 动作结束（SUCCESS / FAILURE / 被取消）时调用
//
//   注意：update() 必须是非阻塞的，不允许在内部 sleep 或 spin。
// ─────────────────────────────────────────────────────────────────────────────
class IAction {
public:
    explicit IAction(std::string name)
        : name_(std::move(name)) {}

    virtual ~IAction() = default;

    // 禁止拷贝和移动，Action 通过 shared_ptr 管理
    IAction(const IAction&) = delete;
    IAction& operator=(const IAction&) = delete;
    IAction(IAction&&) = delete;
    IAction& operator=(IAction&&) = delete;

    // ── 供子类重写 ────────────────────────────────────────────────────────────

    // 进入时调用，可在此初始化内部计数器、记录起始时刻等
    virtual void on_enter() {}

    // 每帧调用，返回执行状态
    virtual ActionStatus update() = 0;

    // 退出时调用（无论是 SUCCESS / FAILURE 还是被外部 cancel），可在此做清理
    virtual void on_exit() {}

    // ── 供外部调用 ────────────────────────────────────────────────────────────

    const std::string& name() const { return name_; }

    // 获取动作已运行的帧数
    uint64_t elapsed_ticks() const { return elapsed_ticks_; }

    // ── 框架内部使用（由 ActionSequence / ActionSet 调用）───────────────────

    // 初始化并执行第一帧，返回状态
    ActionStatus tick_first() {
        elapsed_ticks_ = 0;
        on_enter();
        return tick();
    }

    // 执行后续帧
    ActionStatus tick() {
        ++elapsed_ticks_;
        return update();
    }

    // 强制退出（被取消时使用）
    void cancel() { on_exit(); }

private:
    std::string name_;
    uint64_t elapsed_ticks_{0};
};

} // namespace rmcs_dart_guidance::manager
