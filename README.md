# DartManager

RMCS 框架插件组件，负责：
1. 维护飞镖全局状态机 (State Machine)
2. 管理任务队列 (Task Queue)，每帧驱动当前任务向前推进
3. 通过 RMCS Interface 黑板与底层硬件组件通信
4. 对任务失败提供统一的异常处理入口

## 状态机转换图

```text
  ┌─────────────────────────────────────────────────────────────────┐
  │                                                                 │
  │   IDLE ──[提交Task]──► RUNNING ──[SUCCESS]──► IDLE              │
  │     ▲                     │                                     │
  │     │                     ├──[FAILURE]──► ERROR                 │
  │     │                     │                                     │
  │  [recover]            [cancel()]                                │
  │     │                     │                                     │
  │   ERROR ◄────────────────-┘                                     │
  └─────────────────────────────────────────────────────────────────┘
```

## Interface 约定

### 输入 (来自底层硬件反馈)：
- `/dart/drive_belt/left/velocity` (`double`) 左同步带实际速度
- `/dart/drive_belt/right/velocity` (`double`) 右同步带实际速度
- `/dart/force_screw/velocity` (`double`) 丝杆实际速度

### 输出 (写给底层控制器)：
- `/dart/manager/belt/target_velocity` (`double`) 同步带目标速度
- `/dart/manager/force_screw/target_velocity` (`double`) 丝杆目标速度
- `/dart/manager/trigger/target_angle` (`double`) 扳机舵机目标角度

### 命令接口 (由外部组件/上位机写入，Manager 每帧读取)：
- `/dart/manager/command` (`std::string`) 任务命令字符串

## 简单开发手册

### 1. 组件初始化 (`DartManager`)
`DartManager` 继承自 `rmcs_executor::Component` 和 `rclcpp::Node`，在初始化时会注册一系列输入输出接口，用于与底层硬件和外部组件通信。

### 2. 状态机管理
系统状态由 `State` 枚举定义：
- `IDLE`：空闲，无任务运行。
- `RUNNING`：有任务正在执行。
- `ERROR`：任务失败，等待恢复。

每帧通过 `update()` 函数轮询命令并驱动任务执行：
- **空闲 (`IDLE`)**：从队列中取出下一个任务并开始执行。
- **运行中 (`RUNNING`)**：调用 `tick_current_task()` 推进当前任务。根据任务返回的状态 (`SUCCESS`, `FAILURE`, 或 `RUNNING`) 进行状态转换。
- **错误 (`ERROR`)**：暂停调度，等待外部发送 `recover` 命令。

### 3. 命令处理
外部组件可以通过写入 `/dart/manager/command` 接口发送指令（字符串格式）。
内置保留命令包括：
- `cancel`：取消当前任务并清空任务队列。
- `recover`：从 `ERROR` 状态恢复到 `IDLE`。

其他任务命令（如 `load`, `fire`）会在 `poll_command()` 中解析并生成对应的 `Task` 加入队列。

### 4. 任务调度 (`Task` & `Action`)
开发者可以通过派生 `Action` 或组装现有的 Action 创建 `Task`。`DartManager` 提供了 API 供任务访问底层的硬件输入输出接口，例如 `belt_target_velocity()` 和 `left_belt_velocity()`。

任务执行失败时会触发 `on_task_failure()`，该函数会将输出置零，保证系统安全，并将状态机置为 `ERROR`。
