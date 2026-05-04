# dart guidance 模块接入视觉需求单

## 1. 文档目的

本文档用于补充 `rmcs_dart_guidance` 模块的视觉接入需求，范围仅包含需求说明，不包含本轮代码实现。

本文档采用“当前实现现状 + 目标需求”的写法：

- 先说明当前仓库里已经具备的视觉与控制能力。
- 再明确接入 manager 所需补齐的接口、任务、配置和验收标准。
- 未在当前代码中实现的能力，本文档统一按“待实现需求”描述，不写成已有功能。

## 2. 当前实现现状

### 2.1 已实现能力

当前仓库已经具备一条基础视觉处理链路，核心能力如下：

- 已存在 `DartVisionCore` 组件，负责相机取图、图像预处理、目标初始识别和后续 tracking。
- 图像预处理当前基于 HSV 阈值分割和形态学开闭运算。
- 目标识别阶段会在连续若干帧内筛选最稳定的候选目标，得到初始目标坐标。
- 进入 tracking 阶段后，会持续输出当前目标在图像中的像素坐标。

当前视觉链路已经定义了以下输出接口：

- `/dart_guidance/camera/camera_image`
  当前原始调试图像。
- `/dart_guidance/camera/display_image`
  当前二值化后的调试图像。
- `/dart_guidance/camera/target_position`
  当前目标像素坐标，类型为 `cv::Point2i`。
- `/dart_guidance/tracker/tracking`
  当前 tracking 状态，类型为 `bool`。

当前目标坐标的无效值约定为：

- `target_position == cv::Point2i(-1, -1)` 表示当前没有可用目标坐标。

当前控制链路已经具备以下能力：

- `DartManager` 已经输出 `/dart_manager/angle/error_vector`。
- `DartSettingController` 已经消费 `/dart_manager/angle/error_vector`，并将其转换为 yaw / pitch 电机目标速度。
- runtime 已经具备 `ActionSet`，并支持 `ALL_SUCCESS` 并行收敛策略。

### 2.2 当前缺口

当前仓库中，与“视觉辅助发射准备”需求相比，仍存在以下缺口：

- `DartManager` 目前没有视觉输入接口，`ManagerInputContext` 中没有目标坐标、tracking 状态或视觉新鲜度字段。
- 当前没有 `vision_aim` action，无法根据视觉目标坐标自动生成 `/dart_manager/angle/error_vector`。
- 当前 `launch_prepare` 是纯机械顺序任务，没有视觉分支，也没有并行收敛逻辑。
- 当前没有根据 `fire_count` 返回不同视觉参数的配置类。
- 当前 `dart-guidance.yaml` 未将 `rmcs_dart_guidance::DartVisionCore` 加入主运行组件列表，因此主流程尚未真正启用视觉链路。

## 3. 目标需求概述

本需求的目标是在保留现有纯机械 `launch_prepare` 的前提下，新增一个视觉辅助版发射准备任务。

本文档统一将该任务命名为：

- `launch_prepare_with_vision`

该任务的目标是：

- 在执行现有机械发射准备流程的同时，使用视觉输出的目标像素坐标进行图像平面引导。
- 当目标被引导到给定参考点附近的允许误差范围内时，视觉分支判定成功。
- 机械分支和视觉分支均成功时，整个任务成功。

说明：

- 本需求中的闭环语义是“图像平面误差闭环”。
- 本需求不要求本轮引入相机标定、三维解算或真实空间中的 yaw / pitch 几何反解。
- `output.angle_error_vector` 在该需求下复用为“像素误差驱动量”，不是物理角度测量值。

## 4. 任务与动作需求

### 4.1 任务命名与总体结构

新增任务 `launch_prepare_with_vision`，其结构为：

- 机械分支：复用当前 `launch_prepare` 的机械行为。
- 视觉分支：新增 `vision_aim` action。
- 两个分支通过并行容器执行，收敛策略为 `ActionSet::Policy::ALL_SUCCESS`。

整体约束如下：

- 机械分支成功且视觉分支成功，整个任务成功。
- 任一分支失败，整个任务失败。
- 整体失败后，继续复用 `DartManager` 现有的失败收口逻辑进入 `ERROR`。

### 4.2 机械分支行为

视觉版任务的机械分支行为应与现有 `launch_prepare` 保持一致，不在本文档中重新定义机械控制细节。

具体要求：

- `fire_count == 0` 时，机械流程行为与现有首发 `launch_prepare` 一致。
- `fire_count > 0` 时，机械流程行为与现有后续发射 `launch_prepare` 一致。
- 视觉接入不得改变现有纯机械流程的动作顺序、退出语义和故障处理方式。

### 4.3 视觉分支动作 `vision_aim`

`vision_aim` 是一个新增 action，用于根据当前目标像素坐标生成图像平面误差，并驱动现有 yaw / pitch 控制链路。

#### 输入

`vision_aim` 需要以下输入：

- 当前目标坐标：`cv::Point2i current_target`
- 当前 tracking 状态：`bool tracking`
- 视觉新鲜度序号：`uint64_t target_seq`
- 参考瞄准坐标：`cv::Point2i reference_point`
- 当前发射序号对应的相对修正量：`cv::Point2i offset`
- 可接受误差范围：`cv::Point2i allowable_error`
- 动作超时：`uint64_t timeout_ticks`

说明：

- `reference_point` 表示参考瞄准点。
- `offset` 表示本次发射相对参考点的修正量。
- `allowable_error` 表示 x / y 两轴上的允许像素误差。
- `target_seq` 用于判定视觉输入是否仍在刷新，避免黑板中保留旧值却被误认为有效。

#### 输出

`vision_aim` 只复用现有输出接口：

- `output.angle_error_vector`

输出语义固定为：

- `angle_error_vector[0]` 表示图像 x 方向误差驱动量。
- `angle_error_vector[1]` 表示图像 y 方向误差驱动量。

说明：

- 虽然字段名为 `angle_error_vector`，但在该需求下，其数值语义是像素误差驱动量。
- 实际 yaw / pitch 电机方向与速度比例，继续由下游 `DartSettingController` 的现有增益配置负责。

#### 计算规则

目标瞄准点按以下规则计算：

- `desired_point = reference_point + offset`

误差按以下规则计算：

- `error_x = desired_point.x - current_target.x`
- `error_y = desired_point.y - current_target.y`

发布规则为：

- `output.angle_error_vector = Eigen::Vector2d(error_x, error_y)`

#### 成功条件

`vision_aim` 满足以下条件时判定成功：

- `tracking == true`
- `current_target != cv::Point2i(-1, -1)`
- `abs(error_x) <= allowable_error.x`
- `abs(error_y) <= allowable_error.y`

#### 失败条件

`vision_aim` 满足以下任一条件时判定失败：

- 动作执行超时，即 `elapsed_ticks() >= timeout_ticks`
- `tracking == false`
- `current_target == cv::Point2i(-1, -1)`
- 视觉输入在超时窗口内没有刷新，即 `target_seq` 在动作执行期间始终未发生变化
- 当前发射序号没有对应的视觉参数配置

本需求中对丢失目标的处理要求如下：

- v1 不做自动重识别等待。
- 一旦进入 `vision_aim` 后出现目标无效或 tracking 失效，动作直接失败。

#### 退出行为

`vision_aim` 在以下三种退出路径中都必须清零输出：

- `SUCCESS`
- `FAILURE`
- `CANCEL`

清零要求为：

- `output.angle_error_vector = Eigen::Vector2d::Zero()`

## 5. manager 接口接入需求

为了让 `vision_aim` 能在 manager 内使用，后续实现时需要补齐以下输入接口：

- 当前目标坐标接口
- tracking 状态接口
- 视觉新鲜度序号接口

本文档统一约定视觉新鲜度序号接口为：

- `/dart_guidance/camera/target_seq`

类型约定为：

- `uint64_t`

刷新规则约定为：

- 每当视觉模块对当前帧完成目标结果更新时，`target_seq` 自增一次。
- 无论当前帧结果是有效目标还是无效目标，只要该帧完成了视觉结果写入，都应刷新 `target_seq`。

该设计的目的在于：

- 区分“目标静止但视觉仍在更新”和“视觉线程已停滞但黑板仍保留旧坐标”。

## 6. 按发射次数取参需求

### 6.1 类职责

需要新增一个只负责视觉参数取用的类，本文档统一命名为：

- `VisionAimProfileProvider`

该类职责固定如下：

- 负责从 YAML 读取视觉瞄准参数。
- 负责根据当前 `fire_count` 返回本次发射所需的视觉参数。
- 只负责视觉参数读取与查询，不负责 task 调度、状态机推进或控制输出。

### 6.2 支持范围

- 最多支持 4 次发射参数。
- `fire_count == 0` 对应第 1 次发射参数。
- `fire_count == 1` 对应第 2 次发射参数。
- `fire_count == 2` 对应第 3 次发射参数。
- `fire_count == 3` 对应第 4 次发射参数。

超出范围时的要求：

- 若当前 `fire_count` 没有对应配置，任务创建或动作初始化应直接失败。

## 7. YAML 配置需求

本文档只定义配置需求，不定义本轮具体代码实现。

配置应分为两类：

- 公共配置
- 按发射次数区分的 shot profile 配置

### 7.1 公共配置

公共配置至少应包含：

- `allowable_error`
- `timeout_ticks`

### 7.2 shot profile 配置

每个 shot profile 至少应包含：

- `reference_point`
- `offset`

### 7.3 建议配置形态

建议采用如下 YAML 结构表达需求：

```yaml
dart_manager:
  ros__parameters:
    vision_aim:
      allowable_error: {x: 5, y: 5}
      timeout_ticks: 3000
      shot_profiles:
        - reference_point: {x: 640, y: 360}
          offset: {x: 0, y: 0}
        - reference_point: {x: 640, y: 360}
          offset: {x: 8, y: -3}
        - reference_point: {x: 640, y: 360}
          offset: {x: 10, y: -5}
        - reference_point: {x: 640, y: 360}
          offset: {x: 12, y: -6}
```

说明：

- `reference_point` 与 `offset` 在运行时语义上等价于 `cv::Point2i`。
- 文档中 YAML 使用 `{x, y}` 仅作为配置表达形式。
- `timeout_ticks` 的单位应与 manager 主循环 tick 保持一致；按当前运行配置，`1000 ticks` 约等于 `1 s`。
- 若实际只需要前 1 到 3 次发射参数，也允许只配置对应数量，但超过已配置次数时必须失败，不得隐式复用最后一组参数。

## 8. 启动与运行要求

视觉辅助发射准备要真正工作，后续实现时还必须满足以下启动条件：

- 主运行配置需要加载 `rmcs_dart_guidance::DartVisionCore`
- manager 需要接入视觉输入接口
- 视觉版任务需要能被 command / task factory 创建

当前调试图像相关组件的定位如下：

- `camera_image` 与 `display_image` 仅作为调试观察能力
- `ImagePublisher` 与 `DebugDisplayComponent` 仅作为辅助调试手段
- 调试显示不是 `launch_prepare_with_vision` 成功的功能前提

## 9. 验收标准

后续实现完成后，至少应满足以下验收场景：

### 9.1 正常收敛

- 视觉组件正常运行并持续刷新 `target_position`、`tracking` 和 `target_seq`
- `launch_prepare_with_vision` 启动后，机械分支按现有流程运行
- 视觉分支持续发布误差到 `/dart_manager/angle/error_vector`
- 目标进入 `allowable_error` 范围后，视觉分支成功
- 机械分支和视觉分支均成功后，整个任务成功

### 9.2 视觉超时失败

- 机械分支正常完成
- 但目标始终未进入允许误差范围
- 到达 `timeout_ticks` 后，`vision_aim` 失败
- 整个任务失败，并复用现有 manager 安全收口逻辑

### 9.3 tracking 丢失失败

- 任务执行过程中出现 `tracking == false`
- 或 `target_position == (-1, -1)`
- `vision_aim` 直接失败
- 整个任务失败

### 9.4 视觉输入停滞失败

- 任务执行期间黑板里保留旧目标值
- 但 `target_seq` 在整个动作超时窗口内没有更新
- 应判定为视觉输入失活并失败

### 9.5 参数缺失失败

- 当前 `fire_count` 对应的 shot profile 不存在
- 任务不得继续进入正常视觉瞄准流程
- 应直接失败，而不是使用默认值或复用其他次数的参数

### 9.6 兼容性要求

- 现有纯机械 `launch_prepare` 行为保持不变
- 现有 `fire_preload`、`manual_control` 等任务行为保持不变
- `recover` 后 `fire_count` 清零的现有语义保持不变

## 10. 非目标范围

本需求文档明确不包含以下内容：

- 本轮代码实现
- 相机标定
- PnP
- 三维空间坐标解算
- 弹道模型
- 对现有纯机械 `launch_prepare` 的替换

## 11. 结论

当前仓库已经具备“输出目标像素坐标”和“消费姿态误差向量”的两端能力，但中间仍缺少 manager 侧的视觉接入、视觉动作、发射参数提供者和运行配置接线。

因此，本需求的核心工作不是重写现有视觉模块，而是在保持现有机械流程不变的前提下，补齐以下闭环链路：

- 视觉结果接入 manager
- `vision_aim` 生成像素误差驱动量
- `launch_prepare_with_vision` 并行收敛
- `VisionAimProfileProvider` 按 `fire_count` 提供参数
- YAML 和启动配置完成接线
