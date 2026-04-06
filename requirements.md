# 项目需求

## 核心需求

1. 总体设计不需要大改，核心思想依然是将飞镖发射架的各个机械动作封装成action，多个action将组成task

2. action分两种，一种是action开始，直到某个反馈（比如堵转）后结束这个动作；另一种是action开始后依然可以做其他action，直到某个action结束后跟着一起结束，适用于belt_wait这样需要
在其他任务完成前保持当前状态的任务

3. 需要更加丰富的日志信息，现在需要具体到action内，当action开始执行时输出一次，当因为异常退出时也要具体到是因为哪个动作

4. 需要拥有并行任务的能力，未来会有视觉修正接入，视觉修正需要和发射准备并行

5. 动作和任务切换时要紧密衔接，不要出现空帧

6. task_context 再次拆分，可以分为inputinterface、outputinterface和setting_parameter，职责更清晰，代码可读性更强