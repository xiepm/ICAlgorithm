# `coordinator/` 使用文档

## 这个目录是做什么的

`coordinator/` 是整个项目最像“总控大脑”的目录。  
当前这里只有一个核心头文件：

- `hic_control_coordinator.h`

里面的 `HicControlCoordinator` 负责把下面这些模块串起来：

- 机器人状态观测
- 扭矩/外力观测
- 运动学
- 动力学
- 笛卡尔阻抗核心
- 安全限制
- 力矩 / 电流输出

如果你把项目理解成“很多小模块拼起来”，  
那 `coordinator/` 就是负责“拼”和“调度顺序”的地方。

## 它在项目中的位置

```text
外部 C API
   -> coordinator
      -> state
      -> sensing
      -> kinematics
      -> dynamics
      -> control
      -> safety + output
```

## 这个目录最适合谁看

- 想理解整个控制流程的人
- 想改模式切换逻辑的人
- 想改零力示教的人
- 想改定点阻抗 / 轨迹阻抗流程的人
- 想查“最终输出为什么变成这样”的人

## 当前最重要的类

### `HicControlCoordinator`

它的职责可以分成 6 大块：

### 1. 生命周期与初始化

负责：

- 接收 `HicControlConfig`
- 初始化运动学/动力学后端
- 初始化状态观测器和力观测器
- 初始化阻抗控制核心
- 重置运行期缓存

### 2. 高频状态输入

负责把外部输入送入内部状态链路，例如：

- 关节位置
- 电机电流
- 关节速度
- 关节加速度
- 关节测量力矩

### 3. 模式管理

当前顶层主模式：

- `IDLE`
- `FORCE_CONTROL`

力控子模式：

- `ZERO_FORCE`
- `IMPEDANCE_FIXED`
- `IMPEDANCE_TRAJECTORY`

### 4. 参数与目标管理

负责：

- 设置阻抗增益
- 设置目标模式
- 设置固定点目标
- 设置在线轨迹目标
- 设置动力学参数
- 设置电流/力矩换算参数
- 设置安全边界

### 5. 控制输出

负责输出两种执行器命令：

- 关节力矩命令
- 电机电流命令

同时会把安全逻辑也一起接上，例如：

- 关节限位
- 力矩限幅
- 力矩变化率限制
- 电流限幅

### 6. 调试与状态读取

负责对外提供：

- 当前控制模式
- 当前力控子模式
- 当前机器人状态
- 当前末端状态
- 最近一次关节力矩命令
- 最近一次笛卡尔 wrench 指令

## 零力示教是在哪里完成的

零力示教不是一个单独目录，而是在 `HicControlCoordinator` 中组织完成的。  
最关键的是这几类函数：

- `startZeroForceMode`
- `prepareStopZeroForceMode`
- `computeZeroForceTorqueCommand`
- `computeZeroForceCurrentCommand`

它的大致控制链路是：

```text
先检查能不能进入
 -> 读取当前机器人状态
 -> 计算重力补偿
 -> 加速度阻尼 / 软边界阻尼
 -> 安全限幅
 -> 输出力矩或电流
 -> 满足条件后平滑退出
```

## 定点阻抗是在哪里完成的

定点阻抗也不是一个单独类，而是在 coordinator 内部通过下面这些组成：

- `setImpedanceTargetMode(HIC_TARGET_MODE_FIXED)`
- `setFixedTargetPose(...)`
  或 `captureCurrentPoseAsFixedTarget()`
- `startCartesianImpedanceMode()`
- `computeCartesianImpedanceTorqueCommand()`
- `computeCartesianImpedanceCurrentCommand()`

## 外部应用应不应该直接依赖这个目录

一般不建议。  
对外正式入口应该还是：

- `interface/hic_c_api.h`

只有在你是项目内部开发者、需要改控制流程本身时，才建议直接依赖 `coordinator/`。

## 一句总结

`coordinator/` 是整个项目的“总装配和总调度层”，  
大多数“这个控制流程为什么这样走”的答案，都在这里。
