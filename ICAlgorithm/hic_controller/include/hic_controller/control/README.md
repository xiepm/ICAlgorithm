# `control/` 使用文档

## 这个目录是做什么的

`control/` 放的是“控制算法核心”，当前最重要的文件是：

- `hic_cartesian_impedance_core.h`

它的职责很纯粹：  
只负责**笛卡尔阻抗的数学计算和目标管理**，不负责把整个机器人控制流程跑起来。

## 当前目录里的核心类

### `HicCartesianImpedanceCore`

主要能力：

- 初始化阻抗控制核心
- 设置阻抗增益
- 设置目标模式
  - 固定点目标
  - 在线轨迹目标
- 根据当前末端状态和 Jacobian 计算关节力矩命令
- 输出最近一次阻抗计算的笛卡尔 wrench 和笛卡尔误差，方便调试

## 它负责什么，不负责什么

### 它负责

- 6 维笛卡尔误差计算
- 阻抗律计算
- 末端 wrench 到关节力矩的映射
- 固定目标 / 轨迹目标的管理

### 它不负责

- 模式切换
- 状态更新
- 关节限位保护
- 电流限幅
- 力矩转电流
- 零力示教
- 机器人动力学补偿

换句话说，`control/` 更像是“数学内核”，不是“控制系统总调度器”。

## 它通常被谁调用

正常情况下，不建议外部应用直接调用它。  
它通常由下面这个类统一调度：

- `coordinator/HicControlCoordinator`

调用关系大致是：

```text
外部状态输入
  -> coordinator
  -> kinematics 计算当前 pose / jacobian / twist
  -> control 核心计算阻抗力矩
  -> coordinator 再做安全限幅和电流换算
```

## 什么时候适合直接看这个目录

你遇到下面这些问题时，应该优先看 `control/`：

- 定点阻抗“手感不对”
- 轨迹阻抗跟踪响应异常
- 想调刚度和阻尼
- 想看当前阻抗误差和 wrench 指令到底怎么来的

## 最小理解路径

如果你只想快速理解阻抗核，建议按这个顺序看 `hic_cartesian_impedance_core.h`：

1. `initialize`
2. `setGains`
3. `setTargetMode`
4. `setFixedTargetPose`
5. `setOnlineTargetPose`
6. `computeJointTorque`
7. `getLastCartesianWrenchCommand`
8. `getLastCartesianError`

## 四种模式的标准调用顺序

下面这张表不是 `control/` 目录自己单独完成的调用顺序，而是“外部通过 C API 使用力控模式时，`control/` 内核通常在整条链路中的位置”。

| 模式 | 标准调用顺序 | 说明 |
|---|---|---|
| 零力示教 | `hic_initialize_control(...)` -> 运行配置接口 -> `hic_update_state_estimates(...)` -> `hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_ZERO_FORCE)` -> 每周期 `hic_update_state_estimates(...)` -> 每周期 `hic_get_force_control_current_commands(...)` | 零力示教不依赖 `control/` 里的笛卡尔阻抗核求解主链路。它主要走重力补偿、阻尼、限位保护和力矩转电流链路。 |
| 定点位置保持 | `hic_initialize_control(...)` -> 运行配置接口 -> `hic_set_cartesian_impedance_gains(...)` -> 可选 `hic_set_force_control_nullspace_config(...)` -> 可选 `hic_capture_current_joint_position_as_nullspace_target()` -> `hic_update_state_estimates(...)` -> `hic_set_cartesian_fixed_position_target(...)` 或 `hic_capture_current_position_as_fixed_target()` -> `hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION)` -> 每周期 `hic_update_state_estimates(...)` -> 每周期 `hic_get_force_control_current_commands(...)` | 这里会真正进入 `control/` 里的阻抗核。目标是只约束末端位置 XYZ，不约束姿态。若启用零空间，最好在启动前先抓取一次零空间参考关节位姿。 |
| 定点位姿保持 | `hic_initialize_control(...)` -> 运行配置接口 -> `hic_set_cartesian_impedance_gains(...)` -> 可选 `hic_set_force_control_nullspace_config(...)` -> 可选 `hic_capture_current_joint_position_as_nullspace_target()` -> `hic_update_state_estimates(...)` -> `hic_set_cartesian_fixed_pose_target_zyx_euler(...)` 或 `hic_capture_current_pose_as_fixed_target()` -> `hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE)` -> 每周期 `hic_update_state_estimates(...)` -> 每周期 `hic_get_force_control_current_commands(...)` | 这条链路和“定点位置保持”很像，但这里位置和姿态都会被约束。外部姿态输入是固定坐标系 `Z-Y-X` 欧拉角，进入 `hic_c_api.cpp` 后会先转成四元数，再交给内部阻抗核。 |
| 轨迹笛卡尔阻抗 | `hic_initialize_control(...)` -> 运行配置接口 -> `hic_set_cartesian_impedance_gains(...)` -> 可选 `hic_set_force_control_nullspace_config(...)` -> 可选 `hic_capture_current_joint_position_as_nullspace_target()` -> `hic_update_state_estimates(...)` -> `hic_set_cartesian_trajectory_target_zyx_euler(...)` 先给一个初始目标 -> `hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY)` -> 每周期 `hic_update_state_estimates(...)` -> 每周期在线刷新 `hic_set_cartesian_trajectory_target_zyx_euler(...)` -> 每周期 `hic_get_force_control_current_commands(...)` | 轨迹模式和固定点模式最大的区别是：目标不是设一次就不动，而是运行中持续刷新。`control/` 内核每个周期都基于最新目标位姿、目标速度和当前末端状态重新求解关节力矩。 |

### 这张表里最关键的顺序关系

有三条顺序最好记住：

1. 阻抗增益、零空间参数一般在 `hic_start_force_control_mode(...)` 之前先设置。  
   这样第一次进入阻抗模式时，内核就已经拿到完整参数。

2. “抓当前姿态/位置作为目标”之前，最好先做一次 `hic_update_state_estimates(...)`。  
   否则抓到的可能不是最新末端状态。

3. 轨迹模式下，`hic_set_cartesian_trajectory_target_zyx_euler(...)` 不只是启动前调用一次，  
   运行中通常也要持续调用，用来刷新目标位姿和目标速度。

## 一句总结

`control/` 是“算控制量”的地方，  
不是“负责整台机器人安全运行”的地方。
