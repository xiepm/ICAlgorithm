# `interface/` 使用文档

## 这个目录是做什么的

`interface/` 是整个项目最正式、最推荐外部使用的接口层。  
如果你是：

- 上位机开发者
- PLC/工控集成方
- Python/C/C++ 外部调用方
- demo 或测试程序作者

一般都应该优先从这个目录进入。

## 当前目录里的文件

- `hic_c_api.h`
  正式对外 C API。
- `hic_export.h`
  导出宏定义，用于跨平台符号导出。

## 为什么推荐优先用 C API

因为它把内部复杂细节屏蔽掉了。  
外部调用方不需要直接知道：

- coordinator 里有哪些私有状态
- kinematics / dynamics backend 如何切换
- 零力示教内部怎么做安全判断
- 阻抗控制内部如何调用 Jacobian

外部通常只需要做三件事：

1. 初始化
2. 周期送状态
3. 按当前模式取输出

## 当前 API 的整体逻辑

### 1. 高频输入接口

核心接口：

- `hic_update_state_estimates(...)`

你每个控制周期都应该先把最新状态送进来。  
当前常见输入包括：

- 关节位置
- 电机电流
- 当前时间戳

### 2. 低频配置接口

例如：

- 初始化控制器
- 设置动力学参数
- 设置电流-力矩换算参数
- 设置关节安全范围
- 设置传感器配置

这些一般不需要每周期调用。

### 3. 力控参数与目标接口

这部分主要给“阻抗子模式”用，例如：

- 设置阻抗增益
- 设置固定点目标
- 设置在线轨迹目标

### 4. 力控模式接口

当前项目已经统一成“力控大类”：

- 顶层模式：`IDLE / FORCE_CONTROL`
- 力控子模式：
  - `ZERO_FORCE`
  - `IMPEDANCE_FIXED`
  - `IMPEDANCE_TRAJECTORY`

因此对外推荐优先使用统一接口，例如：

- 启动力控零力示教
- 启动力控阻抗模式
- 请求停止力控模式
- 统一获取力控输出命令

## 旧接口和新接口的关系

你可能会在 `hic_c_api.h` 里看到一些旧名字，例如：

- 获取零力示教电流命令
- 获取阻抗模式电流命令
- 获取阻抗模式力矩命令

这些接口仍然保留，是为了兼容旧测试和旧调用方。  
但从接口设计角度看，推荐新代码优先使用“统一的力控大类接口”。

## 最推荐的外部使用顺序

### 零力示教

```text
初始化
 -> 更新状态
 -> 启动零力示教
 -> 每周期更新状态
 -> 每周期读取力控输出
 -> 请求停止力控
```

### 定点阻抗

```text
初始化
 -> 设置阻抗增益
 -> 更新状态
 -> 捕获当前位姿或设置固定目标
 -> 启动阻抗子模式
 -> 每周期更新状态
 -> 每周期读取力控输出
```

## 最适合配合哪个测试看

如果你想看最接近“真实外部调用”的用法，建议看：

- `hic_controller/test/hic_controller_cli.cpp`
- `hic_controller/test/hic_state_generator.cpp`

## 一句总结

`interface/` 是外部应该握在手里的那层接口，  
新代码能不直接碰内部类，就尽量不要直接碰。

---

## `hic_c_api.h` 接口文档

### 基本约定

`hic_c_api.h` 是 HIC 控制库当前唯一正式对外 C API 入口。
外部集成方应只依赖这个头文件和 `hic_c_api_types.h` / `hic_config.h` 中的公开类型，不建议直接包含或调用 `coordinator`、`control`、`kinematics`、`dynamics` 等内部 C++ 类。

所有接口返回 `int`，语义对应 `HicStatus`：

- `HIC_STATUS_OK`：调用成功。
- `HIC_STATUS_ERROR_INIT`：控制器尚未初始化，或内部模块未准备好。
- `HIC_STATUS_ERROR_INVALID_PARAM`：参数非法，例如枚举值越界、关节数非法、配置值不合理。
- `HIC_STATUS_ERROR_NULL_POINTER`：必需的结构体或数组指针为空。
- `HIC_STATUS_ERROR_ROBOT_STATE`：当前机器人状态不满足控制要求。
- `HIC_STATUS_ERROR_CURRENT_LIMIT`：输出被电流或力矩限制夹紧。
- `HIC_STATUS_ERROR_NOT_IMPLEMENTED`：接口形态已预留，但当前版本还没有完整接线。

每个接口第一个参数 `groupId` 用于标识控制组。当前实现仍以单控制器实例为主，建议外部先统一传 `0`。后续如果扩展多组机器人或多实例控制，可继续沿用这个参数。

### 单位约定

内部计算统一使用 SI 单位：

- 长度：`m`
- 角度：`rad`
- 线速度：`m/s`
- 角速度：`rad/s`
- 角加速度：`rad/s^2`
- 力矩：`N.m`
- 电流：`A`
- 时间：`s`

为了兼容上位机、PLC 和旧系统习惯，部分 C API 输入使用工程单位，并在 `hic_c_api.cpp` 的边界层转换为内部 SI 单位：

- 关节角度输入通常使用 `deg`。
- 关节角速度输入通常使用 `deg/s`。
- 关节角加速度输入通常使用 `deg/s^2`。
- 笛卡尔位置输入通常使用 `mm`。
- 笛卡尔线速度输入通常使用 `mm/s`。
- 姿态欧拉角输入使用 `deg`。

输出接口默认返回内部 SI 语义，除非函数注释另有明确说明。

### 版本接口

| 接口 | 作用 | 调用时机 |
| --- | --- | --- |
| `hic_get_version()` | 返回库内部版本字符串 | 程序启动、日志记录、兼容性检查 |

返回的字符串由库内部管理，调用方不需要释放，也不应该修改。

### 初始化与低频配置

初始化和配置接口通常在控制循环开始前调用，不需要每周期重复调用。

| 接口 | 作用 |
| --- | --- |
| `hic_initialize_control(groupId, config)` | 创建并初始化控制器实例，设置关节数、控制周期、机型和基础运动学参数 |
| `hic_set_dynamics_linear_parameters(groupId, dynamic_params)` | 设置线性化动力学参数 |
| `hic_set_payload_mass_properties(groupId, mass, center_of_mass)` | 设置末端负载质量和质心 |
| `hic_set_motor_torque_conversion_parameters(groupId, torque_constant, gear_ratio, transmission_efficiency)` | 设置电机电流到关节力矩的换算参数 |
| `hic_set_joint_position_limits(groupId, limits)` | 设置关节位置上下界，输入单位 `deg` |
| `hic_set_joint_velocity_limits(groupId, limits)` | 设置关节速度最大绝对值，输入单位 `deg/s` |
| `hic_set_joint_acceleration_limits(groupId, limits)` | 设置关节加速度最大绝对值，输入单位 `deg/s^2` |
| `hic_set_joint_torque_limits(groupId, limits)` | 设置关节力矩上下界，单位 `N.m` |
| `hic_set_motor_current_limits(groupId, limits)` | 设置电机电流上下界，单位 `A` |
| `hic_set_robot_mounting_angles(groupId, rotation, tilt)` | 通过安装角设置重力方向，输入单位 `deg` |
| `hic_set_gravity_vector_in_base(groupId, gx, gy, gz)` | 直接设置基坐标系下重力向量，单位 `m/s^2` |

推荐最小配置顺序：

```text
hic_initialize_control
 -> hic_set_dynamics_linear_parameters
 -> hic_set_motor_torque_conversion_parameters
 -> hic_set_joint_position_limits
 -> hic_set_joint_velocity_limits
 -> hic_set_joint_acceleration_limits
 -> hic_set_joint_torque_limits
 -> hic_set_motor_current_limits
 -> 按需设置重力、传感器、滤波和安全配置
```

### 传感器、滤波与安全配置

| 接口 | 作用 |
| --- | --- |
| `hic_set_joint_torque_sensor_parameters(groupId, config)` | 设置关节扭矩传感器通道、方向、零偏、比例和故障阈值 |
| `hic_set_state_filter_config(groupId, config)` | 设置状态观测器中的位置、速度、加速度、电流滤波参数 |
| `hic_set_zero_force_safety_config(groupId, config)` | 设置零力示教相关安全策略 |
| `hic_set_collision_detection_config(groupId, config)` | 设置碰撞检测策略 |
| `hic_set_friction_compensation_config(groupId, config)` | 设置摩擦补偿参数 |
| `hic_set_dual_encoder_assist_config(groupId, config)` | 设置双编码器辅助参数 |

其中部分接口当前可能返回 `HIC_STATUS_ERROR_NOT_IMPLEMENTED`。这表示接口已经作为 ABI 预留，但内部算法链路尚未全部接通。

### 高频状态输入

高频状态输入接口应在每个控制周期调用，通常先更新状态，再读取输出命令。

| 接口 | 作用 |
| --- | --- |
| `hic_update_state_estimates(groupId, input)` | 更新控制器内部机器人状态估计 |
| `hic_update_raw_joint_torque_sensor(groupId, raw_torque_by_hardware_channel)` | 更新原始关节扭矩传感器数据，单位 `N.m` |
| `hic_update_wrench_sensor_data(groupId, input)` | 更新六维力/力矩传感器数据 |
| `hic_update_dual_encoder_joint_position(groupId, input)` | 更新双编码器关节位置，输入单位 `deg` |

`hic_update_state_estimates` 是最核心的周期输入接口。常用字段包括：

- `jointCount`：当前有效关节数。
- `commandJointPosition`：当前命令关节位置，输入单位 `deg`。
- `jointPosition`：当前反馈关节位置，输入单位 `deg`。
- `motorCurrent`：当前电机电流，单位 `A`。
- `currentTime`：当前时间戳，单位 `s`。

### 力控参数与目标

这些接口主要服务于零力示教和笛卡尔阻抗控制。

| 接口 | 作用 |
| --- | --- |
| `hic_set_cartesian_impedance_gains(groupId, gains)` | 设置 6 维笛卡尔阻抗刚度和阻尼 |
| `hic_set_force_control_nullspace_config(groupId, config)` | 设置零空间控制配置 |
| `hic_capture_current_joint_position_as_nullspace_target(groupId)` | 抓取当前关节位置作为零空间目标 |
| `hic_set_cartesian_fixed_position_target(groupId, target_position)` | 设置定点位置目标，长度 3，单位 `mm` |
| `hic_capture_current_position_as_fixed_target(groupId)` | 抓取当前末端位置作为定点位置目标 |
| `hic_set_cartesian_fixed_pose_target_zyx_euler(groupId, target_pose_zyx_euler)` | 设置定点位姿目标，格式 `[px, py, pz, rz, ry, rx]` |
| `hic_capture_current_pose_as_fixed_target(groupId)` | 抓取当前末端位姿作为定点位姿目标 |
| `hic_set_cartesian_trajectory_target_zyx_euler(groupId, target_pose_zyx_euler, target_velocity)` | 设置轨迹模式在线位姿和速度目标 |

`target_pose_zyx_euler` 的格式固定为：

```text
[px, py, pz, rz, ry, rx]
```

- `px / py / pz`：位置，单位 `mm`。
- `rz / ry / rx`：固定坐标系 Z-Y-X 欧拉角，单位 `deg`。

`target_velocity` 的格式固定为：

```text
[vx, vy, vz, wx, wy, wz]
```

- `vx / vy / vz`：线速度，单位 `mm/s`。
- `wx / wy / wz`：角速度，单位 `deg/s`。

### 力控模式

| 接口 | 作用 |
| --- | --- |
| `hic_start_force_control_mode(groupId, force_control_mode)` | 按 `HicForceControlMode` 启动力控子模式 |
| `hic_prepare_stop_force_control_mode(groupId)` | 请求退出当前力控模式 |

当前推荐使用统一入口 `hic_start_force_control_mode`，不要再扩展多个“启动某某模式”的旧式函数。

常见 `force_control_mode`：

- `HIC_FORCE_CONTROL_MODE_ZERO_FORCE`：零力示教。
- `HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION`：定点位置阻抗。
- `HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE`：定点位姿阻抗。
- `HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY`：轨迹笛卡尔阻抗。

`HIC_FORCE_CONTROL_MODE_NONE` 只表示当前无力控子模式，不应作为启动目标传入。

### 输出与状态读取

| 接口 | 作用 |
| --- | --- |
| `hic_get_force_control_current_commands(groupId, motor_current_commands, joint_protection_status)` | 获取当前力控电流命令，单位 `A` |
| `hic_get_force_control_torque_commands(groupId, joint_torque_commands, joint_protection_status)` | 获取当前力控力矩命令，单位 `N.m` |
| `hic_get_active_control_state(groupId, state_out)` | 获取当前主模式、力控子模式、输出能力和零空间状态 |
| `hic_get_robot_state(groupId, state_out)` | 获取内部机器人关节状态，输出按 SI 语义 |
| `hic_get_motion_constraint_status(groupId, status_out)` | 获取运动约束和保护状态 |
| `hic_get_estimated_dynamics_torques(groupId, torques_out)` | 获取内部动力学估计项 |
| `hic_get_current_cartesian_state(groupId, state_out)` | 获取当前末端位姿和速度 |
| `hic_get_force_control_mode(groupId)` | 获取当前力控子模式 |
| `hic_get_last_status(groupId)` | 获取最近一次 coordinator 返回状态码 |
| `hic_reset_control(groupId)` | 复位内部运行状态和缓存 |

读取输出命令时，调用方应同时检查返回值和 `joint_protection_status`。
当返回 `HIC_STATUS_ERROR_CURRENT_LIMIT` 时，输出数组通常仍可用，但已经经过限幅或保护处理。

### 独立计算与外设辅助

| 接口 | 作用 |
| --- | --- |
| `hic_compute_inverse_dynamics_torque(groupId, input, output)` | 独立计算逆动力学分项，不改变当前控制状态 |
| `hic_compute_gravity_torque(groupId, joint_position, gravity_torque)` | 独立计算重力项力矩 |
| `hic_compute_gravity_coriolis_torque(groupId, joint_position, joint_velocity, torque)` | 独立计算重力加科氏/离心项力矩 |
| `hic_enable_wrench_sensor_for_collision_detection(groupId, enable)` | 使能力传感器参与碰撞检测 |
| `hic_enable_wrench_sensor_for_force_feedforward(groupId, enable)` | 使能力传感器参与力控前馈 |

这些接口适合调试、离线验证或外设功能逐步接入。当前版本中部分实现仍可能返回 `HIC_STATUS_ERROR_NOT_IMPLEMENTED`。

### 典型调用流程

#### 零力示教

```c
const RTS_IEC_INT groupId = 0;

hic_initialize_control(groupId, &init_config);
hic_set_dynamics_linear_parameters(groupId, dynamic_params);
hic_set_motor_torque_conversion_parameters(groupId, kt, ratio, efficiency);
hic_set_joint_position_limits(groupId, &position_limits);
hic_set_joint_velocity_limits(groupId, &velocity_limits);
hic_set_joint_acceleration_limits(groupId, &acceleration_limits);
hic_set_joint_torque_limits(groupId, &torque_limits);
hic_set_motor_current_limits(groupId, &current_limits);

hic_update_state_estimates(groupId, &state_input);
hic_start_force_control_mode(groupId, HIC_FORCE_CONTROL_MODE_ZERO_FORCE);

while (running) {
    hic_update_state_estimates(groupId, &state_input);
    hic_get_force_control_current_commands(groupId, current_cmd, protection);
}

hic_prepare_stop_force_control_mode(groupId);
```

#### 定点位姿阻抗

```c
const RTS_IEC_INT groupId = 0;

hic_initialize_control(groupId, &init_config);
hic_set_cartesian_impedance_gains(groupId, &gains);

hic_update_state_estimates(groupId, &state_input);
hic_capture_current_pose_as_fixed_target(groupId);
hic_start_force_control_mode(groupId, HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);

while (running) {
    hic_update_state_estimates(groupId, &state_input);
    hic_get_force_control_torque_commands(groupId, torque_cmd, protection);
}

hic_prepare_stop_force_control_mode(groupId);
```

#### 轨迹阻抗

```c
const RTS_IEC_INT groupId = 0;

hic_initialize_control(groupId, &init_config);
hic_set_cartesian_impedance_gains(groupId, &gains);
hic_start_force_control_mode(groupId, HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY);

while (running) {
    hic_update_state_estimates(groupId, &state_input);
    hic_set_cartesian_trajectory_target_zyx_euler(groupId, target_pose, target_velocity);
    hic_get_force_control_torque_commands(groupId, torque_cmd, protection);
}

hic_prepare_stop_force_control_mode(groupId);
```

### 集成注意事项

- 初始化只负责建立控制器和加载基础机型参数，运行期限制、动力学参数、传感器配置应继续通过各个 `hic_set_xxx` 接口下发。
- 高频循环建议顺序固定为：先 `hic_update_state_estimates`，再读取输出命令。
- 所有输出数组由调用方分配，长度按 `HIC_MAX_JOINTS` 准备。
- 所有结构体建议先清零再填字段，避免未初始化字段影响后续扩展。
- `groupId` 当前建议统一传 `0`。
- 返回非 `HIC_STATUS_OK` 时，调用方应停止使用本周期输出，除非接口文档明确说明输出仍可用。
