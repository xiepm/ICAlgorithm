# `types/` 使用文档

## 这个目录是做什么的

`types/` 是整个项目的公共语言层。  
这里定义了：

- 状态码
- 控制模式枚举
- 力控子模式枚举
- 维度常量
- 基础配置结构体
- 机器人状态结构体
- 笛卡尔状态结构体

如果没有这层，项目各模块就很容易各说各话。

## 当前目录里的两个核心头文件

- `hic_types.h`
- `hic_config.h`

## `hic_types.h` 主要放什么

### 1. 状态码 `HicStatus`

这是整个项目统一返回值体系。  
不管你是：

- C API
- coordinator
- 状态观测器
- 动力学/运动学适配器

基本都应该复用这套状态码。

### 2. 顶层控制模式 `HicControlMode`

当前只有两类：

- `IDLE`
- `FORCE_CONTROL`

### 3. 力控子模式 `HicForceControlSubMode`

这是现在最值得上层直接关注的模式枚举：

- `NONE`
- `ZERO_FORCE`
- `IMPEDANCE_FIXED`
- `IMPEDANCE_TRAJECTORY`

### 4. 输出环路类型 `HicCommandLoopType`

用于描述当前输出的是：

- 电流环命令
- 力矩环命令

### 5. 维度常量 `HicDimensions`

这里定义了很多固定容量常量，例如：

- `HIC_MAX_JOINTS = 16`
- `HIC_CARTESIAN_DIM = 6`
- `HIC_POSE_DIM = 7`

要特别注意：

- `HIC_MAX_JOINTS` 是“数组容量上限”
- 不是“机器人真实自由度”
- 真实自由度由 `jointCount` 决定

## `hic_config.h` 主要放什么

### 1. `HicControlConfig`

这是初始化控制器时最重要的总配置。  
里面通常包括：

- 关节数
- 控制周期
- robotType
- 运动学/动力学参数
- 力矩换算参数
- 关节速度/加速度/位置/力矩/电流限制

### 2. `HicImpedanceGains`

阻抗控制的刚度和阻尼。

### 3. `HicRobotStateObserverConfig`

状态观测器的滤波和边界配置。

### 4. `HicTorqueSensorConfig`

扭矩传感器的映射、滤波、故障检查配置。

### 5. `HicRobotState`

标准化关节状态输出。

### 6. `HicCartesianState`

标准化末端状态输出。

## 新同事最容易搞混的几个点

### 1. `jointCount` 和 `HIC_MAX_JOINTS`

- `jointCount` 是当前机器人的真实关节数
- `HIC_MAX_JOINTS` 是数组容量上限

### 2. `HicControlMode` 和 `HicForceControlSubMode`

- 前者是顶层大类
- 后者是力控内部细分类

### 3. `HicTargetMode` 和 `HicForceControlSubMode`

- `HicTargetMode` 只描述阻抗目标是固定点还是轨迹
- 它不是顶层控制模式

## 一句总结

`types/` 是全工程共享的“词典”和“语法”，  
不先看懂这里，读别的目录会一直混概念。
