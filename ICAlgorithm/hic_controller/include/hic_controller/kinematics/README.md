# `kinematics/` 使用文档

## 这个目录是做什么的

`kinematics/` 负责机器人运动学相关能力，例如：

- 正运动学 FK
- Jacobian
- 末端 twist

它不做动力学补偿，也不做电流/力矩输出。

## 你应该先看哪个文件

优先看：

- `hic_kinematics_base.h`
- `hic_kinematics_adapter.h`

如果你要看具体机型，再看：

- `elfinKinematics.h`
- `URKinematics.h`
- `palletKinematics.h`
- `DSKinematics.h`

## 当前目录里的层次

### 1. base

定义运动学后端需要提供的统一能力。

### 2. adapter

屏蔽具体机型差异，对外只暴露统一接口。

### 3. 具体机型

真正实现不同机器人结构对应的运动学求解。

## `HicKinematicsAdapter` 负责什么

主要负责：

- 根据 `robotType` 选择 backend
- 正运动学计算
- Jacobian 计算
- 末端 twist 计算

## 哪些模块会依赖这个目录

### 1. 阻抗控制

阻抗模式强依赖运动学，因为要知道：

- 当前末端位姿
- 当前末端速度
- 当前 Jacobian

如果没有这些，阻抗核就无法正常工作。

### 2. 当前末端状态读取

当外部想获取当前笛卡尔状态时，也会经过这里。

## 零力示教是否依赖这个目录

零力示教第一版主要靠动力学重力补偿和关节阻尼，  
所以对运动学依赖相对较弱。

这也是为什么：

- 零力示教通常更容易先打通
- 阻抗模式往往更依赖机型运动学是否正确

## 你之前关心的 7 轴机型

如果 `robotType = 20`，当前项目已经把它接到：

- 动力学：`sevendofDynamics`
- 运动学：`DSKinematics`

所以如果你后面发现：

- 零力示教表现基本正常
- 但定点阻抗表现不合理

优先要怀疑的通常就是 `kinematics/` 这条链是否真的适配了这台 7 轴机型。

## 一句总结

`kinematics/` 回答的是“机器人末端现在在哪里、怎么动”，  
而不是“该输出多大电流或力矩”。
