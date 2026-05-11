# `dynamics/` 使用文档

## 这个目录是做什么的

`dynamics/` 负责“机器人动力学相关能力”的统一接口和机型适配。  
这里既有抽象接口，也有不同机型的具体动力学类声明。

## 你应该先看哪个文件

优先看这两个：

- `hic_dynamics_base.h`
- `hic_dynamics_adapter.h`

如果你是想知道“某个 robotType 最后会落到哪个机型实现”，  
再去看具体机型类头文件，例如：

- `elfinDynamics.h`
- `urDynamics.h`
- `palletDynamics.h`
- `dsDynamics.h`
- `sevendofDynamics.h`
- `anthorDynamics.h`

## 当前目录里的层次

### 1. base

- `hic_dynamics_base.h`

它定义动力学后端应该提供哪些基础能力。

### 2. adapter

- `hic_dynamics_adapter.h`

它负责把“统一调用接口”转发到“具体机型动力学实现”。

外部通常不关心具体是 `elfin` 还是 `ur`，  
只要传 `robotType`，adapter 会内部选择正确 backend。

### 3. 具体机型类

这些类是真正承载某个机型动力学模型的地方。  
例如：

- 重力项
- 科氏项
- 质量矩阵
- 摩擦项

## `HicDynamicsAdapter` 负责什么

它目前主要负责：

- 根据 `robotType` 选择后端
- 初始化动力学 backend
- 在线更新动力学参数
- 设置重力向量
- 设置 payload 质量和质心
- 输出：
  - 重力力矩
  - 科氏力矩
  - 质量矩阵
  - 摩擦力矩

## 这个目录最常见的使用场景

### 场景 1：零力示教

零力示教最依赖的是：

- `computeGravityTorque`

因为第一版零力示教主逻辑本质上是：

- 重力补偿
- 再叠加一点速度阻尼

### 场景 2：阻抗控制中的动力学补偿

如果后续要增强阻抗控制，`dynamics/` 也可以继续为：

- 重力补偿
- 科氏补偿
- 更复杂模型项

提供基础。

## `robotType` 与机型后端的关系

当前项目里，adapter 会根据 `robotType` 选择不同后端。  
例如你之前关心的 `robotType = 20`，现在对应的是：

- 动力学：`sevendofDynamics`

这意味着：

- 只要初始化时 `robotType = 20`
- 后续 coordinator 通过 adapter 计算重力力矩时
- 最终就会动态派发到 `sevendofDynamics`

## 新增一个新机型通常怎么做

一般步骤是：

1. 新增一个机型动力学类头文件和实现
2. 让它符合 base 约定
3. 在 adapter 里按 `robotType` 接进去
4. 在测试里给这个机型准备合适的参数和状态输入

## 一句总结

`dynamics/` 解决的是“这台机器人的动力学怎么算”，  
而不是“整条控制链路怎么跑”。
