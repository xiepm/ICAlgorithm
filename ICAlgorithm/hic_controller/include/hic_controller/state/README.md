# `state/` 使用文档

## 这个目录是做什么的

`state/` 负责机器人本体状态观测�? 
当前核心文件�?

- `hic_robot_state_observer.h`

如果�?`sensing/` 更偏交互力，  
�?`state/` 更偏“这台机器人自己现在是什么状态”�?

## 当前核心�?

### `HicRobotStateObserver`

它统一管理下面这些量：

- 关节位置
- 关节速度
- 关节加速度
- 电机电流
- 关节测量力矩
- 电机估计力矩

并提供：

- 有限值检�?
- 范围检�?
- 一阶滤�?
- 基于时间戳的速度/加速度差分估计
- 电流到估计力矩的换算

## 这个目录在整条控制链里的位置

典型链路是：

```text
外部送入 jointPosition / motorCurrent / currentTime
  -> state observer
  -> 得到 jointVelocity / jointAcceleration / motorEstimatedTorque_current
  -> coordinator 再拿这些量去做控�?
```

## 它最常见的输入接�?

### `updateRobotState(...)`

这是最常用的便捷入口�? 
当外部只有：

- 当前关节位置
- 当前电机电流
- 当前时间�?

时，状态观测器会内部估计：

- 关节速度
- 关节加速度
- 基于电机参数的估计力�?

这也是为什么测试例里通常只要提供位置、电流和时间，就能先把流程跑起来�?

## 这个目录最常见的输�?

### `getRobotState(...)`

输出标准化的 `HicRobotState`，包括：

- `jointPosition`
- `jointVelocity`
- `jointAcceleration`
- `motorCurrent`
- `jointMeasuredTorque_sensor`
- `motorEstimatedTorque_current`

## 它和零力示教有什么关�?

零力示教第一版非常依赖这里的结果，尤其是�?

- `jointPosition`
- `jointVelocity`
- `motorCurrent`

因为安全进入检测里会检查：

- 状态有没有更新
- 数据是不是有限�?
- 速度是不是过�?
- 位置是不是接近硬限位

## 它和定点阻抗有什么关�?

阻抗模式也依赖它，但依赖链更长�? 
阻抗模式会在此基础上继续经过：

- kinematics
- control

所以如果阻抗结果不对，不一定是 `state/` 的问题，�?`state/` 永远是第一层输入基础�?

## 一句总结

`state/` 是整个控制系统的“机器人自身状态入口”，  
没有可靠状态，后面的控制几乎都站不住�?
