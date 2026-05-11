# `hic_controller/include/hic_controller` 逻辑框架

这个目录是 `hic_controller` 库真正的“公开头文件主体”。  
如果把整个项目看成一条完整控制链，它大致是下面这个关系：

```text
外部上位机 / CLI / PLC / Demo
            |
            v
interface/hic_c_api.h
            |
            v
coordinator/hic_control_coordinator.h
      |         |         |         |
      v         v         v         v
   state/    sensing/  control/  kinematics/
                            |
                            v
                        dynamics/
            |
            v
types/ 提供所有公共枚举、配置和状态结构体
```

## 先记住一个原则

- `interface/` 是外部正式入口。
- `coordinator/` 是内部总协调器。
- `control/` 是算法核，不负责完整控制闭环。
- `kinematics/` 和 `dynamics/` 是机型后端适配层。
- `state/` 和 `sensing/` 是输入观测层。
- `types/` 是全项目公共语言。

## 如果你是不同角色，最推荐看哪里

### 1. 外部集成方

你只想“能调起来、能送状态、能拿输出”，建议主要看：

1. `types/`
2. `interface/`
3. `test/` 里的 demo

通常不需要直接依赖 `coordinator/`、`control/` 或某个具体动力学类。

### 2. 算法开发者

你想改控制逻辑，建议看：

1. `coordinator/`
2. `control/`
3. `state/`
4. `kinematics/`
5. `dynamics/`

### 3. 机型适配开发者

你想新增一个 robotType，建议看：

1. `dynamics/`
2. `kinematics/`
3. `coordinator/`
4. `types/`

## 当前项目里的主控制思路

当前版本顶层控制模式只有两类：

- `IDLE`
- `FORCE_CONTROL`

其中 `FORCE_CONTROL` 内部再细分为：

- 零力示教
- 定点阻抗
- 轨迹阻抗

这意味着：

- “零力示教”和“阻抗模式”不是平级顶层模式。
- 它们都属于“力控大类”。
- 输出形式再分成：
  - 电流环命令
  - 力矩环命令

## 文档阅读方式建议

这个目录下每个子目录我都放了一个 `README.md`。  
建议你按下面顺序读：

1. `types/README.md`
2. `interface/README.md`
3. `coordinator/README.md`
4. 其余目录按需要查阅
