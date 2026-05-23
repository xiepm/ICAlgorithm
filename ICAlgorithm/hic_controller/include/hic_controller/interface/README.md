# `interface/` 使用文档

`interface/` 是项目正式对外的 C API 层。外部上位机、PLC、Python/C/C++ 调用方、demo 和测试程序都建议只依赖：

```cpp
#include "hic_controller/interface/hic_c_api.h"
```

`hic_c_api.h` 已经合并了原先分散在 `hic_export.h`、`hic_types.h`、`hic_config.h`、`hic_c_api_types.h` 中的公共内容。旧头文件已删除，后续不要再新增对 `hic_controller/types/...` 的包含。

## API 约定

- 所有接口返回 `int`，语义对应 `HicStatus`。
- `groupId` 当前建议统一传 `0`，为后续多实例控制预留。
- 内部计算统一使用 SI 单位。
- 外部输入中，关节角度通常按 `deg`，关节角速度按 `deg/s`，笛卡尔位置按 `mm`，接口边界会在 `hic_c_api.cpp` 中换算到内部 SI 单位。
- 输出接口默认返回内部 SI 语义，除非函数注释另有说明。

## 常用流程

```text
hic_initialize_control
 -> hic_set_dynamics_linear_parameters
 -> hic_set_motor_torque_conversion_parameters
 -> hic_set_joint_position_limits
 -> hic_set_joint_velocity_limits
 -> hic_set_joint_acceleration_limits
 -> hic_set_joint_torque_limits
 -> hic_set_motor_current_limits
 -> 每周期 hic_update_state_estimates
 -> 按模式读取 hic_get_force_control_current_commands 或 hic_get_force_control_torque_commands
```

需要看真实调用方式时，优先参考：

- `hic_controller/test/hic_controller_cli.cpp`
- `hic_controller/test/hic_impedance_demo.cpp`
- `hic_controller/test/hic_state_generator.cpp`
