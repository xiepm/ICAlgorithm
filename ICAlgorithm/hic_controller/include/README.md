# `hic_controller/include` 使用文档

这个目录是 `hic_controller` 对外暴露头文件的入口层。

## 推荐包含方式

外部集成方、测试程序和库内部需要公共 ABI 类型时，都优先包含一个头文件：

```cpp
#include "hic_controller/interface/hic_c_api.h"
```

`hic_c_api.h` 现在同时包含导出宏、状态码、模式枚举、固定维度常量、配置结构体、状态结构体、传感器结构体和 C API 函数声明。

## 目录职责

- `interface/`：正式对外 C API。`hic_c_api.h` 是唯一公共入口。
- `coordinator/`：内部总协调层，把状态、运动学、动力学、阻抗、安全和输出串成完整链路。
- `control/`：控制算法核心，目前主要是笛卡尔阻抗控制。
- `kinematics/`：运动学统一接口和不同机型的运动学后端。
- `dynamics/`：动力学统一接口和不同机型的动力学后端。
- `sensing/`：交互力、扭矩传感器相关观测。
- `state/`：机器人本体状态观测。

顶层兼容头 `hic_common_types.h`、`hic_frames.hpp` 仍用于旧路径兼容；新代码通常不需要直接包含它们。
