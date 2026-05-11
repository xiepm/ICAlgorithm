# `hic_controller/include` 使用文档

这个目录是整个 `hic_controller` 工程对外暴露头文件的入口层，主要分成两部分：

- `hic_controller/`
  放项目自己的正式头文件，按功能目录拆分。
- 顶层兼容头文件
  例如 `hic_common_types.h`、`hic_frames.hpp`，主要用于兼容旧代码的 include 路径。

## 建议阅读顺序

如果你是第一次接触这个项目，推荐按下面顺序看：

1. `hic_controller/types/`
   先认识所有基础类型、状态码、配置结构体。
2. `hic_controller/interface/`
   看正式对外 C API，理解外部系统应该怎么调用。
3. `hic_controller/coordinator/`
   看内部总协调器，理解整个控制链路是如何串起来的。
4. `hic_controller/state/`、`sensing/`
   看状态输入和传感器/外力观测链路。
5. `hic_controller/control/`
   看阻抗控制核心本身的数学职责。
6. `hic_controller/kinematics/`、`dynamics/`
   看不同机型的运动学/动力学后端是如何被适配进来的。

## 顶层两个兼容头文件

### `hic_common_types.h`

- 这是一个兼容入口头文件。
- 它本身不定义新类型，只是转发到 `src/common/hic_common_types.h`。
- 新代码如果只使用控制器公开接口，通常不需要直接包含它。

### `hic_frames.hpp`

- 这也是一个兼容入口头文件。
- 它本身不实现新功能，只是转发到 `src/common/hic_frames.hpp`。
- 主要用于旧代码或已有几何/坐标系工具链兼容。

## 新代码最推荐的包含方式

如果你是“外部集成方”或“测试程序作者”，最推荐从这里开始：

```cpp
#include "hic_controller/interface/hic_c_api.h"
```

如果你是“库内部开发者”，通常还会进一步包含：

```cpp
#include "hic_controller/types/hic_types.h"
#include "hic_controller/types/hic_config.h"
#include "hic_controller/coordinator/hic_control_coordinator.h"
```

## 目录职责概览

- `control/`
  控制算法核心，目前最重要的是笛卡尔阻抗数学。
- `coordinator/`
  内部总调度层，把状态、运动学、动力学、阻抗、安全和输出串成完整链路。
- `dynamics/`
  动力学统一接口和不同机型的动力学后端。
- `interface/`
  正式对外 C API。
- `kinematics/`
  运动学统一接口和不同机型的运动学后端。
- `sensing/`
  交互力/扭矩传感器相关观测。
- `state/`
  机器人本体状态观测，包括位置、速度、加速度、电流、估计力矩。
- `types/`
  全项目通用类型、状态码和配置结构体。
