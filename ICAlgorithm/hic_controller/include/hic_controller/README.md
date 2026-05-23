# `hic_controller/include/hic_controller` 逻辑框架

这个目录是 `hic_controller` 库的公开头文件主体。

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
```

## 原则

- `interface/hic_c_api.h` 是唯一公共入口，集中定义导出宏、状态码、配置、状态结构体和 C API 函数。
- `coordinator/` 是内部总协调器。
- `control/` 是算法核心，不负责完整控制闭环。
- `kinematics/` 和 `dynamics/` 是机型后端适配层。
- `state/` 和 `sensing/` 是输入观测层。

## 推荐阅读顺序

1. `interface/README.md`
2. `coordinator/README.md`
3. `state/README.md`、`sensing/README.md`
4. `control/README.md`
5. `kinematics/README.md`、`dynamics/README.md`

外部集成代码通常只需要：

```cpp
#include "hic_controller/interface/hic_c_api.h"
```
