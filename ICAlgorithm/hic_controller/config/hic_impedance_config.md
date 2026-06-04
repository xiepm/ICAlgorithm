# 阻抗控制调参接口说明

发送格式统一为：

```text
命令名,参数1,参数2,...;
```

参数之间使用英文逗号 `,` 分隔，命令末尾使用英文分号 `;` 结束。

关节编号从 1 开始，`jointIndex = 1` 表示第 1 关节。除全局参数和笛卡尔参数外，单关节参数均带 `jointIndex`。

命名约定：

- 后缀 `_current` 表示该量来自电机电流反推，例如 `motorEstimatedTorque_current`、`externalTorque_current`。
- 后缀 `_sensor` 表示该量来自传感器或实测力矩，例如 `jointMeasuredTorque_sensor`、`measuredTorqueFilterAlpha_sensor`。
- `modelTorque_current` 表示使用电流力矩整定参数计算出的动力学模型力矩。
- `modelTorque_sensor` 表示使用扭矩传感器整定参数计算出的动力学模型力矩。
- 外力矩估计关系为：`externalTorque_current = motorEstimatedTorque_current - modelTorque_current`。
- 外力矩估计关系为：`externalTorque_sensor = jointMeasuredTorque_sensor - modelTorque_sensor`。
- 关节阻抗里的 `enableExternalTorqueCompensation` 是功能开关，不表示数据来源，因此保持原名。

## 一、基础说明

### 1. impedance_setTimeCycle

描述：设置控制器周期，属于全局参数，不区分关节

对应 C API：`hic_initialize_control` 中的 `controlPeriod`

发送格式：

```text
impedance_setTimeCycle,timeCycleMs;
```

参数1：控制器周期，int，单位：ms，可选 1ms/4ms

示例：

```text
impedance_setTimeCycle,1;
```

### 2. impedance_setPhysicalParams

描述：设置指定关节的电机到关节力矩换算参数

对应 C API：`hic_set_motor_torque_conversion_parameters`

发送格式：

```text
impedance_setPhysicalParams,jointIndex,K,N,efficiency,reserved1,reserved2;
```

参数1：关节编号 jointIndex，int，范围：1..jointCount

参数2：力矩常数 K，double，单位：N.m/A

参数3：减速比 N，double，默认 1

参数4：传动效率 efficiency，double，范围通常为 0..1

参数5：预留，double，默认 0

参数6：预留，double，默认 0

示例：

```text
impedance_setPhysicalParams,1,0.1,100,1,0,0;
```

### 3. impedance_setDynamicsLinearParams

描述：设置通用动力学线性参数。该接口用于兼容旧流程，会同时更新 current/sensor 两套动力学参数。

对应 C API：`hic_set_dynamics_linear_parameters`

发送格式：

```text
impedance_setDynamicsLinearParams,p1,p2,...;
```

参数：动力学线性参数数组，double，长度不超过 `HIC_MAX_DYNAMIC_PARAMS`

### 4. impedance_setDynamicsLinearParams_current

描述：设置基于电流力矩整定得到的动力学线性参数，用于计算 `modelTorque_current`

对应 C API：`hic_set_dynamics_linear_parameters_current`

发送格式：

```text
impedance_setDynamicsLinearParams_current,p1,p2,...;
```

参数：动力学线性参数数组，double，长度不超过 `HIC_MAX_DYNAMIC_PARAMS`

### 5. impedance_setDynamicsLinearParams_sensor

描述：设置基于扭矩传感器整定得到的动力学线性参数，用于计算 `modelTorque_sensor`

对应 C API：`hic_set_dynamics_linear_parameters_sensor`

发送格式：

```text
impedance_setDynamicsLinearParams_sensor,p1,p2,...;
```

参数：动力学线性参数数组，double，长度不超过 `HIC_MAX_DYNAMIC_PARAMS`

## 二、关节阻抗参数

### 1. impedance_setJointImpedanceParams

描述：设置指定关节的关节空间阻抗参数

对应 C API：`hic_set_single_joint_impedance_config`

说明：该命令只修改一个关节的阻抗参数，其他关节保持当前配置不变。`enableExternalTorqueCompensation` 是整组关节阻抗配置的全局开关，调用该命令会同步更新该开关。

发送格式：

```text
impedance_setJointImpedanceParams,jointIndex,stiffness,damping,targetPositionDeg,targetVelocityRadPerSec,targetAccelerationRadPerSec2,enableExternalTorqueCompensation,externalTorqueSource;
```

参数1：关节编号 jointIndex，int，范围：1..jointCount

参数2：关节刚度 stiffness，double，单位：N.m/rad

参数3：关节阻尼 damping，double，单位：N.m.s/rad

参数4：目标关节位置 targetPositionDeg，double，单位：deg

参数5：目标关节速度 targetVelocityRadPerSec，double，单位：rad/s

参数6：目标关节加速度 targetAccelerationRadPerSec2，double，单位：rad/s^2

参数7：是否启用外力矩补偿 enableExternalTorqueCompensation，int，0=关闭，1=开启

参数8：外力矩补偿来源 externalTorqueSource，int，0=current，1=sensor

示例：

```text
impedance_setJointImpedanceParams,1,200,28,0,0,0,1,1;
```

### 2. impedance_setAllJointImpedanceParams

描述：一次性设置整组关节空间阻抗参数

对应 C API：`hic_set_joint_impedance_config`

发送格式：

```text
impedance_setAllJointImpedanceParams,enableExternalTorqueCompensation,externalTorqueSource,joint1K,joint1D,joint1TargetPositionDeg,joint1TargetVelocityRadPerSec,joint1TargetAccelerationRadPerSec2,...;
```

参数1：是否启用外力矩补偿 enableExternalTorqueCompensation，int，0=关闭，1=开启

参数2：外力矩补偿来源 externalTorqueSource，int，0=current，1=sensor

随后每个关节 5 个参数：

```text
stiffness,damping,targetPositionDeg,targetVelocityRadPerSec,targetAccelerationRadPerSec2
```

说明：该命令适合一次性下发所有关节阻抗配置；如果只改一个关节，优先使用 `impedance_setJointImpedanceParams`。

### 3. impedance_captureCurrentJointAsTarget

描述：抓取当前滤波后的关节位置作为关节阻抗平衡点

对应 C API：`hic_capture_current_joint_position_as_impedance_target`

发送格式：

```text
impedance_captureCurrentJointAsTarget;
```

参数：无

示例：

```text
impedance_captureCurrentJointAsTarget;
```

## 三、笛卡尔阻抗参数

### 1. impedance_setCartesianGains

描述：设置笛卡尔 6 维阻抗刚度和阻尼

对应 C API：`hic_set_cartesian_impedance_gains`

发送格式：

```text
impedance_setCartesianGains,Kx,Ky,Kz,Krx,Kry,Krz,Dx,Dy,Dz,Drx,Dry,Drz;
```

参数1-参数6：笛卡尔刚度 `[Kx, Ky, Kz, Krx, Kry, Krz]`

参数7-参数12：笛卡尔阻尼 `[Dx, Dy, Dz, Drx, Dry, Drz]`

位置方向刚度单位：N/m

姿态方向刚度单位：N.m/rad

位置方向阻尼单位：N.s/m

姿态方向阻尼单位：N.m.s/rad

示例：

```text
impedance_setCartesianGains,200,200,200,20,20,20,30,30,30,5,5,5;
```

### 2. impedance_setCartesianFixedPositionTarget

描述：设置笛卡尔定点位置目标

对应 C API：`hic_set_cartesian_fixed_position_target`

发送格式：

```text
impedance_setCartesianFixedPositionTarget,xMm,yMm,zMm;
```

参数1-参数3：目标位置 `[x, y, z]`，double，单位：mm

示例：

```text
impedance_setCartesianFixedPositionTarget,300,0,450;
```

### 3. impedance_captureCurrentPositionAsFixedTarget

描述：抓取当前末端位置作为定点位置目标

对应 C API：`hic_capture_current_position_as_fixed_target`

发送格式：

```text
impedance_captureCurrentPositionAsFixedTarget;
```

参数：无

示例：

```text
impedance_captureCurrentPositionAsFixedTarget;
```

### 4. impedance_setCartesianFixedPoseTarget

描述：设置笛卡尔定点位姿目标，姿态采用固定坐标系 Z-Y-X 欧拉角

对应 C API：`hic_set_cartesian_fixed_pose_target_zyx_euler`

发送格式：

```text
impedance_setCartesianFixedPoseTarget,pxMm,pyMm,pzMm,rzDeg,ryDeg,rxDeg;
```

参数1-参数3：目标位置 `[px, py, pz]`，double，单位：mm

参数4-参数6：目标姿态 `[rz, ry, rx]`，double，单位：deg

示例：

```text
impedance_setCartesianFixedPoseTarget,300,0,450,0,0,0;
```

### 5. impedance_captureCurrentPoseAsFixedTarget

描述：抓取当前末端位姿作为定点位姿目标

对应 C API：`hic_capture_current_pose_as_fixed_target`

发送格式：

```text
impedance_captureCurrentPoseAsFixedTarget;
```

参数：无

示例：

```text
impedance_captureCurrentPoseAsFixedTarget;
```

### 6. impedance_setCartesianTrajectoryTarget

描述：设置轨迹模式下的在线位姿和速度目标，姿态采用固定坐标系 Z-Y-X 欧拉角

对应 C API：`hic_set_cartesian_trajectory_target_zyx_euler`

发送格式：

```text
impedance_setCartesianTrajectoryTarget,pxMm,pyMm,pzMm,rzDeg,ryDeg,rxDeg,vxMmPerSec,vyMmPerSec,vzMmPerSec,wxDegPerSec,wyDegPerSec,wzDegPerSec;
```

参数1-参数6：目标位姿 `[px, py, pz, rz, ry, rx]`，位置单位 mm，姿态单位 deg

参数7-参数12：目标速度 `[vx, vy, vz, wx, wy, wz]`，线速度单位 mm/s，角速度单位 deg/s

示例：

```text
impedance_setCartesianTrajectoryTarget,300,0,450,0,0,0,10,0,0,0,0,0;
```

## 四、零空间参数

### 1. impedance_setNullspaceParams

描述：设置指定关节的零空间目标位置、刚度和阻尼

对应 C API：`hic_set_force_control_nullspace_config`

发送格式：

```text
impedance_setNullspaceParams,jointIndex,enabled,targetJointPositionRad,stiffness,damping;
```

参数1：关节编号 jointIndex，int，范围：1..jointCount

参数2：是否启用零空间控制 enabled，int，0=关闭，1=开启

参数3：零空间目标关节位置 targetJointPositionRad，double，单位：rad

参数4：零空间刚度 stiffness，double，单位：N.m/rad

参数5：零空间阻尼 damping，double，单位：N.m.s/rad

示例：

```text
impedance_setNullspaceParams,1,1,0,2,0.2;
```

### 2. impedance_captureCurrentJointAsNullspaceTarget

描述：抓取当前关节位置作为零空间目标

对应 C API：`hic_capture_current_joint_position_as_nullspace_target`

发送格式：

```text
impedance_captureCurrentJointAsNullspaceTarget;
```

参数：无

示例：

```text
impedance_captureCurrentJointAsNullspaceTarget;
```

## 五、展开阻抗参数

### 1. impedance_setFlattenedParams

描述：一次性设置展开后的阻抗性能参数

对应 C API：`hic_set_impedance_parameters`

发送格式：

```text
impedance_setFlattenedParams,p0,p1,p2,...;
```

参数布局固定为：

```text
[0] enableExternalTorqueCompensation
随后每关节 2 个参数：[jointStiffness, jointDamping]
随后 6 个笛卡尔刚度：[Kx, Ky, Kz, Krx, Kry, Krz]
随后 6 个笛卡尔阻尼：[Dx, Dy, Dz, Drx, Dry, Drz]
最后每关节 3 个零空间参数：[targetJointPosition, stiffness, damping]
```

参数数量：

```text
1 + HIC_MAX_JOINTS * 2 + 6 * 2 + HIC_MAX_JOINTS * 3
```

说明：该接口只更新影响阻抗性能的参数，不修改关节阻抗目标位置、目标速度、目标加速度，也不修改零空间 enabled 状态。

## 六、摩擦补偿参数

### 1. impedance_setFrictionCompensation

描述：设置指定关节的摩擦补偿参数

对应 C API：`hic_set_friction_compensation_config`

发送格式：

```text
impedance_setFrictionCompensation,jointIndex,enabled,frictionCompensationFactor,dynamicFrictionCompensationFactor,frictionLowVelocityThreshold,actuatorDamping;
```

参数1：关节编号 jointIndex，int，范围：1..jointCount

参数2：是否启用摩擦补偿 enabled，int，0=关闭，1=开启

参数3：静态/基础摩擦补偿系数 frictionCompensationFactor，double

参数4：动态摩擦补偿系数 dynamicFrictionCompensationFactor，double

参数5：低速摩擦判断阈值 frictionLowVelocityThreshold，double，单位：rad/s

参数6：执行器等效阻尼 actuatorDamping，double，单位：N.m.s/rad

示例：

```text
impedance_setFrictionCompensation,1,1,0.2,0.1,0.02,0.01;
```

## 七、安全与限幅参数

### 1. impedance_setJointPositionLimits

描述：设置关节位置上下界

对应 C API：`hic_set_joint_position_limits`

发送格式：

```text
impedance_setJointPositionLimits,jointIndex,lowerDeg,upperDeg;
```

参数2-参数3：位置下限/上限，单位：deg

### 2. impedance_setJointVelocityLimit

描述：设置关节速度最大绝对值

对应 C API：`hic_set_joint_velocity_limits`

发送格式：

```text
impedance_setJointVelocityLimit,jointIndex,maxAbsDegPerSec;
```

参数2：速度最大绝对值，单位：deg/s

### 3. impedance_setJointAccelerationLimit

描述：设置关节加速度最大绝对值

对应 C API：`hic_set_joint_acceleration_limits`

发送格式：

```text
impedance_setJointAccelerationLimit,jointIndex,maxAbsDegPerSec2;
```

参数2：加速度最大绝对值，单位：deg/s^2

### 4. impedance_setJointTorqueLimits

描述：设置关节力矩上下界

对应 C API：`hic_set_joint_torque_limits`

发送格式：

```text
impedance_setJointTorqueLimits,jointIndex,lowerNm,upperNm;
```

参数2-参数3：力矩下限/上限，单位：N.m

### 5. impedance_setMotorCurrentLimits

描述：设置电机电流上下界

对应 C API：`hic_set_motor_current_limits`

发送格式：

```text
impedance_setMotorCurrentLimits,jointIndex,lowerA,upperA;
```

参数2-参数3：电流下限/上限，单位：A

## 八、力控模式

### 1. impedance_startForceControlMode

描述：启动指定力控模式

对应 C API：`hic_start_force_control_mode`

发送格式：

```text
impedance_startForceControlMode,forceControlMode;
```

参数1：力控模式 forceControlMode，int

可选值：

```text
1 = HIC_FORCE_CONTROL_MODE_ZERO_FORCE
2 = HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION
3 = HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE
4 = HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY
5 = HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE
```

示例：

```text
impedance_startForceControlMode,5;
```

### 2. impedance_prepareStopForceControlMode

描述：准备退出当前力控模式

对应 C API：`hic_prepare_stop_force_control_mode`

发送格式：

```text
impedance_prepareStopForceControlMode;
```

参数：无

示例：

```text
impedance_prepareStopForceControlMode;
```

## 九、发送示例

关节阻抗模式常用发送顺序：

```text
impedance_setTimeCycle,1;
impedance_setPhysicalParams,1,0.1,100,1,0,0;
impedance_setJointImpedanceParams,1,200,28,0,0,0,1;
impedance_setJointTorqueLimits,1,-80,80;
impedance_setMotorCurrentLimits,1,-10,10;
impedance_startForceControlMode,5;
```

笛卡尔定点位姿阻抗常用发送顺序：

```text
impedance_setTimeCycle,1;
impedance_setCartesianGains,200,200,200,20,20,20,30,30,30,5,5,5;
impedance_captureCurrentJointAsNullspaceTarget;
impedance_setCartesianFixedPoseTarget,300,0,450,0,0,0;
impedance_startForceControlMode,3;
```
