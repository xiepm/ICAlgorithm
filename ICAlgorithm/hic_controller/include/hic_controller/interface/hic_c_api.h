#pragma once

/// @file hic_c_api.h
/// @brief HIC 对外 C API 入口。
/// @note 本文件是对外唯一公共入口，集中定义导出宏、公共类型和函数声明。
/// @note 内部 coordinator、observer、kinematics、dynamics 统一使用 SI 单位：
/// 位置/长度使用 m，角度使用 rad，角速度使用 rad/s，角加速度使用 rad/s^2，
/// 力矩使用 N.m，电流使用 A，时间使用 s。
/// @note 对外输入接口中，若参数天然来自上位机或旧系统，位置/长度相关量可按 mm 输入，
/// 关节角度相关量可按 deg 输入，hic_c_api.cpp 会在输入边界完成到 SI 的换算。
/// @note 对外输出接口默认继续返回内部 SI 语义，除非某个接口的注释另有明确说明。

#include <stdint.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#if defined(_WIN32) || defined(_WIN64)
#if defined(HIC_BUILD_SHARED)
#define HIC_EXPORT __declspec(dllexport)
#else
#define HIC_EXPORT __declspec(dllimport)
#endif
#else
#define HIC_EXPORT __attribute__((visibility("default")))
#endif

typedef int8_t RTS_IEC_BOOL;
typedef int8_t RTS_BOOL;
typedef int16_t RTS_IEC_INT;
typedef int64_t RTS_IEC_LINT;
typedef double RTS_IEC_LREAL;
typedef uint16_t RTS_IEC_UINT;
typedef int32_t RTS_IEC_DINT;

/// @brief C API 统一返回状态码。
/// @note 所有导出函数均返回该枚举值，0 表示成功，非 0 表示不同类型的错误。
typedef enum HicStatus
{
	HIC_STATUS_OK = 0,                 ///< 调用成功。
	HIC_STATUS_ERROR_INIT = 1,         ///< 控制器尚未初始化，或内部模块初始化失败。
	HIC_STATUS_ERROR_INVALID_PARAM = 2,///< 输入参数非法，例如关节数越界、模式不支持、限值无效等。
	HIC_STATUS_ERROR_NULL_POINTER = 3, ///< 传入了空指针。
	HIC_STATUS_ERROR_ROBOT_STATE = 4,  ///< 机器人状态无效，例如状态未更新、存在 NaN/Inf 或超出观测器范围。
	HIC_STATUS_ERROR_KINEMATICS = 5,   ///< 运动学计算失败。
	HIC_STATUS_ERROR_DYNAMICS = 6,     ///< 动力学计算失败。
	HIC_STATUS_ERROR_SINGULARITY = 7,  ///< 运动学奇异或接近奇异。
	HIC_STATUS_ERROR_CURRENT_LIMIT = 8,///< 电流或力矩限幅被触发；命令已被限幅，可继续读取保护标志。
	HIC_STATUS_ERROR_JOINT_LIMIT = 9,  ///< 关节硬限位保护被触发。
	HIC_STATUS_ERROR_NOT_IMPLEMENTED = 10 ///< 接口预留但当前版本尚未实现。
} HicStatus;

/// @brief 控制器顶层工作模式。
typedef enum HicControlMode
{
	HIC_CONTROL_MODE_IDLE = 0,         ///< 空闲模式，不输出力控命令。
	HIC_CONTROL_MODE_FORCE_CONTROL = 1 ///< 力控模式，具体子模式由 HicForceControlMode 决定。
} HicControlMode;

/// @brief 力控子模式。
typedef enum HicForceControlMode
{
	HIC_FORCE_CONTROL_MODE_NONE = 0,                     ///< 未进入任何力控子模式。
	HIC_FORCE_CONTROL_MODE_ZERO_FORCE = 1,               ///< 零力拖动模式：重力补偿 + 速度阻尼 + 安全限幅。
	HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION = 2, ///< 笛卡尔固定位置阻抗，只控制末端 XYZ 位置。
	HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE = 3,     ///< 笛卡尔固定位姿阻抗，控制末端位置 + 姿态。
	HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY = 4,     ///< 笛卡尔在线轨迹阻抗，目标位姿可实时刷新。
	HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE = 5           ///< 关节空间阻抗模式，按关节位置/速度误差计算阻抗力矩。
} HicForceControlMode;

/// @brief 命令闭环类型。
typedef enum HicCommandLoopType
{
	HIC_COMMAND_LOOP_CURRENT = 0, ///< 外部驱动器接收电流命令，单位 A。
	HIC_COMMAND_LOOP_TORQUE = 1   ///< 外部驱动器接收关节力矩命令，单位 N.m。
} HicCommandLoopType;

/// @brief HIC 固定维度常量。
typedef enum HicDimensions
{
	HIC_MAX_JOINTS = 16, ///< 最大支持关节数，实际关节数由初始化配置 jointCount 指定。
	HIC_CARTESIAN_DIM = 6, ///< 笛卡尔速度/力维度：[x, y, z, rx, ry, rz]。
	HIC_POSE_DIM = 7, ///< 末端位姿维度：[x, y, z, qw, qx, qy, qz]。
	HIC_DYNAMIC_PARAM_PER_JOINT = 13, ///< 每个关节对应 13 个动力学线性参数。
	HIC_MAX_DYNAMIC_PARAMS = HIC_MAX_JOINTS * HIC_DYNAMIC_PARAM_PER_JOINT, ///< 最大动力学参数数量。
	HIC_MAX_JACOBIAN_SIZE = HIC_CARTESIAN_DIM * HIC_MAX_JOINTS ///< 最大雅可比矩阵元素数量，行主序。
} HicDimensions;

/// @brief 控制器初始化配置。
/// @note 该结构体只包含创建控制器所需的最小参数；运行时限值、动力学参数、电机参数等通过 set 接口继续配置。
typedef struct HicInitializeConfig
{
	int jointCount; ///< 机器人实际关节数，必须大于 0 且不超过 HIC_MAX_JOINTS。
	double controlPeriod; ///< 控制周期，单位 s，例如 0.001 表示 1 kHz。
	int robotType; ///< 机器人类型编号，用于选择内部运动学/动力学模型；例如现有 7 自由度示例使用 20。
	double kinematicDHParams[HIC_MAX_JOINTS]; ///< 运动学参数数组，具体含义由 robotType 对应模型解释，内部按 SI 单位使用。
} HicInitializeConfig;

/// @brief 控制器完整运行时配置快照。
/// @note 对外一般不直接整体设置该结构，而是通过 hic_initialize_control() 和各类 hic_set_xxx() 接口逐项配置。
typedef struct HicControlConfig
{
	int jointCount; ///< 关节数。
	double controlPeriod; ///< 控制周期，单位 s。
	int robotType; ///< 机器人类型编号，用于选择内部运动学/动力学模型。

	double kinematicParams[HIC_MAX_JOINTS]; ///< 运动学参数，内部按 SI 单位使用。
	double dynamicParams[HIC_MAX_DYNAMIC_PARAMS]; ///< 动力学线性参数，每关节 13 个参数。

	double torqueConstant[HIC_MAX_JOINTS]; ///< 电机力矩常数 Kt，单位 N.m/A。
	double gearRatio[HIC_MAX_JOINTS]; ///< 减速比。
	double transmissionEfficiency[HIC_MAX_JOINTS]; ///< 传动效率，通常为 0~1。

	double lowerJointTorque[HIC_MAX_JOINTS]; ///< 关节力矩下限，单位 N.m。
	double upperJointTorque[HIC_MAX_JOINTS]; ///< 关节力矩上限，单位 N.m。
	double maxJointTorque[HIC_MAX_JOINTS]; ///< 关节力矩绝对值上限，单位 N.m。
	double lowerMotorCurrent[HIC_MAX_JOINTS]; ///< 电机电流下限，单位 A。
	double upperMotorCurrent[HIC_MAX_JOINTS]; ///< 电机电流上限，单位 A。
	double maxJointCurrent[HIC_MAX_JOINTS]; ///< 电流绝对值上限，单位 A。
	double maxTorqueRate[HIC_MAX_JOINTS]; ///< 关节力矩变化率上限，单位 N.m/s。
	double lowerJointVelocity[HIC_MAX_JOINTS]; ///< 关节速度下限，内部单位 rad/s。
	double upperJointVelocity[HIC_MAX_JOINTS]; ///< 关节速度上限，内部单位 rad/s。
	double lowerJointAcceleration[HIC_MAX_JOINTS]; ///< 关节加速度下限，内部单位 rad/s^2。
	double upperJointAcceleration[HIC_MAX_JOINTS]; ///< 关节加速度上限，内部单位 rad/s^2。
	double upperJointLimit[HIC_MAX_JOINTS]; ///< 关节位置硬上限，内部单位 rad。
	double lowerJointLimit[HIC_MAX_JOINTS]; ///< 关节位置硬下限，内部单位 rad。

	bool enableGravityCompensation; ///< 是否启用重力补偿。
	bool enableCoriolisCompensation; ///< 是否启用科氏/离心项补偿。
	bool enableTorqueRateLimit; ///< 是否启用力矩变化率限制。
	bool enableCurrentLimit; ///< 是否启用电流限幅。
} HicControlConfig;

/// @brief 笛卡尔阻抗刚度/阻尼参数。
/// @note 位置方向刚度通常为 N/m，姿态方向刚度通常为 N.m/rad；阻尼按对应速度维度配置。
typedef struct HicImpedanceGains
{
	double stiffness[HIC_CARTESIAN_DIM]; ///< 笛卡尔 6 维刚度。
	double damping[HIC_CARTESIAN_DIM]; ///< 笛卡尔 6 维阻尼。
} HicImpedanceGains;

/// @brief 笛卡尔阻抗模式下的零空间关节控制配置。
typedef struct HicNullspaceControlConfig
{
	bool enabled; ///< 是否启用零空间控制。
	double targetJointPosition[HIC_MAX_JOINTS]; ///< 零空间目标关节位置，内部单位 rad。
	double stiffness[HIC_MAX_JOINTS]; ///< 零空间关节刚度，单位 N.m/rad。
	double damping[HIC_MAX_JOINTS]; ///< 零空间关节阻尼，单位 N.m.s/rad。
} HicNullspaceControlConfig;

/// @brief 关节空间阻抗控制配置。
/// @note hic_set_joint_impedance_config() 是 C API 边界函数，其中 targetPosition 按 deg 输入，
///       会在 hic_c_api.cpp 内部转换为 rad 后存储；其他字段已经是 SI 单位，不做换算。
typedef struct HicJointImpedanceConfig
{
	double stiffness[HIC_MAX_JOINTS]; ///< 关节刚度 Kd，单位 N.m/rad。
	double damping[HIC_MAX_JOINTS]; ///< 关节阻尼 Dd，单位 N.m.s/rad。
	double targetPosition[HIC_MAX_JOINTS]; ///< 期望关节位置 q_d；C API 输入单位 deg，内部存储 rad。
	double targetVelocity[HIC_MAX_JOINTS]; ///< 期望关节速度 dq_d，单位 rad/s，通常设为 0。
	double targetAcceleration[HIC_MAX_JOINTS]; ///< 期望关节加速度 ddq_d，单位 rad/s^2；当前 PD 控制律暂不使用。
	bool enableExternalTorqueCompensation; ///< 是否减去外力矩估计 tau_ext_hat，实现外力矩补偿。
} HicJointImpedanceConfig;

/// @brief 机器人关节状态观测器配置。
typedef struct HicRobotStateObserverConfig
{
	int jointCount; ///< 有效关节数量。
	double controlPeriod; ///< 控制周期，单位 s。

	bool enablePositionFilter; ///< 是否启用关节位置滤波。
	bool enableVelocityFilter; ///< 是否启用关节速度滤波。
	bool enableAccelerationFilter; ///< 是否启用关节加速度滤波。
	bool enableMotorCurrentFilter; ///< 是否启用电机电流滤波。
	bool enableMeasuredTorqueFilter; ///< 是否启用实测关节力矩滤波。

	double positionFilterAlpha[HIC_MAX_JOINTS]; ///< 位置一阶低通滤波系数，范围通常为 0~1。
	double velocityFilterAlpha[HIC_MAX_JOINTS]; ///< 速度一阶低通滤波系数，范围通常为 0~1。
	double accelerationFilterAlpha[HIC_MAX_JOINTS]; ///< 加速度一阶低通滤波系数，范围通常为 0~1。
	double motorCurrentFilterAlpha[HIC_MAX_JOINTS]; ///< 电机电流一阶低通滤波系数，范围通常为 0~1。
	double measuredTorqueFilterAlpha[HIC_MAX_JOINTS]; ///< 实测力矩一阶低通滤波系数，范围通常为 0~1。

	double lowerJointPosition[HIC_MAX_JOINTS]; ///< 关节位置下限，内部单位 rad。
	double upperJointPosition[HIC_MAX_JOINTS]; ///< 关节位置上限，内部单位 rad。
	double maxAbsJointPosition[HIC_MAX_JOINTS]; ///< 关节位置最大绝对值，内部单位 rad。
	double lowerJointVelocity[HIC_MAX_JOINTS]; ///< 关节速度下限，单位 rad/s。
	double upperJointVelocity[HIC_MAX_JOINTS]; ///< 关节速度上限，单位 rad/s。
	double maxAbsJointVelocity[HIC_MAX_JOINTS]; ///< 关节速度最大绝对值，单位 rad/s。
	double lowerJointAcceleration[HIC_MAX_JOINTS]; ///< 关节加速度下限，单位 rad/s^2。
	double upperJointAcceleration[HIC_MAX_JOINTS]; ///< 关节加速度上限，单位 rad/s^2。
	double maxAbsJointAcceleration[HIC_MAX_JOINTS]; ///< 关节加速度最大绝对值，单位 rad/s^2。
	double lowerMotorCurrent[HIC_MAX_JOINTS]; ///< 电机电流下限，单位 A。
	double upperMotorCurrent[HIC_MAX_JOINTS]; ///< 电机电流上限，单位 A。
	double maxAbsMotorCurrent[HIC_MAX_JOINTS]; ///< 电机电流最大绝对值，单位 A。
	double lowerMeasuredTorque[HIC_MAX_JOINTS]; ///< 实测关节力矩下限，单位 N.m。
	double upperMeasuredTorque[HIC_MAX_JOINTS]; ///< 实测关节力矩上限，单位 N.m。
	double maxAbsMeasuredTorque[HIC_MAX_JOINTS]; ///< 实测关节力矩最大绝对值，单位 N.m。

	double torqueConstant[HIC_MAX_JOINTS]; ///< 电机力矩常数，单位 N.m/A。
	double gearRatio[HIC_MAX_JOINTS]; ///< 减速比，用于电机侧到关节侧换算。
	double transmissionEfficiency[HIC_MAX_JOINTS]; ///< 传动效率，范围通常为 0~1。

	bool enableCurrentToTorqueEstimate; ///< 是否根据电机电流估算关节力矩。
} HicRobotStateObserverConfig;

/// @brief 单个关节力矩传感器通道配置。
typedef struct HicJointTorqueSensorConfig
{
	bool enabled; ///< 是否启用该关节力矩传感器。
	int jointIndex; ///< 该传感器对应的关节索引，从 0 开始。
	int hardwareChannel; ///< 硬件采集通道编号。

	double ratedCapacityNm; ///< 传感器额定量程，单位 N.m。
	int directionSign; ///< 方向符号，通常为 1 或 -1，用于统一力矩正方向。

	double zeroOffsetNm; ///< 零点偏置，单位 N.m。
	double scale; ///< 标定缩放系数。
	double biasNm; ///< 额外偏置补偿，单位 N.m。
	double maxValidTorqueNm; ///< 有效力矩最大绝对值，超过后可判为异常，单位 N.m。
} HicJointTorqueSensorConfig;

/// @brief 关节力矩传感器整体配置。
typedef struct HicTorqueSensorConfig
{
	char version[16]; ///< 配置版本字符串。
	char unit[16]; ///< 传感器数据单位描述，例如 "N.m"。
	char sensorLocation[32]; ///< 传感器安装位置描述。

	int jointCount; ///< 有效关节数量。
	HicJointTorqueSensorConfig joints[HIC_MAX_JOINTS]; ///< 各关节力矩传感器配置。

	bool enableTorqueSensorFilter; ///< 是否启用原始关节力矩传感器滤波。
	double torqueSensorFilterAlpha[HIC_MAX_JOINTS]; ///< 原始力矩传感器一阶低通滤波系数。

	bool enableExternalTorqueFilter; ///< 是否启用外力矩估计滤波。
	double externalTorqueFilterAlpha[HIC_MAX_JOINTS]; ///< 外力矩估计一阶低通滤波系数。

	bool enableSaturationCheck; ///< 是否启用传感器饱和检查。
	bool enableFaultCheck; ///< 是否启用传感器故障检查。
} HicTorqueSensorConfig;

/// @brief 当前机器人关节状态输出。
typedef struct HicRobotState
{
	double jointPosition[HIC_MAX_JOINTS]; ///< 关节位置，单位 rad。
	double jointVelocity[HIC_MAX_JOINTS]; ///< 关节速度，单位 rad/s。
	double jointAcceleration[HIC_MAX_JOINTS]; ///< 关节加速度，单位 rad/s^2。
	double motorCurrent[HIC_MAX_JOINTS]; ///< 电机电流，单位 A。
	double jointMeasuredTorque[HIC_MAX_JOINTS]; ///< 关节实测力矩，单位 N.m。
	double motorEstimatedTorque[HIC_MAX_JOINTS]; ///< 由电机电流估算的关节力矩，单位 N.m。
} HicRobotState;

/// @brief 当前末端笛卡尔状态输出。
typedef struct HicCartesianState
{
	double pose[HIC_POSE_DIM]; ///< 末端位姿，[x, y, z, qw, qx, qy, qz]，位置单位 m，姿态为单位四元数。
	double twist[HIC_CARTESIAN_DIM]; ///< 末端速度，[vx, vy, vz, wx, wy, wz]，线速度 m/s，角速度 rad/s。
} HicCartesianState;

/// @brief 高频状态估计输入。
typedef struct HicStateEstimateInput
{
	int jointCount; ///< 有效关节数量。
	double commandJointPosition[HIC_MAX_JOINTS]; ///< 指令关节位置，外部接口单位 deg。
	double jointPosition[HIC_MAX_JOINTS]; ///< 实际关节位置，外部接口单位 deg。
	double motorCurrent[HIC_MAX_JOINTS]; ///< 电机电流，单位 A。
	double currentTime; ///< 当前采样时间，单位 s。
} HicStateEstimateInput;

/// @brief 按上下界描述的关节限制。
typedef struct HicJointRangeLimits
{
	int jointCount; ///< 有效关节数量。
	double lower[HIC_MAX_JOINTS]; ///< 下限数组，单位由具体 API 说明决定。
	double upper[HIC_MAX_JOINTS]; ///< 上限数组，单位由具体 API 说明决定。
} HicJointRangeLimits;

/// @brief 按最大绝对值描述的关节限制。
typedef struct HicJointMaxLimits
{
	int jointCount; ///< 有效关节数量。
	double maxAbs[HIC_MAX_JOINTS]; ///< 最大绝对值数组，单位由具体 API 说明决定。
} HicJointMaxLimits;

/// @brief 动力学力矩分项。
typedef struct HicDynamicsTorqueTerms
{
	int jointCount; ///< 有效关节数量。
	double inertiaTorque[HIC_MAX_JOINTS]; ///< 惯性项力矩 M(q)ddq，单位 N.m。
	double gravityTorque[HIC_MAX_JOINTS]; ///< 重力项力矩 G(q)，单位 N.m。
	double coriolisTorque[HIC_MAX_JOINTS]; ///< 科氏/离心项力矩 C(q,dq)dq，单位 N.m。
	double frictionTorque[HIC_MAX_JOINTS]; ///< 摩擦项力矩，单位 N.m。
	double modelTorque[HIC_MAX_JOINTS]; ///< 动力学模型合力矩，单位 N.m。
} HicDynamicsTorqueTerms;

/// @brief 内部估计的动力学与外力矩输出。
typedef struct HicEstimatedDynamicsTorques
{
	HicDynamicsTorqueTerms terms; ///< 动力学模型各分项。
	double externalTorque[HIC_MAX_JOINTS]; ///< 估计外力矩，单位 N.m。
} HicEstimatedDynamicsTorques;

/// @brief 当前控制器活动状态。
typedef struct HicActiveControlState
{
	int controlMode; ///< 当前顶层控制模式，对应 HicControlMode。
	int forceControlMode; ///< 当前力控子模式，对应 HicForceControlMode。
	bool supportsCurrentLoop; ///< 当前模式是否支持输出电流命令。
	bool supportsTorqueLoop; ///< 当前模式是否支持输出力矩命令。
	bool nullspaceEnabled; ///< 笛卡尔阻抗模式下零空间控制是否启用。
} HicActiveControlState;

/// @brief 运动保护或限幅触发原因。
typedef enum HicProtectionReason
{
	HIC_PROTECTION_NONE = 0, ///< 未触发保护。
	HIC_PROTECTION_COLLISION = 1, ///< 碰撞检测触发。
	HIC_PROTECTION_JOINT_POSITION_LIMIT = 2, ///< 关节位置限位触发。
	HIC_PROTECTION_JOINT_VELOCITY_LIMIT = 3, ///< 关节速度限制触发。
	HIC_PROTECTION_JOINT_ACCELERATION_LIMIT = 4, ///< 关节加速度限制触发。
	HIC_PROTECTION_JOINT_TORQUE_LIMIT = 5, ///< 关节力矩限制触发。
	HIC_PROTECTION_MOTOR_CURRENT_LIMIT = 6, ///< 电机电流限制触发。
	HIC_PROTECTION_CONTROL_BOX_CURRENT_LIMIT = 7, ///< 控制箱总电流限制触发。
	HIC_PROTECTION_POWER_MOMENTUM_LIMIT = 8, ///< 功率/动量约束触发。
	HIC_PROTECTION_STATE_INVALID = 9, ///< 状态输入无效。
	HIC_PROTECTION_PARAM_INVALID = 10 ///< 参数配置无效。
} HicProtectionReason;

/// @brief 状态滤波时间常数配置。
typedef struct HicStateFilterConfig
{
	int jointCount; ///< 有效关节数量。
	bool enable_position_filter; ///< 是否启用关节位置滤波。
	bool enable_velocity_filter; ///< 是否启用关节速度滤波。
	bool enable_acceleration_filter; ///< 是否启用关节加速度滤波。
	bool enable_current_filter; ///< 是否启用电机电流滤波。
	double position_filter_time_constant[HIC_MAX_JOINTS]; ///< 位置滤波时间常数，单位 s。
	double velocity_filter_time_constant[HIC_MAX_JOINTS]; ///< 速度滤波时间常数，单位 s。
	double acceleration_filter_time_constant[HIC_MAX_JOINTS]; ///< 加速度滤波时间常数，单位 s。
	double current_filter_time_constant[HIC_MAX_JOINTS]; ///< 电流滤波时间常数，单位 s。
} HicStateFilterConfig;

/// @brief 零力示教模式安全配置。
typedef struct HicZeroForceSafetyConfig
{
	int jointCount; ///< 有效关节数量。
	bool enable_start_safety_check; ///< 是否在进入零力模式时执行启动安全检查。
	double start_check_time; ///< 启动安全检查持续时间，单位 s。
	bool enable_smooth_exit; ///< 是否启用零力模式平滑退出。
	double zero_force_exit_velocity_threshold[HIC_MAX_JOINTS]; ///< 平滑退出速度阈值，单位 rad/s。
	double joint_current_max_abs[HIC_MAX_JOINTS]; ///< 单关节电流最大绝对值，单位 A。
	double control_box_total_current_limit; ///< 控制箱总电流限制，单位 A。
} HicZeroForceSafetyConfig;

/// @brief 碰撞检测配置。
typedef struct HicCollisionDetectionConfig
{
	int jointCount; ///< 有效关节数量。
	bool enable_iso15066_fast_strategy; ///< 是否启用 ISO/TS 15066 相关快速碰撞策略。
	double collision_stop_threshold[HIC_MAX_JOINTS]; ///< 普通模式碰撞停止阈值，单位通常为 N.m。
	double momentum_collision_threshold[HIC_MAX_JOINTS]; ///< 动量碰撞检测阈值。
	double zero_force_collision_threshold[HIC_MAX_JOINTS]; ///< 零力模式碰撞阈值，单位通常为 N.m。
	int collision_recovery_mode; ///< 碰撞后的恢复策略编号。
} HicCollisionDetectionConfig;

/// @brief 当前运动约束/限幅状态。
typedef struct HicMotionConstraintStatus
{
	int jointCount; ///< 有效关节数量。
	bool joint_position_limit_active[HIC_MAX_JOINTS]; ///< 各关节位置限位是否触发。
	bool joint_velocity_limit_active[HIC_MAX_JOINTS]; ///< 各关节速度限制是否触发。
	bool joint_acceleration_limit_active[HIC_MAX_JOINTS]; ///< 各关节加速度限制是否触发。
	bool joint_torque_limit_active[HIC_MAX_JOINTS]; ///< 各关节力矩限制是否触发。
	bool motor_current_limit_active[HIC_MAX_JOINTS]; ///< 各关节电机电流限制是否触发。
	bool any_limit_active; ///< 是否有任意约束或限幅处于触发状态。
	int main_reason; ///< 主要触发原因，对应 HicProtectionReason。
} HicMotionConstraintStatus;

/// @brief 摩擦补偿配置。
typedef struct HicFrictionCompensationConfig
{
	int jointCount; ///< 有效关节数量。
	bool enable_friction_compensation; ///< 是否启用摩擦补偿。
	double friction_compensation_factor[HIC_MAX_JOINTS]; ///< 静态/基础摩擦补偿系数。
	double dynamic_friction_compensation_factor[HIC_MAX_JOINTS]; ///< 动态摩擦补偿系数。
	double friction_low_velocity_threshold[HIC_MAX_JOINTS]; ///< 低速摩擦判断阈值，单位 rad/s。
	double actuator_damping[HIC_MAX_JOINTS]; ///< 执行器等效阻尼，单位 N.m.s/rad。
} HicFrictionCompensationConfig;

/// @brief 独立动力学计算输入。
typedef struct HicDynamicsComputeInput
{
	int jointCount; ///< 有效关节数量。
	double joint_position[HIC_MAX_JOINTS]; ///< 关节位置，外部接口单位 deg。
	double joint_velocity[HIC_MAX_JOINTS]; ///< 关节速度，外部接口单位 deg/s。
	double joint_acceleration[HIC_MAX_JOINTS]; ///< 关节加速度，外部接口单位 deg/s^2。
} HicDynamicsComputeInput;

/// @brief 独立动力学计算输出。
typedef HicDynamicsTorqueTerms HicDynamicsComputeOutput;

/// @brief 六维力/力矩传感器输入。
typedef struct HicWrenchSensorInput
{
	double wrench[HIC_CARTESIAN_DIM]; ///< 六维力，格式 [Fx, Fy, Fz, Mx, My, Mz]，力单位 N，力矩单位 N.m。
	int frame_type; ///< 数据所在坐标系类型编号，由上层约定。
} HicWrenchSensorInput;

/// @brief 双编码器关节位置输入。
typedef struct HicDualEncoderInput
{
	int jointCount; ///< 有效关节数量。
	double motor_side_joint_position[HIC_MAX_JOINTS]; ///< 电机侧折算关节位置，外部接口单位 deg。
	double joint_side_joint_position[HIC_MAX_JOINTS]; ///< 关节侧编码器位置，外部接口单位 deg。
} HicDualEncoderInput;

/// @brief 双编码器辅助检查配置。
typedef struct HicDualEncoderAssistConfig
{
	int jointCount; ///< 有效关节数量。
	bool enable; ///< 是否启用双编码器一致性辅助检查。
	double diff_threshold[HIC_MAX_JOINTS]; ///< 电机侧与关节侧位置差阈值，单位 rad。
} HicDualEncoderAssistConfig;

/// @brief 功率与动量约束配置。
typedef struct HicPowerMomentumConstraintConfig
{
	int jointCount; ///< 有效关节数量。
	bool enable_power_constraint; ///< 是否启用功率约束。
	bool enable_momentum_constraint; ///< 是否启用动量约束。
	double max_power; ///< 整机最大允许功率，单位 W。
	double max_momentum; ///< 整机最大允许动量，单位按内部模型约定。
	double joint_power_limit[HIC_MAX_JOINTS]; ///< 单关节功率限制，单位 W。
} HicPowerMomentumConstraintConfig;

/// @brief 功率与动量约束运行状态。
typedef struct HicPowerMomentumConstraintStatus
{
	int jointCount; ///< 有效关节数量。
	bool constraint_active[HIC_MAX_JOINTS]; ///< 各关节功率/动量约束是否触发。
	double joint_velocity_scale[HIC_MAX_JOINTS]; ///< 约束计算得到的关节速度缩放系数。
	double joint_acceleration_scale[HIC_MAX_JOINTS]; ///< 约束计算得到的关节加速度缩放系数。
	double electric_power[HIC_MAX_JOINTS]; ///< 各关节电功率估计，单位 W。
	double mechanical_power[HIC_MAX_JOINTS]; ///< 各关节机械功率估计，单位 W。
	double momentum[HIC_MAX_JOINTS]; ///< 各关节动量估计，单位按内部模型约定。
	bool any_constraint_active; ///< 是否有任意功率/动量约束触发。
} HicPowerMomentumConstraintStatus;

#ifdef __cplusplus
namespace hic
{
using ::HicStatus;
using ::HicControlMode;
using ::HicForceControlMode;
using ::HicCommandLoopType;
using ::HicDimensions;
using ::HicInitializeConfig;
using ::HicControlConfig;
using ::HicImpedanceGains;
using ::HicNullspaceControlConfig;
using ::HicJointImpedanceConfig;
using ::HicRobotStateObserverConfig;
using ::HicJointTorqueSensorConfig;
using ::HicTorqueSensorConfig;
using ::HicRobotState;
using ::HicCartesianState;
using ::HicStateEstimateInput;
using ::HicJointRangeLimits;
using ::HicJointMaxLimits;
using ::HicDynamicsTorqueTerms;
using ::HicEstimatedDynamicsTorques;
using ::HicActiveControlState;
using ::HicProtectionReason;
using ::HicStateFilterConfig;
using ::HicZeroForceSafetyConfig;
using ::HicCollisionDetectionConfig;
using ::HicMotionConstraintStatus;
using ::HicFrictionCompensationConfig;
using ::HicDynamicsComputeInput;
using ::HicDynamicsComputeOutput;
using ::HicWrenchSensorInput;
using ::HicDualEncoderInput;
using ::HicDualEncoderAssistConfig;
using ::HicPowerMomentumConstraintConfig;
using ::HicPowerMomentumConstraintStatus;
}
#endif

#define HIC_VERSION_STRING "20260511-v1.1"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 0. 版本
// ============================================================

/// @brief 获取版本字符串。
/// @return 返回库内部固定版本字符串，例如 `20260511-v1.1`。
/// @note 该返回值指向库内部只读静态字符串，调用方不应修改，也无需释放。
HIC_EXPORT const char* hic_get_version();

// ============================================================
// 1. 高频状态输入
// ============================================================

/// @brief 更新机器人状态估计输入。
HIC_EXPORT int hic_update_state_estimates(RTS_IEC_INT groupId,const HicStateEstimateInput* input);

/// @brief 更新原始关节扭矩传感器数据，单位 N.m。
HIC_EXPORT int hic_update_raw_joint_torque_sensor(RTS_IEC_INT groupId,
	const double raw_torque_by_hardware_channel[HIC_MAX_JOINTS]);

/// @brief 更新六维力/力矩传感器数据。
HIC_EXPORT int hic_update_wrench_sensor_data(RTS_IEC_INT groupId, const HicWrenchSensorInput* input);

/// @brief 更新双编码器关节位置输入，单位 deg。
HIC_EXPORT int hic_update_dual_encoder_joint_position(RTS_IEC_INT groupId, const HicDualEncoderInput* input);

// ============================================================
// 2. 低频参数配置
// ============================================================

/// @brief 按最小基础配置初始化控制器。
/// @param config 基础初始化配置。
/// @return 返回 HicStatus 状态码。
/// @note 该接口只负责建立控制器实例、绑定机型、设置控制周期和加载基础运动学参数。
/// @note 执行器换算参数、关节限位、速度/加速度约束、电流/力矩限制等运行配置，
/// 应通过后续各个 hic_set_xxx(...) 接口单独设置。
HIC_EXPORT int hic_initialize_control(RTS_IEC_INT groupId, const HicInitializeConfig* config);

/// @brief 设置线性化动力学参数。
HIC_EXPORT int hic_set_dynamics_linear_parameters(RTS_IEC_INT groupId,
	const double dynamic_params[HIC_MAX_DYNAMIC_PARAMS]);

/// @brief 设置末端负载质量与质心。
HIC_EXPORT int hic_set_payload_mass_properties(RTS_IEC_INT groupId,
	double mass,
	const double center_of_mass[3]);

/// @brief 设置关节扭矩传感器参数。
HIC_EXPORT int hic_set_joint_torque_sensor_parameters(RTS_IEC_INT groupId,const HicTorqueSensorConfig* config);

/// @brief 设置电机电流到关节力矩的换算参数。
HIC_EXPORT int hic_set_motor_torque_conversion_parameters(RTS_IEC_INT groupId,
	const double torque_constant[HIC_MAX_JOINTS],
	const double gear_ratio[HIC_MAX_JOINTS],
	const double transmission_efficiency[HIC_MAX_JOINTS]);

/// @brief 设置关节位置上下界，单位 deg。
HIC_EXPORT int hic_set_joint_position_limits(RTS_IEC_INT groupId,const HicJointRangeLimits* limits);

/// @brief 设置关节速度最大绝对值，单位 deg/s。
HIC_EXPORT int hic_set_joint_velocity_limits(RTS_IEC_INT groupId,const HicJointMaxLimits* limits);

/// @brief 设置关节加速度最大绝对值，单位 deg/s^2。
HIC_EXPORT int hic_set_joint_acceleration_limits(RTS_IEC_INT groupId,const HicJointMaxLimits* limits);

/// @brief 设置关节力矩上下界，单位 N.m。
HIC_EXPORT int hic_set_joint_torque_limits(RTS_IEC_INT groupId,const HicJointRangeLimits* limits);

/// @brief 设置电机电流上下界，单位 A。
HIC_EXPORT int hic_set_motor_current_limits(RTS_IEC_INT groupId,const HicJointRangeLimits* limits);

/// @brief 设置机器人安装角度。
/// @param rotation 安装方位角，单位 deg。
/// @param tilt 安装倾角，单位 deg。
/// @return 返回 HicStatus 状态码。
/// @note 该接口会把安装角度转换为机器人基坐标系下的重力向量，
/// 然后复用 hic_set_gravity_vector_in_base() 的内部链路。
/// @note 当 tilt = 0 时，等价于标准正装，重力向量为 [0, 0, -9.81]。
HIC_EXPORT int hic_set_robot_mounting_angles(RTS_IEC_INT groupId,double rotation, double tilt);

/// @brief 设置基坐标系下的重力向量，单位 m/s^2。
HIC_EXPORT int hic_set_gravity_vector_in_base(RTS_IEC_INT groupId,double gx, double gy, double gz);

/// @brief 设置状态滤波配置。
HIC_EXPORT int hic_set_state_filter_config(RTS_IEC_INT groupId,const HicStateFilterConfig* config);

/// @brief 设置零力示教安全配置。
HIC_EXPORT int hic_set_zero_force_safety_config(RTS_IEC_INT groupId,const HicZeroForceSafetyConfig* config);

/// @brief 设置碰撞检测配置。
HIC_EXPORT int hic_set_collision_detection_config(RTS_IEC_INT groupId,const HicCollisionDetectionConfig* config);

/// @brief 设置摩擦补偿配置。
HIC_EXPORT int hic_set_friction_compensation_config(RTS_IEC_INT groupId,
	const HicFrictionCompensationConfig* config);

/// @brief 设置双编码器辅助配置。
HIC_EXPORT int hic_set_dual_encoder_assist_config(RTS_IEC_INT groupId,const HicDualEncoderAssistConfig* config);



// ============================================================
// 3. 力控参数与目标
// ============================================================

/// @brief 设置笛卡尔阻抗增益。
HIC_EXPORT int hic_set_cartesian_impedance_gains(RTS_IEC_INT groupId,const HicImpedanceGains* gains);

/// @brief 设置零空间配置。
HIC_EXPORT int hic_set_force_control_nullspace_config(RTS_IEC_INT groupId,const HicNullspaceControlConfig* config);

/// @brief 抓取当前关节位置作为零空间目标。
HIC_EXPORT int hic_capture_current_joint_position_as_nullspace_target(RTS_IEC_INT groupId);

/// @brief 设置关节空间阻抗参数，可在运行时实时更新。
/// @param groupId 控制组编号；当前实现使用单控制器实例，通常传 0。
/// @param config 关节阻抗配置指针；targetPosition 输入单位为 deg，内部转换为 rad。
/// @return HicStatus 状态码。
/// @note stiffness 单位为 N.m/rad，damping 单位为 N.m.s/rad，不做单位换算。
/// @note targetVelocity 和 targetAcceleration 输入单位分别为 rad/s、rad/s^2，不做单位换算。
HIC_EXPORT int hic_set_joint_impedance_config(
	RTS_IEC_INT groupId,
	const HicJointImpedanceConfig* config);

/// @brief 抓取当前滤波后的关节位置作为关节阻抗平衡点。
/// @param groupId 控制组编号；通常传 0。
/// @return HicStatus 状态码。
/// @note 该接口直接读取内部状态观测器中的关节位置，内部单位为 rad，不再做 deg/rad 换算。
HIC_EXPORT int hic_capture_current_joint_position_as_impedance_target(RTS_IEC_INT groupId);

/// @brief 设置定点位置目标，长度为 3，单位 mm。
HIC_EXPORT int hic_set_cartesian_fixed_position_target(RTS_IEC_INT groupId,const double target_position[3]);

/// @brief 抓取当前末端位置作为定点位置目标。
HIC_EXPORT int hic_capture_current_position_as_fixed_target(RTS_IEC_INT groupId);

/// @brief 设置定点位姿目标，输入姿态格式为固定坐标系 Z-Y-X 欧拉角。
/// @param target_pose_zyx_euler 长度固定为 6，格式为 [px, py, pz, rz, ry, rx]。
/// @return 返回 HicStatus 状态码。
/// @note 位置分量 px/py/pz 单位为 mm。
/// @note 姿态分量 rz/ry/rx 单位为 deg，表示依次绕固定基坐标系的 Z、Y、X 轴旋转。
/// @note 该接口会在 hic_c_api.cpp 内部把 Z-Y-X 固定轴欧拉角转换成四元数，
/// 再复用内部的四元数位姿目标链路。
HIC_EXPORT int hic_set_cartesian_fixed_pose_target_zyx_euler(RTS_IEC_INT groupId,
	const double target_pose_zyx_euler[HIC_CARTESIAN_DIM]);

/// @brief 抓取当前末端位姿作为定点位姿目标。
HIC_EXPORT int hic_capture_current_pose_as_fixed_target(RTS_IEC_INT groupId);

/// @brief 设置轨迹模式下的在线位姿和速度目标，输入姿态格式为固定坐标系 Z-Y-X 欧拉角。
/// @param target_pose_zyx_euler 长度固定为 6，格式为 [px, py, pz, rz, ry, rx]。
/// @param target_velocity 长度固定为 6，格式为 [vx, vy, vz, wx, wy, wz]。
/// @return 返回 HicStatus 状态码。
/// @note `target_pose_zyx_euler` 的位置分量单位为 mm，姿态分量单位为 deg。
/// @note 其中 rz/ry/rx 表示依次绕固定基坐标系的 Z、Y、X 轴旋转。
/// @note `target_velocity` 的前三维线速度单位为 mm/s，后三维角速度单位为 deg/s。
/// @note 该接口会在 hic_c_api.cpp 内部先把欧拉角位姿转换成四元数位姿，再复用已有轨迹目标链路。
HIC_EXPORT int hic_set_cartesian_trajectory_target_zyx_euler(RTS_IEC_INT groupId,
	const double target_pose_zyx_euler[HIC_CARTESIAN_DIM],
	const double target_velocity[HIC_CARTESIAN_DIM]);

// ============================================================
// 4. 力控模式
// ============================================================

/// @brief 按统一模式枚举切换到指定力控模式。
/// @param force_control_mode 目标力控模式，对应 HicForceControlMode。
/// @return 返回 HicStatus 状态码。
/// @note 当前支持的 `force_control_mode` 语义如下：
/// 1. `HIC_FORCE_CONTROL_MODE_ZERO_FORCE`
///    零力示教模式。当前版本以重力补偿 + 阻尼 + 软/硬限位保护为主。
/// 2. `HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION`
///    定点位置保持模式。只约束末端位置 XYZ，不约束姿态，可叠加零空间控制。
/// 3. `HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE`
///    定点位姿保持模式。位置和姿态同时约束，可叠加零空间控制。
/// 4. `HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY`
///    轨迹笛卡尔阻抗模式。目标位姿和目标速度允许在线刷新。
/// @note HIC_FORCE_CONTROL_MODE_NONE 不是一个“启动模式”，若传入该值将返回参数错误。
HIC_EXPORT int hic_start_force_control_mode(RTS_IEC_INT groupId,int force_control_mode);

/// @brief 启动关节空间阻抗模式。
/// @param groupId 控制组编号；通常传 0。
/// @return HicStatus 状态码。
/// @note 等价于 hic_start_force_control_mode(groupId, HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE)。
/// @note 启动后可通过 hic_get_force_control_torque_commands() 获取力矩命令，
///       或通过 hic_get_force_control_current_commands() 获取换算后的电流命令。
HIC_EXPORT int hic_start_joint_impedance_mode(RTS_IEC_INT groupId);

/// @brief 准备退出当前力控模式。
HIC_EXPORT int hic_prepare_stop_force_control_mode(RTS_IEC_INT groupId);

// ============================================================
// 5. 命令输出与状态读取
// ============================================================

/// @brief 获取当前力控计算得到的目标电流命令，单位 A。
/// @param groupId 控制组编号；通常传 0。
/// @param motor_current_commands 输出电机电流命令数组，单位 A，长度至少 HIC_MAX_JOINTS。
/// @param joint_protection_status 输出每个关节的保护状态，true 表示该关节触发限幅或保护。
/// @return HicStatus 状态码。
/// @note 内部会先计算关节力矩命令，再通过电机力矩常数、减速比和传动效率转换为电流。
/// @note 零力、笛卡尔阻抗、关节阻抗等力控模式统一走该接口，不需要为关节阻抗单独取电流。
HIC_EXPORT int hic_get_force_control_current_commands(RTS_IEC_INT groupId,
	double motor_current_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS]);

/// @brief 获取当前力控计算得到的目标力矩命令，单位 N.m。
/// @param groupId 控制组编号；通常传 0。
/// @param joint_torque_commands 输出关节力矩命令数组，单位 N.m，长度至少 HIC_MAX_JOINTS。
/// @param joint_protection_status 输出每个关节的保护状态，true 表示该关节触发限幅或保护。
/// @return HicStatus 状态码。
/// @note 该接口用于调试或直接力矩驱动。关节阻抗模式下输出为：
///       阻抗 PD 力矩 + 重力补偿 + 可选科氏/离心补偿，并已通过统一安全限幅。
HIC_EXPORT int hic_get_force_control_torque_commands(RTS_IEC_INT groupId,
	double joint_torque_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS]);

/// @brief 获取当前活动控制状态。
HIC_EXPORT int hic_get_active_control_state(RTS_IEC_INT groupId, HicActiveControlState* state_out);

/// @brief 获取当前机器人关节状态。
/// @note 返回值沿用内部 SI 语义：关节位置单位 rad，关节速度单位 rad/s，
/// 关节加速度单位 rad/s^2，电流单位 A，力矩单位 N.m。
HIC_EXPORT int hic_get_robot_state(RTS_IEC_INT groupId,HicRobotState* state_out);

/// @brief 获取运动约束状态。
HIC_EXPORT int hic_get_motion_constraint_status(RTS_IEC_INT groupId,HicMotionConstraintStatus* status_out);

/// @brief 获取内部动力学估计项。
HIC_EXPORT int hic_get_estimated_dynamics_torques(RTS_IEC_INT groupId,HicEstimatedDynamicsTorques* torques_out);

/// @brief 获取当前末端位姿和速度。
/// @note 返回值沿用内部 SI 语义：pose 的位置分量单位 m，四元数分量无单位；
/// twist 前三维线速度单位 m/s，后三维角速度单位 rad/s。
HIC_EXPORT int hic_get_current_cartesian_state(RTS_IEC_INT groupId,HicCartesianState* state_out);

// ============================================================
// 6. 独立计算与外设辅助
// ============================================================

/// @brief 独立计算逆动力学各分项，不改变内部控制状态。
HIC_EXPORT int hic_compute_inverse_dynamics_torque(RTS_IEC_INT groupId,
	const HicDynamicsComputeInput* input,
	HicDynamicsComputeOutput* output);

/// @brief 计算重力项力矩，单位 N.m。
/// @note 输入关节位置数组按外部接口语义使用 deg。
HIC_EXPORT int hic_compute_gravity_torque(RTS_IEC_INT groupId,
	const double joint_position[HIC_MAX_JOINTS],
	double gravity_torque[HIC_MAX_JOINTS]);

/// @brief 计算重力加科氏/离心项合力矩，单位 N.m。
/// @note 输入关节位置单位为 deg，输入关节速度单位为 deg/s。
HIC_EXPORT int hic_compute_gravity_coriolis_torque(RTS_IEC_INT groupId,
	const double joint_position[HIC_MAX_JOINTS],
	const double joint_velocity[HIC_MAX_JOINTS],
	double torque[HIC_MAX_JOINTS]);


/// @brief 使能力传感器参与碰撞检测。
HIC_EXPORT int hic_enable_wrench_sensor_for_collision_detection(RTS_IEC_INT groupId,bool enable);

/// @brief 使能力传感器参与力控前馈。
HIC_EXPORT int hic_enable_wrench_sensor_for_force_feedforward(RTS_IEC_INT groupId,bool enable);


// ============================================================
// 6. 通用调试接口
// ============================================================


/// @brief 获取当前力控子模式，对应 HicForceControlMode。
/// @note 若当前不处于力控模式，通常返回 HIC_FORCE_CONTROL_MODE_NONE。
HIC_EXPORT int hic_get_force_control_mode(RTS_IEC_INT groupId);

/// @brief 获取最近一次 coordinator 执行返回的状态码。
HIC_EXPORT int hic_get_last_status(RTS_IEC_INT groupId);

/// @brief 复位控制器内部运行状态和缓存，但不重新做基础初始化。
HIC_EXPORT int hic_reset_control(RTS_IEC_INT groupId);



#ifdef __cplusplus
}
#endif
