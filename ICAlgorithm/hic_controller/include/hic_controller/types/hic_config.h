#pragma once

#include "hic_controller/types/hic_types.h"

#ifndef __cplusplus
#include <stdbool.h>
#endif

/// @brief HIC 控制器基础初始化配置。
/// 说明：
/// 1. 所有数组按 HIC_MAX_JOINTS 预留，实际使用长度由 jointCount 决定；
/// 2. 该结构体只放“对象能被创建并识别机型”所必需的最小信息；
/// 3. 运行阶段的执行器参数、限位、电流、力矩等配置不再塞进初始化入口，
///    而是通过各个 hic_set_xxx(...) 接口单独设置；
/// 4. `kinematicDHParams` 的具体长度和语义由 robotType 对应的运动学实现解释。
typedef struct HicInitializeConfig
{
	/// @brief 当前 robotType 对应的实际关节数。
	/// @note 实际控制计算只使用数组前 jointCount 个元素；
	/// @note 数组固定容量由 HIC_MAX_JOINTS 决定，不代表机器人一定是 16 轴。
	int jointCount;
	/// @brief 控制周期，单位 s。
	double controlPeriod;
	/// @brief 机器人机型编号。
	int robotType;
	/// @brief 运动学 DH 参数或机型相关几何参数。
	/// @note 外部只负责按当前 robotType 的约定填写，内部不在 C API 层解释其物理含义。
	double kinematicDHParams[HIC_MAX_JOINTS];
} HicInitializeConfig;

/// @brief HIC 控制器运行配置全集。
/// 说明：
/// 1. 该结构体描述的是“内部实际生效的控制配置快照”；
/// 2. 大部分字段应通过各个 hic_set_xxx(...) 接口逐步下发，而不是一次性塞给 initialize；
/// 3. 该结构体仍保留在公共类型层，便于调试、快照读取和内部模块共享统一配置语言；
/// 4. 对于从 hic_c_api 输入进来的角度相关运行配置，外部单位语义为：
///    关节位置限制使用 deg，关节速度限制使用 deg/s，关节加速度限制使用 deg/s^2；
///    hic_c_api.cpp 会在边界换算成内部 rad 体系。
typedef struct HicControlConfig
{
	int jointCount;
	double controlPeriod;
	int robotType;

	double kinematicParams[HIC_MAX_JOINTS];
	double dynamicParams[HIC_MAX_DYNAMIC_PARAMS];

	double torqueConstant[HIC_MAX_JOINTS];
	double gearRatio[HIC_MAX_JOINTS];
	double transmissionEfficiency[HIC_MAX_JOINTS];

	double lowerJointTorque[HIC_MAX_JOINTS];
	double upperJointTorque[HIC_MAX_JOINTS];
	double maxJointTorque[HIC_MAX_JOINTS];
	double lowerMotorCurrent[HIC_MAX_JOINTS];
	double upperMotorCurrent[HIC_MAX_JOINTS];
	double maxJointCurrent[HIC_MAX_JOINTS];
	double maxTorqueRate[HIC_MAX_JOINTS];
	double lowerJointVelocity[HIC_MAX_JOINTS];
	double upperJointVelocity[HIC_MAX_JOINTS];
	double lowerJointAcceleration[HIC_MAX_JOINTS];
	double upperJointAcceleration[HIC_MAX_JOINTS];
	double upperJointLimit[HIC_MAX_JOINTS];
	double lowerJointLimit[HIC_MAX_JOINTS];

	bool enableGravityCompensation;
	bool enableCoriolisCompensation;
	bool enableTorqueRateLimit;
	bool enableCurrentLimit;
} HicControlConfig;

/// @brief 6 维笛卡尔阻抗参数。
/// 顺序统一为 XYZ 线性分量 + XYZ 角速度/姿态分量。
typedef struct HicImpedanceGains
{
	double stiffness[HIC_CARTESIAN_DIM];
	double damping[HIC_CARTESIAN_DIM];
} HicImpedanceGains;

/// @brief 零空间控制配置。
/// 说明：
/// 1. 仅在冗余机械臂且当前力控模式支持零空间时有意义；
/// 2. targetJointPosition 是零空间参考关节位置；
/// 3. stiffness / damping 是关节空间的零空间回复参数；
/// 4. 第一版按关节空间弹簧阻尼理解，便于先把接口和基础能力搭起来。
typedef struct HicNullspaceControlConfig
{
	bool enabled;
	double targetJointPosition[HIC_MAX_JOINTS];
	double stiffness[HIC_MAX_JOINTS];
	double damping[HIC_MAX_JOINTS];
} HicNullspaceControlConfig;

/// @brief 机器人自身状态观测器配置。
/// 说明：
/// 1. alpha 为一阶低通滤波系数，取值范围建议在 [0, 1]；
/// 2. enableCurrentToTorqueEstimate 为 true 时，按电机参数估算电机侧等效力矩；
/// 3. 所有阈值按 jointCount 的有效关节数使用，其余元素仅做预留。
typedef struct HicRobotStateObserverConfig
{
	int jointCount;
	double controlPeriod;

	bool enablePositionFilter;
	bool enableVelocityFilter;
	bool enableAccelerationFilter;
	bool enableMotorCurrentFilter;
	bool enableMeasuredTorqueFilter;

	double positionFilterAlpha[HIC_MAX_JOINTS];
	double velocityFilterAlpha[HIC_MAX_JOINTS];
	double accelerationFilterAlpha[HIC_MAX_JOINTS];
	double motorCurrentFilterAlpha[HIC_MAX_JOINTS];
	double measuredTorqueFilterAlpha[HIC_MAX_JOINTS];

	double lowerJointPosition[HIC_MAX_JOINTS];
	double upperJointPosition[HIC_MAX_JOINTS];
	double maxAbsJointPosition[HIC_MAX_JOINTS];
	double lowerJointVelocity[HIC_MAX_JOINTS];
	double upperJointVelocity[HIC_MAX_JOINTS];
	double maxAbsJointVelocity[HIC_MAX_JOINTS];
	double lowerJointAcceleration[HIC_MAX_JOINTS];
	double upperJointAcceleration[HIC_MAX_JOINTS];
	double maxAbsJointAcceleration[HIC_MAX_JOINTS];
	double lowerMotorCurrent[HIC_MAX_JOINTS];
	double upperMotorCurrent[HIC_MAX_JOINTS];
	double maxAbsMotorCurrent[HIC_MAX_JOINTS];
	double lowerMeasuredTorque[HIC_MAX_JOINTS];
	double upperMeasuredTorque[HIC_MAX_JOINTS];
	double maxAbsMeasuredTorque[HIC_MAX_JOINTS];

	double torqueConstant[HIC_MAX_JOINTS];
	double gearRatio[HIC_MAX_JOINTS];
	double transmissionEfficiency[HIC_MAX_JOINTS];

	bool enableCurrentToTorqueEstimate;
} HicRobotStateObserverConfig;

/// @brief 单关节扭矩传感器配置。
/// jointIndex 使用 1-based 编号，hardwareChannel 使用 0-based 编号。
typedef struct HicJointTorqueSensorConfig
{
	bool enabled;
	int jointIndex;
	int hardwareChannel;

	double ratedCapacityNm;
	int directionSign;

	double zeroOffsetNm;
	double scale;
	double biasNm;
	double maxValidTorqueNm;
} HicJointTorqueSensorConfig;

/// @brief 扭矩传感器总配置。
/// 用于描述原始硬件通道到关节编号的映射，以及标定、滤波和故障检查参数。
typedef struct HicTorqueSensorConfig
{
	char version[16];
	char unit[16];
	char sensorLocation[32];

	int jointCount;
	HicJointTorqueSensorConfig joints[HIC_MAX_JOINTS];

	bool enableTorqueSensorFilter;
	double torqueSensorFilterAlpha[HIC_MAX_JOINTS];

	bool enableExternalTorqueFilter;
	double externalTorqueFilterAlpha[HIC_MAX_JOINTS];

	bool enableSaturationCheck;
	bool enableFaultCheck;
} HicTorqueSensorConfig;

/// @brief 机器人关节状态。
/// jointPosition / jointVelocity / jointAcceleration / motorCurrent /
/// jointMeasuredTorque / motorEstimatedTorque 的有效长度均为 jointCount。
/// @note 该结构体主要用于内部状态读取和调试输出，单位沿用内部 SI 语义：
/// jointPosition = rad，jointVelocity = rad/s，jointAcceleration = rad/s^2，
/// motorCurrent = A，jointMeasuredTorque / motorEstimatedTorque = N.m。
typedef struct HicRobotState
{
	double jointPosition[HIC_MAX_JOINTS];
	double jointVelocity[HIC_MAX_JOINTS];
	double jointAcceleration[HIC_MAX_JOINTS];
	double motorCurrent[HIC_MAX_JOINTS];
	double jointMeasuredTorque[HIC_MAX_JOINTS];
	double motorEstimatedTorque[HIC_MAX_JOINTS];
} HicRobotState;

/// @brief 笛卡尔状态。
/// pose = [px, py, pz, qw, qx, qy, qz]；
/// twist = [vx, vy, vz, wx, wy, wz]。
/// @note 该结构体主要用于内部状态读取和调试输出，单位沿用内部 SI 语义：
/// pose 的位置分量单位 m，四元数分量无单位；
/// twist 的线速度分量单位 m/s，角速度分量单位 rad/s。
typedef struct HicCartesianState
{
	double pose[HIC_POSE_DIM];
	double twist[HIC_CARTESIAN_DIM];
} HicCartesianState;

#ifdef __cplusplus
namespace hic
{
using ::HicInitializeConfig;
using ::HicControlConfig;
using ::HicImpedanceGains;
using ::HicNullspaceControlConfig;
using ::HicRobotStateObserverConfig;
using ::HicJointTorqueSensorConfig;
using ::HicTorqueSensorConfig;
using ::HicRobotState;
using ::HicCartesianState;
}
#endif
