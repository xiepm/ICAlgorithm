#pragma once

/// @file hic_c_api_types.h
/// @brief HIC C API 专用类型定义。
/// @note 本文件只放对外 C 接口使用的 enum / struct，
/// 避免把类型定义和函数声明混在同一个大头文件中。
/// @note 内部计算链路统一使用 SI 单位。
/// @note 对外输入型结构体中，若字段天然来自上位机或旧系统，hic_c_api.cpp 可在输入边界
/// 把 mm / deg 这一类工程单位换算成内部 SI；输出型结构体默认继续按内部 SI 语义返回。
/// @note 因此阅读本文件时，应结合“该结构体是输入还是输出”来理解字段单位。

#include "hic_controller/types/hic_types.h"

#ifndef __cplusplus
#include <stdbool.h>
#endif

/// @brief 高频状态输入。
typedef struct HicStateEstimateInput
{
	/// @brief 当前有效关节数。
	/// @note 所有固定长度数组实际只使用前 jointCount 个元素。
	int jointCount;
	/// @brief 当前关节命令位置。
	/// @note 作为 hic_update_state_estimates() 的外部输入时，按 deg 传入；
	/// C API 内部如需使用，会先换算成 rad。
	/// @note 该量通常表示驱动器或上位控制器在当前周期下发给各关节的目标位置，
	/// 可用于后续做跟踪误差分析、观测器增强或调试记录。
	double commandJointPosition[HIC_MAX_JOINTS];
	/// @brief 当前关节位置。
	/// @note 作为 hic_update_state_estimates() 的外部输入时，按 deg 传入；
	/// C API 内部会换算成 rad 再交给状态观测器。
	/// @note 这是状态观测器最核心的输入之一，内部会结合 currentTime 估计速度和加速度。
	double jointPosition[HIC_MAX_JOINTS];
	/// @brief 当前电机电流，单位 A。
	/// @note 该量会用于电机估计力矩、状态有效性检查以及部分安全链路。
	double motorCurrent[HIC_MAX_JOINTS];
	/// @brief 当前时间戳，单位 s。
	/// @note 主要用于和上一拍做时间差分，避免速度/加速度估计失真。
	double currentTime;
} HicStateEstimateInput;

/// @brief 通用上下界限制。
typedef struct HicJointRangeLimits
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 每个关节的下界。
	/// @note 具体单位取决于调用场景，例如位置限制输入时单位为 deg，电流限制时单位为 A。
	double lower[HIC_MAX_JOINTS];
	/// @brief 每个关节的上界。
	/// @note 具体单位取决于调用场景，应与 lower 的单位保持一致。
	double upper[HIC_MAX_JOINTS];
} HicJointRangeLimits;

/// @brief 通用最大绝对值限制。
typedef struct HicJointMaxLimits
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 每个关节允许的最大绝对值。
	/// @note 典型用于速度和加速度这种天然按幅值限制的量。
	double maxAbs[HIC_MAX_JOINTS];
} HicJointMaxLimits;

/// @brief 通用动力学分项输出。
typedef struct HicDynamicsTorqueTerms
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 惯性项力矩，单位 N.m。
	double inertiaTorque[HIC_MAX_JOINTS];
	/// @brief 重力项力矩，单位 N.m。
	double gravityTorque[HIC_MAX_JOINTS];
	/// @brief 科氏力/离心力项力矩，单位 N.m。
	double coriolisTorque[HIC_MAX_JOINTS];
	/// @brief 摩擦项力矩，单位 N.m。
	double frictionTorque[HIC_MAX_JOINTS];
	/// @brief 动力学模型合成总力矩，单位 N.m。
	/// @note 一般可理解为 inertia + gravity + coriolis + friction。
	double modelTorque[HIC_MAX_JOINTS];
} HicDynamicsTorqueTerms;

/// @brief 动力学估计输出。
typedef struct HicEstimatedDynamicsTorques
{
	/// @brief 内部动力学模型分项输出。
	HicDynamicsTorqueTerms terms;
	/// @brief 外部力矩估计，单位 N.m。
	/// @note 该量通常表示“测量/估计结果减去模型项”后的剩余关节力矩。
	double externalTorque[HIC_MAX_JOINTS];
} HicEstimatedDynamicsTorques;

/// @brief 当前活动控制状态快照。
typedef struct HicActiveControlState
{
	/// @brief 当前主控制模式，对应 HicControlMode。
	int controlMode;
	/// @brief 当前力控子模式，对应 HicForceControlMode。
	int forceControlMode;
	/// @brief 当前模式是否支持读取电流环目标命令。
	bool supportsCurrentLoop;
	/// @brief 当前模式是否支持读取力矩环目标命令。
	bool supportsTorqueLoop;
	/// @brief 当前零空间控制是否使能。
	bool nullspaceEnabled;
} HicActiveControlState;

/// @brief 保护触发原因。
typedef enum HicProtectionReason
{
	/// @brief 未触发保护。
	HIC_PROTECTION_NONE = 0,
	/// @brief 触发碰撞保护。
	HIC_PROTECTION_COLLISION = 1,
	/// @brief 触发关节位置限制保护。
	HIC_PROTECTION_JOINT_POSITION_LIMIT = 2,
	/// @brief 触发关节速度限制保护。
	HIC_PROTECTION_JOINT_VELOCITY_LIMIT = 3,
	/// @brief 触发关节加速度限制保护。
	HIC_PROTECTION_JOINT_ACCELERATION_LIMIT = 4,
	/// @brief 触发关节力矩限制保护。
	HIC_PROTECTION_JOINT_TORQUE_LIMIT = 5,
	/// @brief 触发电机电流限制保护。
	HIC_PROTECTION_MOTOR_CURRENT_LIMIT = 6,
	/// @brief 触发控制箱总电流限制保护。
	HIC_PROTECTION_CONTROL_BOX_CURRENT_LIMIT = 7,
	/// @brief 触发功率或动量约束保护。
	HIC_PROTECTION_POWER_MOMENTUM_LIMIT = 8,
	/// @brief 当前机器人状态无效。
	HIC_PROTECTION_STATE_INVALID = 9,
	/// @brief 当前参数配置无效。
	HIC_PROTECTION_PARAM_INVALID = 10
} HicProtectionReason;

/// @brief 状态滤波配置，时间常数单位 s。
typedef struct HicStateFilterConfig
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 是否使能位置滤波。
	bool enable_position_filter;
	/// @brief 是否使能速度滤波。
	bool enable_velocity_filter;
	/// @brief 是否使能加速度滤波。
	bool enable_acceleration_filter;
	/// @brief 是否使能电流滤波。
	bool enable_current_filter;
	/// @brief 关节位置滤波时间常数，单位 s。
	double position_filter_time_constant[HIC_MAX_JOINTS];
	/// @brief 关节速度滤波时间常数，单位 s。
	double velocity_filter_time_constant[HIC_MAX_JOINTS];
	/// @brief 关节加速度滤波时间常数，单位 s。
	double acceleration_filter_time_constant[HIC_MAX_JOINTS];
	/// @brief 电机电流滤波时间常数，单位 s。
	double current_filter_time_constant[HIC_MAX_JOINTS];
} HicStateFilterConfig;

/// @brief 零力示教安全配置。
typedef struct HicZeroForceSafetyConfig
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 是否使能启动阶段安全检查。
	/// @note 使能后，进入零力示教前会先检查状态有效性、速度和边界等条件。
	bool enable_start_safety_check;
	/// @brief 启动检查时间窗口，单位 s。
	/// @note 用于进入零力示教后的一段观察期。
	double start_check_time;
	/// @brief 是否使能平滑退出。
	/// @note 使能后，退出零力示教时不会立刻掉到 IDLE，而是先进入阻尼减速阶段。
	bool enable_smooth_exit;
	/// @brief 零力退出速度阈值。
	/// @note 作为 C API 外部输入时按 deg/s 填写，内部会换算成 rad/s。
	/// @note 当所有关节速度都低于该阈值时，可认为已具备安全退出条件。
	double zero_force_exit_velocity_threshold[HIC_MAX_JOINTS];
	/// @brief 每个关节允许的最大电流绝对值，单位 A。
	double joint_current_max_abs[HIC_MAX_JOINTS];
	/// @brief 控制箱允许的总电流上限，单位 A。
	/// @note 该量针对整机总电流，不等同于单关节电流上限。
	double control_box_total_current_limit;
} HicZeroForceSafetyConfig;

/// @brief 碰撞检测配置。
typedef struct HicCollisionDetectionConfig
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 是否使能 ISO 15066 风格的快速碰撞检测策略。
	bool enable_iso15066_fast_strategy;
	/// @brief 常规碰撞停机阈值，单位 N.m。
	double collision_stop_threshold[HIC_MAX_JOINTS];
	/// @brief 动量碰撞阈值。
	/// @note 该量用于高速或大惯量工况下的碰撞风险判断，单位与内部实现保持一致。
	double momentum_collision_threshold[HIC_MAX_JOINTS];
	/// @brief 零力示教模式专用碰撞阈值，单位 N.m。
	double zero_force_collision_threshold[HIC_MAX_JOINTS];
	/// @brief 碰撞恢复模式。
	/// @note 当前建议约定：
	/// 0 = 直接伺服去使能；
	/// 1 = 切换到零力/自由拖动模式；
	/// 2 = 限幅反弹或受限回退模式。
	int collision_recovery_mode;
} HicCollisionDetectionConfig;

/// @brief 运动约束状态。
typedef struct HicMotionConstraintStatus
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 关节位置限制是否激活。
	bool joint_position_limit_active[HIC_MAX_JOINTS];
	/// @brief 关节速度限制是否激活。
	bool joint_velocity_limit_active[HIC_MAX_JOINTS];
	/// @brief 关节加速度限制是否激活。
	bool joint_acceleration_limit_active[HIC_MAX_JOINTS];
	/// @brief 关节力矩限制是否激活。
	bool joint_torque_limit_active[HIC_MAX_JOINTS];
	/// @brief 电机电流限制是否激活。
	bool motor_current_limit_active[HIC_MAX_JOINTS];
	/// @brief 是否存在任意一种约束正在生效。
	bool any_limit_active;
	/// @brief 当前最主要的保护原因，对应 HicProtectionReason。
	int main_reason;
} HicMotionConstraintStatus;

/// @brief 摩擦补偿配置。
typedef struct HicFrictionCompensationConfig
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 是否使能摩擦补偿。
	bool enable_friction_compensation;
	/// @brief 静摩擦补偿因子。
	/// @note 主要影响低速起步和过零点附近的补偿强度。
	double friction_compensation_factor[HIC_MAX_JOINTS];
	/// @brief 动摩擦补偿因子。
	/// @note 主要影响运动过程中的速度相关摩擦补偿强度。
	double dynamic_friction_compensation_factor[HIC_MAX_JOINTS];
	/// @brief 摩擦低速区阈值。
	/// @note 作为 C API 外部输入时按 deg/s 填写，内部会换算成 rad/s。
	/// @note 用于判断当前是否处于摩擦补偿最敏感的低速区间。
	double friction_low_velocity_threshold[HIC_MAX_JOINTS];
	/// @brief 执行器阻尼系数。
	/// @note 可用于改善零力示教拖动手感，抑制振荡和低速爬行。
	double actuator_damping[HIC_MAX_JOINTS];
} HicFrictionCompensationConfig;

/// @brief 动力学独立计算输入。
typedef struct HicDynamicsComputeInput
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 关节位置。
	/// @note 作为独立动力学计算输入时，外部按 deg 传入，内部会换算成 rad。
	double joint_position[HIC_MAX_JOINTS];
	/// @brief 关节速度。
	/// @note 作为独立动力学计算输入时，外部按 deg/s 传入，内部会换算成 rad/s。
	double joint_velocity[HIC_MAX_JOINTS];
	/// @brief 关节加速度。
	/// @note 作为独立动力学计算输入时，外部按 deg/s^2 传入，内部会换算成 rad/s^2。
	double joint_acceleration[HIC_MAX_JOINTS];
} HicDynamicsComputeInput;

/// @brief 动力学独立计算输出。
typedef HicDynamicsTorqueTerms HicDynamicsComputeOutput;

/// @brief 六维力/力矩传感器输入。
typedef struct HicWrenchSensorInput
{
	/// @brief 六维力/力矩测量值。
	/// @note 顺序固定为 [Fx, Fy, Fz, Mx, My, Mz]；
	/// Fx/Fy/Fz 单位 N，Mx/My/Mz 单位 N.m。
	double wrench[HIC_CARTESIAN_DIM];
	/// @brief 数据所在坐标系类型。
	/// @note 当前建议约定：
	/// 0 = 传感器原始坐标系；
	/// 1 = 工具坐标系；
	/// 2 = 基坐标系；
	/// 3 = 用户自定义坐标系。
	int frame_type;
} HicWrenchSensorInput;

/// @brief 双编码器关节位置输入。
typedef struct HicDualEncoderInput
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 电机侧编码器换算得到的关节位置。
	/// @note 作为 C API 外部输入时按 deg 填写，内部会换算成 rad。
	double motor_side_joint_position[HIC_MAX_JOINTS];
	/// @brief 关节侧编码器直接测得的关节位置。
	/// @note 作为 C API 外部输入时按 deg 填写，内部会换算成 rad。
	double joint_side_joint_position[HIC_MAX_JOINTS];
} HicDualEncoderInput;

/// @brief 双编码器辅助配置。
typedef struct HicDualEncoderAssistConfig
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 是否使能双编码器辅助功能。
	bool enable;
	/// @brief 电机侧和关节侧位置差阈值。
	/// @note 作为 C API 外部输入时按 deg 填写，内部会换算成 rad。
	/// @note 超过该阈值可视为柔性形变、间隙或编码器异常的候选信号。
	double diff_threshold[HIC_MAX_JOINTS];
} HicDualEncoderAssistConfig;

/// @brief 功率和动量约束配置。
typedef struct HicPowerMomentumConstraintConfig
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 是否使能功率约束。
	bool enable_power_constraint;
	/// @brief 是否使能动量约束。
	bool enable_momentum_constraint;
	/// @brief 整机最大功率，单位 W。
	double max_power;
	/// @brief 整机最大动量阈值。
	/// @note 该量的物理定义和单位应与内部实现保持一致。
	double max_momentum;
	/// @brief 每个关节允许的功率上限，单位 W。
	double joint_power_limit[HIC_MAX_JOINTS];
} HicPowerMomentumConstraintConfig;

/// @brief 功率和动量约束状态。
typedef struct HicPowerMomentumConstraintStatus
{
	/// @brief 当前有效关节数。
	int jointCount;
	/// @brief 每个关节是否触发功率或动量约束。
	bool constraint_active[HIC_MAX_JOINTS];
	/// @brief 每个关节建议的速度缩放系数。
	/// @note 上层若继续规划运动，可按该比例收缩目标速度。
	double joint_velocity_scale[HIC_MAX_JOINTS];
	/// @brief 每个关节建议的加速度缩放系数。
	/// @note 上层若继续规划运动，可按该比例收缩目标加速度。
	double joint_acceleration_scale[HIC_MAX_JOINTS];
	/// @brief 每个关节估算的电功率，单位 W。
	double electric_power[HIC_MAX_JOINTS];
	/// @brief 每个关节估算的机械功率，单位 W。
	double mechanical_power[HIC_MAX_JOINTS];
	/// @brief 每个关节估算的动量。
	/// @note 该量的物理定义和单位应与内部实现保持一致。
	double momentum[HIC_MAX_JOINTS];
	/// @brief 是否存在任意关节触发约束。
	bool any_constraint_active;
} HicPowerMomentumConstraintStatus;
