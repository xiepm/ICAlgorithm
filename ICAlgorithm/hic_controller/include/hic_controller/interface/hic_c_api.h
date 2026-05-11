#pragma once

/// @file hic_c_api.h
/// @brief HIC 对外 C API 入口。
/// @note 类型定义已拆分到 `hic_c_api_types.h`，本文件只保留导出函数声明。
/// @note 内部 coordinator、observer、kinematics、dynamics 统一使用 SI 单位：
/// 位置/长度使用 m，角度使用 rad，角速度使用 rad/s，角加速度使用 rad/s^2，
/// 力矩使用 N.m，电流使用 A，时间使用 s。
/// @note 对外输入接口中，若参数天然来自上位机或旧系统，位置/长度相关量可按 mm 输入，
/// 关节角度相关量可按 deg 输入，hic_c_api.cpp 会在输入边界完成到 SI 的换算。
/// @note 对外输出接口默认继续返回内部 SI 语义，除非某个接口的注释另有明确说明。

#include "hic_controller/interface/hic_export.h"
#include "hic_controller/types/hic_c_api_types.h"
#include "hic_controller/types/hic_config.h"

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
HIC_EXPORT int hic_update_state_estimates(const HicStateEstimateInput* input);

/// @brief 更新原始关节扭矩传感器数据，单位 N.m。
HIC_EXPORT int hic_update_raw_joint_torque_sensor(
	const double raw_torque_by_hardware_channel[HIC_MAX_JOINTS]);

/// @brief 更新六维力/力矩传感器数据。
HIC_EXPORT int hic_update_wrench_sensor_data(const HicWrenchSensorInput* input);

/// @brief 更新双编码器关节位置输入，单位 deg。
HIC_EXPORT int hic_update_dual_encoder_joint_position(const HicDualEncoderInput* input);

// ============================================================
// 2. 低频参数配置
// ============================================================

/// @brief 按最小基础配置初始化控制器。
/// @param config 基础初始化配置。
/// @return 返回 HicStatus 状态码。
/// @note 该接口只负责建立控制器实例、绑定机型、设置控制周期和加载基础运动学参数。
/// @note 执行器换算参数、关节限位、速度/加速度约束、电流/力矩限制等运行配置，
/// 应通过后续各个 hic_set_xxx(...) 接口单独设置。
HIC_EXPORT int hic_initialize_control(const HicInitializeConfig* config);

/// @brief 设置线性化动力学参数。
HIC_EXPORT int hic_set_dynamics_linear_parameters(
	const double dynamic_params[HIC_MAX_DYNAMIC_PARAMS]);

/// @brief 设置末端负载质量与质心。
HIC_EXPORT int hic_set_payload_mass_properties(
	double mass,
	const double center_of_mass[3]);

/// @brief 设置关节扭矩传感器参数。
HIC_EXPORT int hic_set_joint_torque_sensor_parameters(const HicTorqueSensorConfig* config);

/// @brief 设置电机电流到关节力矩的换算参数。
HIC_EXPORT int hic_set_motor_torque_conversion_parameters(
	const double torque_constant[HIC_MAX_JOINTS],
	const double gear_ratio[HIC_MAX_JOINTS],
	const double transmission_efficiency[HIC_MAX_JOINTS]);

/// @brief 设置关节位置上下界，单位 deg。
HIC_EXPORT int hic_set_joint_position_limits(const HicJointRangeLimits* limits);

/// @brief 设置关节速度最大绝对值，单位 deg/s。
HIC_EXPORT int hic_set_joint_velocity_limits(const HicJointMaxLimits* limits);

/// @brief 设置关节加速度最大绝对值，单位 deg/s^2。
HIC_EXPORT int hic_set_joint_acceleration_limits(const HicJointMaxLimits* limits);

/// @brief 设置关节力矩上下界，单位 N.m。
HIC_EXPORT int hic_set_joint_torque_limits(const HicJointRangeLimits* limits);

/// @brief 设置电机电流上下界，单位 A。
HIC_EXPORT int hic_set_motor_current_limits(const HicJointRangeLimits* limits);

/// @brief 设置机器人安装角度。
/// @param rotation 安装方位角，单位 deg。
/// @param tilt 安装倾角，单位 deg。
/// @return 返回 HicStatus 状态码。
/// @note 该接口会把安装角度转换为机器人基坐标系下的重力向量，
/// 然后复用 hic_set_gravity_vector_in_base() 的内部链路。
/// @note 当 tilt = 0 时，等价于标准正装，重力向量为 [0, 0, -9.81]。
HIC_EXPORT int hic_set_robot_mounting_angles(double rotation, double tilt);

/// @brief 设置基坐标系下的重力向量，单位 m/s^2。
HIC_EXPORT int hic_set_gravity_vector_in_base(double gx, double gy, double gz);

/// @brief 设置状态滤波配置。
HIC_EXPORT int hic_set_state_filter_config(const HicStateFilterConfig* config);

/// @brief 设置零力示教安全配置。
HIC_EXPORT int hic_set_zero_force_safety_config(const HicZeroForceSafetyConfig* config);

/// @brief 设置碰撞检测配置。
HIC_EXPORT int hic_set_collision_detection_config(const HicCollisionDetectionConfig* config);

/// @brief 设置摩擦补偿配置。
HIC_EXPORT int hic_set_friction_compensation_config(
	const HicFrictionCompensationConfig* config);

/// @brief 设置双编码器辅助配置。
HIC_EXPORT int hic_set_dual_encoder_assist_config(const HicDualEncoderAssistConfig* config);



// ============================================================
// 3. 力控参数与目标
// ============================================================

/// @brief 设置笛卡尔阻抗增益。
HIC_EXPORT int hic_set_cartesian_impedance_gains(const HicImpedanceGains* gains);

/// @brief 设置零空间配置。
HIC_EXPORT int hic_set_force_control_nullspace_config(const HicNullspaceControlConfig* config);

/// @brief 抓取当前关节位置作为零空间目标。
HIC_EXPORT int hic_capture_current_joint_position_as_nullspace_target();

/// @brief 设置定点位置目标，长度为 3，单位 mm。
HIC_EXPORT int hic_set_cartesian_fixed_position_target(const double target_position[3]);

/// @brief 抓取当前末端位置作为定点位置目标。
HIC_EXPORT int hic_capture_current_position_as_fixed_target();

/// @brief 设置定点位姿目标，输入姿态格式为固定坐标系 Z-Y-X 欧拉角。
/// @param target_pose_zyx_euler 长度固定为 6，格式为 [px, py, pz, rz, ry, rx]。
/// @return 返回 HicStatus 状态码。
/// @note 位置分量 px/py/pz 单位为 mm。
/// @note 姿态分量 rz/ry/rx 单位为 deg，表示依次绕固定基坐标系的 Z、Y、X 轴旋转。
/// @note 该接口会在 hic_c_api.cpp 内部把 Z-Y-X 固定轴欧拉角转换成四元数，
/// 再复用内部的四元数位姿目标链路。
HIC_EXPORT int hic_set_cartesian_fixed_pose_target_zyx_euler(
	const double target_pose_zyx_euler[HIC_CARTESIAN_DIM]);

/// @brief 抓取当前末端位姿作为定点位姿目标。
HIC_EXPORT int hic_capture_current_pose_as_fixed_target();

/// @brief 设置轨迹模式下的在线位姿和速度目标，输入姿态格式为固定坐标系 Z-Y-X 欧拉角。
/// @param target_pose_zyx_euler 长度固定为 6，格式为 [px, py, pz, rz, ry, rx]。
/// @param target_velocity 长度固定为 6，格式为 [vx, vy, vz, wx, wy, wz]。
/// @return 返回 HicStatus 状态码。
/// @note `target_pose_zyx_euler` 的位置分量单位为 mm，姿态分量单位为 deg。
/// @note 其中 rz/ry/rx 表示依次绕固定基坐标系的 Z、Y、X 轴旋转。
/// @note `target_velocity` 的前三维线速度单位为 mm/s，后三维角速度单位为 deg/s。
/// @note 该接口会在 hic_c_api.cpp 内部先把欧拉角位姿转换成四元数位姿，再复用已有轨迹目标链路。
HIC_EXPORT int hic_set_cartesian_trajectory_target_zyx_euler(
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
HIC_EXPORT int hic_start_force_control_mode(int force_control_mode);

/// @brief 准备退出当前力控模式。
HIC_EXPORT int hic_prepare_stop_force_control_mode();

// ============================================================
// 5. 命令输出与状态读取
// ============================================================

/// @brief 获取当前力控计算得到的目标电流命令，单位 A。
HIC_EXPORT int hic_get_force_control_current_commands(
	double motor_current_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS]);

/// @brief 获取当前力控计算得到的目标力矩命令，单位 N.m。
HIC_EXPORT int hic_get_force_control_torque_commands(
	double joint_torque_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS]);

/// @brief 获取当前活动控制状态。
HIC_EXPORT int hic_get_active_control_state(HicActiveControlState* state_out);

/// @brief 获取当前机器人关节状态。
/// @note 返回值沿用内部 SI 语义：关节位置单位 rad，关节速度单位 rad/s，
/// 关节加速度单位 rad/s^2，电流单位 A，力矩单位 N.m。
HIC_EXPORT int hic_get_robot_state(HicRobotState* state_out);

/// @brief 获取运动约束状态。
HIC_EXPORT int hic_get_motion_constraint_status(HicMotionConstraintStatus* status_out);

/// @brief 获取内部动力学估计项。
HIC_EXPORT int hic_get_estimated_dynamics_torques(HicEstimatedDynamicsTorques* torques_out);

/// @brief 获取当前末端位姿和速度。
/// @note 返回值沿用内部 SI 语义：pose 的位置分量单位 m，四元数分量无单位；
/// twist 前三维线速度单位 m/s，后三维角速度单位 rad/s。
HIC_EXPORT int hic_get_current_cartesian_state(HicCartesianState* state_out);

// ============================================================
// 6. 独立计算与外设辅助
// ============================================================

/// @brief 独立计算逆动力学各分项，不改变内部控制状态。
HIC_EXPORT int hic_compute_inverse_dynamics_torque(
	const HicDynamicsComputeInput* input,
	HicDynamicsComputeOutput* output);

/// @brief 计算重力项力矩，单位 N.m。
/// @note 输入关节位置数组按外部接口语义使用 deg。
HIC_EXPORT int hic_compute_gravity_torque(
	const double joint_position[HIC_MAX_JOINTS],
	double gravity_torque[HIC_MAX_JOINTS]);

/// @brief 计算重力加科氏/离心项合力矩，单位 N.m。
/// @note 输入关节位置单位为 deg，输入关节速度单位为 deg/s。
HIC_EXPORT int hic_compute_gravity_coriolis_torque(
	const double joint_position[HIC_MAX_JOINTS],
	const double joint_velocity[HIC_MAX_JOINTS],
	double torque[HIC_MAX_JOINTS]);


/// @brief 使能力传感器参与碰撞检测。
HIC_EXPORT int hic_enable_wrench_sensor_for_collision_detection(bool enable);

/// @brief 使能力传感器参与力控前馈。
HIC_EXPORT int hic_enable_wrench_sensor_for_force_feedforward(bool enable);


// ============================================================
// 6. 通用调试接口
// ============================================================


/// @brief 获取当前力控子模式，对应 HicForceControlMode。
/// @note 若当前不处于力控模式，通常返回 HIC_FORCE_CONTROL_MODE_NONE。
HIC_EXPORT int hic_get_force_control_mode();

/// @brief 获取最近一次 coordinator 执行返回的状态码。
HIC_EXPORT int hic_get_last_status();

/// @brief 复位控制器内部运行状态和缓存，但不重新做基础初始化。
HIC_EXPORT int hic_reset_control();



#ifdef __cplusplus
}
#endif
