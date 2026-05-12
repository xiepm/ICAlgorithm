/// @file hic_c_api.cpp
/// @brief HIC 对外 C 接口实现文件。
/// @note 该文件的职责是：
/// 1. 承接 hic_c_api.h 中正式暴露的 C ABI；
/// 2. 完成空指针、初始化状态等边界检查；
/// 3. 将外部结构体/裸数组转发给内部 HicControlCoordinator；
/// 4. 在失败场景下对命令结果数组做安全清零，避免上层误用旧数据。
/// @note 当前版本采用“单 coordinator 单实例”模型，因此本文件内部通过一个全局 shared_ptr
/// 持有唯一的控制器对象。后续如果要扩展到多机器人/多实例，需要从这里改成 handle 风格接口。
#include "hic_controller/interface/hic_c_api.h"

#include "hic_controller/coordinator/hic_control_coordinator.h"

#include <cmath>
#include <cstring>
#include <memory>

namespace
{
/// @brief 当前进程内唯一的 HIC 控制协调器实例。
/// @note 由 hic_initialize_control() 创建或重建。
std::shared_ptr<hic::HicControlCoordinator> g_hicCoordinator;

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMmToM = 0.001;

template <typename T>
void clearArray(T* data, int size)
{
	/// @brief 将命令结果数组清零。
	/// @note 主要用于错误返回前的兜底处理，避免调用方拿到半旧半新的计算结果。
	if (!data)
	{
		return;
	}
	for (int i = 0; i < size; ++i)
	{
		data[i] = T();
	}
}

bool coordinatorReady()
{
	/// @brief 判断 coordinator 是否已经创建。
	return static_cast<bool>(g_hicCoordinator);
}

double timeConstantToAlpha(double controlPeriod, double timeConstant)
{
	if (controlPeriod <= 0.0)
	{
		return 1.0;
	}
	if (timeConstant <= 0.0)
	{
		return 1.0;
	}
	const double alpha = controlPeriod / (controlPeriod + timeConstant);
	if (alpha < 0.0)
	{
		return 0.0;
	}
	if (alpha > 1.0)
	{
		return 1.0;
	}
	return alpha;
}

double degToRad(double deg)
{
	return deg * kDegToRad;
}

double mmToM(double mm)
{
	return mm * kMmToM;
}

void fixedZyxEulerDegToQuaternion(
	double rzDeg,
	double ryDeg,
	double rxDeg,
	double quaternion[4])
{
	const double halfRz = 0.5 * degToRad(rzDeg);
	const double halfRy = 0.5 * degToRad(ryDeg);
	const double halfRx = 0.5 * degToRad(rxDeg);

	const double cz = std::cos(halfRz);
	const double sz = std::sin(halfRz);
	const double cy = std::cos(halfRy);
	const double sy = std::sin(halfRy);
	const double cx = std::cos(halfRx);
	const double sx = std::sin(halfRx);

	// 外部姿态约定为绕固定基坐标系依次执行 Z、Y、X 旋转，
	// 这里按 q = qz * qy * qx 组合得到内部使用的单位四元数。
	quaternion[0] = cz * cy * cx + sz * sy * sx;
	quaternion[1] = cz * cy * sx - sz * sy * cx;
	quaternion[2] = cz * sy * cx + sz * cy * sx;
	quaternion[3] = sz * cy * cx - cz * sy * sx;
}

void convertPoseZyxEulerMmDegToQuaternionPose(
	const double poseZyxEuler[HIC_CARTESIAN_DIM],
	double quaternionPose[HIC_POSE_DIM])
{
	quaternionPose[0] = mmToM(poseZyxEuler[0]);
	quaternionPose[1] = mmToM(poseZyxEuler[1]);
	quaternionPose[2] = mmToM(poseZyxEuler[2]);
	fixedZyxEulerDegToQuaternion(
		poseZyxEuler[3], poseZyxEuler[4], poseZyxEuler[5], &quaternionPose[3]);
}

int requireCoordinator()
{
	/// @brief 返回统一的“是否已初始化”状态码。
	/// @note 便于低频 set 接口复用，减少重复样板代码。
	return coordinatorReady() ? HIC_STATUS_OK : HIC_STATUS_ERROR_INIT;
}

}

extern "C"
{

const char* hic_get_version()
{
	return HIC_VERSION_STRING;
}

int hic_initialize_control(RTS_IEC_INT groupId, const HicInitializeConfig* config)
{
	/// @brief 创建新的 coordinator，并按外部配置完成整体初始化。
	/// @note 当前重复调用会直接覆盖旧实例，因此会丢弃之前缓存的内部状态。
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	HicControlConfig internalConfig = {};
	internalConfig.jointCount = config->jointCount;
	internalConfig.controlPeriod = config->controlPeriod;
	internalConfig.robotType = config->robotType;
	internalConfig.enableGravityCompensation = true;
	internalConfig.enableCoriolisCompensation = false;
	internalConfig.enableTorqueRateLimit = true;
	internalConfig.enableCurrentLimit = true;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		internalConfig.kinematicParams[i] = config->kinematicDHParams[i];
		// 运行参数在初始化之后再由各个 set 接口下发。
		// 这里给执行器换算参数一个保守有效默认值，避免内部状态观测器因 NaN/Inf 直接失效。
		internalConfig.torqueConstant[i] = 1.0;
		internalConfig.gearRatio[i] = 1.0;
		internalConfig.transmissionEfficiency[i] = 1.0;
	}
	g_hicCoordinator.reset(new hic::HicControlCoordinator());
	return g_hicCoordinator->initialize(internalConfig);
}

int hic_set_dynamics_linear_parameters(RTS_IEC_INT groupId, const double dynamic_params[HIC_MAX_DYNAMIC_PARAMS])
{
	/// @brief 设置线性化动力学参数。
	/// @note 参数最终会进入 dynamics adapter，由不同 robotType 的后端各自解释。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	return g_hicCoordinator->setDynamicsLinearParameters(dynamic_params);
}

int hic_set_payload_mass_properties(
	RTS_IEC_INT groupId,
	double mass,
	const double center_of_mass[3])
{
	/// @brief 设置末端负载质量与质心。
	/// @note 当前接口链路已经保留，但具体 backend 是否真正实现取决于对应机型动力学模型。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!center_of_mass)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	double center_of_mass_m[3] = {
		mmToM(center_of_mass[0]),
		mmToM(center_of_mass[1]),
		mmToM(center_of_mass[2])
	};
	return g_hicCoordinator->setPayloadMassProperties(mass, center_of_mass_m);
}

int hic_set_motor_torque_conversion_parameters(
	RTS_IEC_INT groupId,
	const double torque_constant[HIC_MAX_JOINTS],
	const double gear_ratio[HIC_MAX_JOINTS],
	const double transmission_efficiency[HIC_MAX_JOINTS])
{
	/// @brief 设置电机电流到关节力矩的换算参数。
	/// @note 这些参数既会影响补偿估计，也会影响力矩到电流的输出换算。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	return g_hicCoordinator->setMotorTorqueConversionParameters(
		torque_constant, gear_ratio, transmission_efficiency);
}

int hic_set_joint_position_limits(RTS_IEC_INT groupId, const HicJointRangeLimits* limits)
{
	/// @brief 设置关节位置安全边界。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!limits)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	double lower[HIC_MAX_JOINTS] = { 0.0 };
	double upper[HIC_MAX_JOINTS] = { 0.0 };
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		lower[i] = degToRad(limits->lower[i]);
		upper[i] = degToRad(limits->upper[i]);
	}
	return g_hicCoordinator->setJointPositionLimits(lower, upper);
}

int hic_set_joint_velocity_limits(RTS_IEC_INT groupId, const HicJointMaxLimits* limits)
{
	/// @brief 设置关节速度最大绝对值。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!limits)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	double lower[HIC_MAX_JOINTS] = { 0.0 };
	double upper[HIC_MAX_JOINTS] = { 0.0 };
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		lower[i] = -degToRad(limits->maxAbs[i]);
		upper[i] = degToRad(limits->maxAbs[i]);
	}
	return g_hicCoordinator->setJointVelocityLimits(lower, upper);
}

int hic_set_joint_acceleration_limits(RTS_IEC_INT groupId, const HicJointMaxLimits* limits)
{
	/// @brief 设置关节加速度最大绝对值。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!limits)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	double lower[HIC_MAX_JOINTS] = { 0.0 };
	double upper[HIC_MAX_JOINTS] = { 0.0 };
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		lower[i] = -degToRad(limits->maxAbs[i]);
		upper[i] = degToRad(limits->maxAbs[i]);
	}
	return g_hicCoordinator->setJointAccelerationLimits(lower, upper);
}

int hic_set_joint_torque_limits(RTS_IEC_INT groupId, const HicJointRangeLimits* limits)
{
	/// @brief 设置关节力矩安全边界。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!limits)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return g_hicCoordinator->setJointTorqueLimits(limits->lower, limits->upper);
}

int hic_set_motor_current_limits(RTS_IEC_INT groupId, const HicJointRangeLimits* limits)
{
	/// @brief 设置电机电流安全边界。
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!limits)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return g_hicCoordinator->setMotorCurrentLimits(limits->lower, limits->upper);
}

int hic_set_robot_mounting_angles(RTS_IEC_INT groupId, double rotation, double tilt)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}

	const double rotation_rad = degToRad(rotation);
	const double tilt_rad = degToRad(tilt);
	constexpr double kGravity = 9.81;
	const double horizontal = kGravity * std::sin(tilt_rad);
	const double gx = horizontal * std::cos(rotation_rad);
	const double gy = horizontal * std::sin(rotation_rad);
	const double gz = -kGravity * std::cos(tilt_rad);
	return hic_set_gravity_vector_in_base(groupId, gx, gy, gz);
}

int hic_update_state_estimates(RTS_IEC_INT groupId, const HicStateEstimateInput* input)
{
	/// @brief 调试期高频主输入接口。
	/// @note 当前版本主链路仍以 jointPosition 和 motorCurrent 为主；
	/// commandJointPosition 已作为外部接口字段保留，便于后续扩展跟踪误差相关逻辑。
	/// currentTime 主要用于时间对齐和内部差分估计。
	/// @note jointVelocity / jointAcceleration 不再由外部显式输入，
	/// 而是交给 robotStateObserver 基于“位置差分 + 时间戳”在内部估计。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!input)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	double joint_position_rad[HIC_MAX_JOINTS] = { 0.0 };
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		joint_position_rad[i] = degToRad(input->jointPosition[i]);
	}

	return g_hicCoordinator->updateRobotState(
		joint_position_rad, input->motorCurrent, input->currentTime);
}

int hic_set_cartesian_impedance_gains(
	RTS_IEC_INT groupId,
	const HicImpedanceGains* gains)
{
	/// @brief 设置 6 维笛卡尔阻抗增益。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!gains)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return g_hicCoordinator->setCartesianImpedanceGains(*gains);
}

int hic_set_force_control_nullspace_config(RTS_IEC_INT groupId, const HicNullspaceControlConfig* config)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return g_hicCoordinator->setNullspaceConfig(*config);
}

int hic_capture_current_joint_position_as_nullspace_target(RTS_IEC_INT groupId)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentJointPositionAsNullspaceTarget();
}

int hic_set_cartesian_fixed_position_target(RTS_IEC_INT groupId, const double target_position[3])
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!target_position)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	double target_position_m[3] = {
		mmToM(target_position[0]),
		mmToM(target_position[1]),
		mmToM(target_position[2])
	};
	return g_hicCoordinator->setFixedTargetPosition(target_position_m);
}

int hic_capture_current_position_as_fixed_target(RTS_IEC_INT groupId)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentPositionAsFixedTarget();
}

int hic_capture_current_pose_as_fixed_target(RTS_IEC_INT groupId)
{
	/// @brief 将当前末端位姿抓取为固定点阻抗目标。
	/// @note 典型用法是在“机器人已经到达一个自然姿态”后调用，
	/// 让后续阻抗控制围绕这个姿态工作。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentPoseAsFixedTarget();
}

int hic_set_joint_torque_sensor_parameters(RTS_IEC_INT groupId, const HicTorqueSensorConfig* config)
{
	/// @brief 配置关节扭矩传感器的通道映射、零漂、比例系数和故障阈值。
	/// @note 该接口对应 forceObserver 的“传感器数据处理层”，不参与内部动力学建模。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return g_hicCoordinator->setTorqueSensorConfig(*config);
}

int hic_update_raw_joint_torque_sensor(RTS_IEC_INT groupId, const double* raw_torque_by_hardware_channel)
{
	/// @brief 输入外部回传的原始关节扭矩传感器值。
	/// @note 进入 forceObserver 后会完成标定、滤波和故障检查，
	/// 但不会直接参与当前阻抗主循环的动力学补偿计算。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->updateRawJointTorqueSensor(raw_torque_by_hardware_channel);
}

int hic_update_wrench_sensor_data(RTS_IEC_INT groupId, const HicWrenchSensorInput* input)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!input)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_update_dual_encoder_joint_position(RTS_IEC_INT groupId, const HicDualEncoderInput* input)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!input)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_set_gravity_vector_in_base(RTS_IEC_INT groupId, double gx, double gy, double gz)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	return g_hicCoordinator->setGravityVector(gx, gy, gz);
}

int hic_set_state_filter_config(RTS_IEC_INT groupId, const HicStateFilterConfig* config)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicRobotStateObserverConfig observerConfig = {};
	int status = g_hicCoordinator->getRobotStateObserverConfig(observerConfig);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	if (config->jointCount != observerConfig.jointCount)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	observerConfig.enablePositionFilter = config->enable_position_filter;
	observerConfig.enableVelocityFilter = config->enable_velocity_filter;
	observerConfig.enableAccelerationFilter = config->enable_acceleration_filter;
	observerConfig.enableMotorCurrentFilter = config->enable_current_filter;
	for (int i = 0; i < observerConfig.jointCount; ++i)
	{
		observerConfig.positionFilterAlpha[i] =
			timeConstantToAlpha(observerConfig.controlPeriod, config->position_filter_time_constant[i]);
		observerConfig.velocityFilterAlpha[i] =
			timeConstantToAlpha(observerConfig.controlPeriod, config->velocity_filter_time_constant[i]);
		observerConfig.accelerationFilterAlpha[i] =
			timeConstantToAlpha(observerConfig.controlPeriod, config->acceleration_filter_time_constant[i]);
		observerConfig.motorCurrentFilterAlpha[i] =
			timeConstantToAlpha(observerConfig.controlPeriod, config->current_filter_time_constant[i]);
	}
	return g_hicCoordinator->setRobotStateObserverConfig(observerConfig);
}

int hic_set_zero_force_safety_config(RTS_IEC_INT groupId, const HicZeroForceSafetyConfig* config)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_set_collision_detection_config(RTS_IEC_INT groupId, const HicCollisionDetectionConfig* config)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_set_friction_compensation_config(RTS_IEC_INT groupId, const HicFrictionCompensationConfig* config)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_set_dual_encoder_assist_config(RTS_IEC_INT groupId, const HicDualEncoderAssistConfig* config)
{
	const int readyStatus = requireCoordinator();
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_get_active_control_state(
	RTS_IEC_INT groupId,
	HicActiveControlState* state_out)
{
	/// @brief 获取当前活动控制状态的统一快照。
	if (!coordinatorReady())
	{
		if (state_out)
		{
			state_out->controlMode = HIC_CONTROL_MODE_IDLE;
			state_out->forceControlMode = HIC_FORCE_CONTROL_MODE_NONE;
			state_out->supportsCurrentLoop = false;
			state_out->supportsTorqueLoop = false;
			state_out->nullspaceEnabled = false;
		}
		return HIC_STATUS_ERROR_INIT;
	}
	if (!state_out)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	const HicControlMode controlMode = g_hicCoordinator->getControlMode();
	const HicForceControlMode forceControlMode = g_hicCoordinator->getForceControlMode();

	state_out->controlMode = static_cast<int>(controlMode);
	state_out->forceControlMode = static_cast<int>(forceControlMode);
	state_out->supportsCurrentLoop = (forceControlMode != HIC_FORCE_CONTROL_MODE_NONE);
	state_out->supportsTorqueLoop = (forceControlMode != HIC_FORCE_CONTROL_MODE_NONE);
	state_out->nullspaceEnabled = g_hicCoordinator->isNullspaceEnabled();
	return HIC_STATUS_OK;
}

int hic_get_current_cartesian_state(
	RTS_IEC_INT groupId,
	HicCartesianState* state_out)
{
	/// @brief 获取当前末端位姿和 twist。
	/// @note 实际数据来自：
	/// 1. robotStateObserver 内部保存的关节状态；
	/// 2. kinematics adapter 的正运动学和末端速度计算。
	if (!coordinatorReady())
	{
		if (state_out)
		{
			clearArray(state_out->pose, HIC_POSE_DIM);
			clearArray(state_out->twist, HIC_CARTESIAN_DIM);
		}
		return HIC_STATUS_ERROR_INIT;
	}
	if (!state_out)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	const int status = g_hicCoordinator->getCurrentCartesianState(state_out->pose, state_out->twist);
	return status;
}

int hic_get_last_status(RTS_IEC_INT groupId)
{
	/// @brief 返回 coordinator 最近一次执行的状态码。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->getLastStatus();
}

int hic_get_force_control_mode(RTS_IEC_INT groupId)
{
	/// @brief 返回当前力控子模式。
	if (!coordinatorReady())
	{
		return HIC_FORCE_CONTROL_MODE_NONE;
	}
	return static_cast<int>(g_hicCoordinator->getForceControlMode());
}

int hic_reset_control(RTS_IEC_INT groupId)
{
	/// @brief 复位内部控制状态与缓存。
	/// @note 这里不会删除 coordinator，也不会重新加载基础配置；
	/// 作用更接近“清空本轮运行状态”。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->reset();
}

int hic_set_cartesian_fixed_pose_target_zyx_euler(
	RTS_IEC_INT groupId,
	const double target_pose_zyx_euler[HIC_CARTESIAN_DIM])
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!target_pose_zyx_euler)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	double target_pose_quaternion[HIC_POSE_DIM] = { 0.0 };
	convertPoseZyxEulerMmDegToQuaternionPose(target_pose_zyx_euler, target_pose_quaternion);
	return g_hicCoordinator->setFixedTargetPose(target_pose_quaternion);
}

int hic_set_cartesian_trajectory_target_zyx_euler(
	RTS_IEC_INT groupId,
	const double target_pose_zyx_euler[HIC_CARTESIAN_DIM],
	const double target_velocity[HIC_CARTESIAN_DIM])
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!target_pose_zyx_euler || !target_velocity)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	double target_pose_quaternion[HIC_POSE_DIM] = { 0.0 };
	convertPoseZyxEulerMmDegToQuaternionPose(target_pose_zyx_euler, target_pose_quaternion);
	double target_velocity_si[HIC_CARTESIAN_DIM] = {
		mmToM(target_velocity[0]),
		mmToM(target_velocity[1]),
		mmToM(target_velocity[2]),
		degToRad(target_velocity[3]),
		degToRad(target_velocity[4]),
		degToRad(target_velocity[5])
	};
	return g_hicCoordinator->setOnlineTargetPose(target_pose_quaternion, target_velocity_si);
}

int hic_start_force_control_mode(RTS_IEC_INT groupId, int force_control_mode)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}

	switch (static_cast<HicForceControlMode>(force_control_mode))
	{
	case HIC_FORCE_CONTROL_MODE_ZERO_FORCE:
		return g_hicCoordinator->startForceControlZeroForceMode();
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION:
		return g_hicCoordinator->startForceControlCartesianFixedPositionMode();
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE:
		return g_hicCoordinator->startForceControlCartesianFixedPoseMode();
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY:
		return g_hicCoordinator->startForceControlCartesianTrajectoryMode();
	case HIC_FORCE_CONTROL_MODE_NONE:
	default:
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}
}

int hic_prepare_stop_force_control_mode(RTS_IEC_INT groupId)
{
	/// @brief 统一的力控模式退出入口。
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}

	return g_hicCoordinator->prepareStopForceControlMode();
}

int hic_get_force_control_current_commands(
	RTS_IEC_INT groupId,
	double motor_current_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS])
{
	/// @brief 力控大类统一电流环目标命令计算结果入口。
	if (!motor_current_commands || !joint_protection_status)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (!coordinatorReady())
	{
		clearArray(motor_current_commands, HIC_MAX_JOINTS);
		clearArray(joint_protection_status, HIC_MAX_JOINTS);
		return HIC_STATUS_ERROR_INIT;
	}

	const HicForceControlMode forceControlMode = g_hicCoordinator->getForceControlMode();
	if (forceControlMode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		return g_hicCoordinator->computeZeroForceCurrentCommand(
			motor_current_commands, joint_protection_status);
	}
	if (forceControlMode == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION ||
	    forceControlMode == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE ||
	    forceControlMode == HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY)
	{
		return g_hicCoordinator->computeCartesianImpedanceCurrentCommand(
			motor_current_commands, joint_protection_status);
	}

	clearArray(motor_current_commands, HIC_MAX_JOINTS);
	clearArray(joint_protection_status, HIC_MAX_JOINTS);
	return HIC_STATUS_ERROR_INVALID_PARAM;
}

int hic_get_force_control_torque_commands(
	RTS_IEC_INT groupId,
	double joint_torque_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS])
{
	/// @brief 力控大类统一力矩环目标命令计算结果入口。
	if (!joint_torque_commands || !joint_protection_status)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (!coordinatorReady())
	{
		clearArray(joint_torque_commands, HIC_MAX_JOINTS);
		clearArray(joint_protection_status, HIC_MAX_JOINTS);
		return HIC_STATUS_ERROR_INIT;
	}

	const HicForceControlMode forceControlMode = g_hicCoordinator->getForceControlMode();
	if (forceControlMode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		return g_hicCoordinator->computeZeroForceTorqueCommand(
			joint_torque_commands, joint_protection_status);
	}
	if (forceControlMode == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION ||
	    forceControlMode == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE ||
	    forceControlMode == HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY)
	{
		return g_hicCoordinator->computeCartesianImpedanceTorqueCommand(
			joint_torque_commands, joint_protection_status);
	}

	clearArray(joint_torque_commands, HIC_MAX_JOINTS);
	clearArray(joint_protection_status, HIC_MAX_JOINTS);
	return HIC_STATUS_ERROR_INVALID_PARAM;
}

int hic_get_robot_state(
	RTS_IEC_INT groupId,
	HicRobotState* state_out)
{
	/// @brief 获取当前关节状态观测结果。
	if (!coordinatorReady())
	{
		if (state_out)
		{
			clearArray(state_out->jointPosition, HIC_MAX_JOINTS);
			clearArray(state_out->jointVelocity, HIC_MAX_JOINTS);
			clearArray(state_out->jointAcceleration, HIC_MAX_JOINTS);
			clearArray(state_out->motorCurrent, HIC_MAX_JOINTS);
			clearArray(state_out->jointMeasuredTorque, HIC_MAX_JOINTS);
			clearArray(state_out->motorEstimatedTorque, HIC_MAX_JOINTS);
		}
		return HIC_STATUS_ERROR_INIT;
	}
	if (!state_out)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	const int status = g_hicCoordinator->getRobotState(*state_out);
	return status;
}

int hic_get_motion_constraint_status(RTS_IEC_INT groupId, HicMotionConstraintStatus* status_out)
{
	if (!status_out)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	std::memset(status_out, 0, sizeof(*status_out));
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	status_out->jointCount = g_hicCoordinator->getJointCount();
	status_out->main_reason = HIC_PROTECTION_NONE;
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_get_estimated_dynamics_torques(
	RTS_IEC_INT groupId,
	HicEstimatedDynamicsTorques* torques_out)
{
	/// @brief 获取内部动力学估计项。
	/// @note 当前接口语义已经固定，但实现仍未完全接线。
	/// 现阶段统一清零后返回 NOT_IMPLEMENTED，避免上层误以为结果有效。
	if (!coordinatorReady())
	{
		if (torques_out)
		{
			torques_out->terms.jointCount = 0;
			clearArray(torques_out->terms.inertiaTorque, HIC_MAX_JOINTS);
			clearArray(torques_out->terms.gravityTorque, HIC_MAX_JOINTS);
			clearArray(torques_out->terms.coriolisTorque, HIC_MAX_JOINTS);
			clearArray(torques_out->terms.frictionTorque, HIC_MAX_JOINTS);
			clearArray(torques_out->terms.modelTorque, HIC_MAX_JOINTS);
			clearArray(torques_out->externalTorque, HIC_MAX_JOINTS);
		}
		return HIC_STATUS_ERROR_INIT;
	}
	if (!torques_out)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	torques_out->terms.jointCount = 0;
	clearArray(torques_out->terms.inertiaTorque, HIC_MAX_JOINTS);
	clearArray(torques_out->terms.gravityTorque, HIC_MAX_JOINTS);
	clearArray(torques_out->terms.coriolisTorque, HIC_MAX_JOINTS);
	clearArray(torques_out->terms.frictionTorque, HIC_MAX_JOINTS);
	clearArray(torques_out->terms.modelTorque, HIC_MAX_JOINTS);
	clearArray(torques_out->externalTorque, HIC_MAX_JOINTS);
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_compute_inverse_dynamics_torque(
	RTS_IEC_INT groupId,
	const HicDynamicsComputeInput* input,
	HicDynamicsComputeOutput* output)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!input || !output)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	std::memset(output, 0, sizeof(*output));
	output->jointCount = input->jointCount;
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_compute_gravity_torque(
	RTS_IEC_INT groupId,
	const double joint_position[HIC_MAX_JOINTS],
	double gravity_torque[HIC_MAX_JOINTS])
{
	if (!coordinatorReady())
	{
		clearArray(gravity_torque, HIC_MAX_JOINTS);
		return HIC_STATUS_ERROR_INIT;
	}
	if (!joint_position || !gravity_torque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	clearArray(gravity_torque, HIC_MAX_JOINTS);
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_compute_gravity_coriolis_torque(
	RTS_IEC_INT groupId,
	const double joint_position[HIC_MAX_JOINTS],
	const double joint_velocity[HIC_MAX_JOINTS],
	double torque[HIC_MAX_JOINTS])
{
	if (!coordinatorReady())
	{
		clearArray(torque, HIC_MAX_JOINTS);
		return HIC_STATUS_ERROR_INIT;
	}
	if (!joint_position || !joint_velocity || !torque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	clearArray(torque, HIC_MAX_JOINTS);
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_enable_wrench_sensor_for_collision_detection(RTS_IEC_INT groupId, bool enable)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	(void)enable;
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_enable_wrench_sensor_for_force_feedforward(RTS_IEC_INT groupId, bool enable)
{
	if (!coordinatorReady())
	{
		return HIC_STATUS_ERROR_INIT;
	}
	(void)enable;
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

} // extern "C"
