/// @file hic_c_api.cpp
/// @note 该文件的职责是：
/// 2. 完成空指针、初始化状态等边界检查；
/// @note 当前版本采用“单 coordinator 单实例”模型，因此本文件内部通过一个全局 shared_ptr
#include "hic_controller/interface/hic_c_api.h"

#include "hic_controller/coordinator/hic_control_coordinator.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>

namespace
{
// Keep one coordinator per group. group 0 and group 1 must not overwrite each other.
std::map<RTS_IEC_INT, std::shared_ptr<hic::HicControlCoordinator>> g_hicCoordinators;
std::mutex g_hicCoordinatorsMutex;
thread_local std::shared_ptr<hic::HicControlCoordinator> g_hicCoordinator;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;
constexpr double kMmToM = 0.001;

template <typename T>
void clearArray(T* data, int size)
{
	if (!data)
	{
		return;
	}
	for (int i = 0; i < size; ++i)
	{
		data[i] = T();
	}
}

bool coordinatorReady(RTS_IEC_INT groupId)
{
	std::lock_guard<std::mutex> lock(g_hicCoordinatorsMutex);
	std::map<RTS_IEC_INT, std::shared_ptr<hic::HicControlCoordinator>>::const_iterator it =
		g_hicCoordinators.find(groupId);
	if (it == g_hicCoordinators.end() || !it->second)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[hic_c_api] coordinator not initialized for group=%d activeGroupCount=%u\n",
			groupId,
			static_cast<unsigned>(g_hicCoordinators.size()));
#endif
		g_hicCoordinator.reset();
		return false;
	}
	g_hicCoordinator = it->second;
	return true;
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

int clampDebugJointCount(int jointCount)
{
	if (jointCount <= 0)
	{
		return HIC_MAX_JOINTS;
	}
	if (jointCount > HIC_MAX_JOINTS)
	{
		return HIC_MAX_JOINTS;
	}
	return jointCount;
}

bool shouldDebugPrint(int& counter)
{
	return ((counter++ % 1000) == 0);
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

int requireCoordinator(RTS_IEC_INT groupId)
{
	return coordinatorReady(groupId) ? HIC_STATUS_OK : HIC_STATUS_ERROR_INIT;
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
	// Create or replace the coordinator that belongs to this groupId only.
	if (!config)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[hic_initialize_control] group=%d config=null status=%d\n",
			groupId,
			static_cast<int>(HIC_STATUS_ERROR_NULL_POINTER));
#endif
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

#ifdef HIC_ENABLE_DEBUG_PRINT
	std::fprintf(stderr,
		"[hic_initialize_control] external input: group=%d jointCount=%d controlPeriod=%.9f robotType=%d\n",
		groupId,
		config->jointCount,
		config->controlPeriod,
		config->robotType);
#endif

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
		// Runtime conversion parameters can be overwritten later by the set API.
		internalConfig.torqueConstant[i] = 1.0;
		internalConfig.gearRatio[i] = 1.0;
		internalConfig.transmissionEfficiency[i] = 1.0;
#ifdef HIC_ALLOW_DEBUG_PARAM_VALIDATION_BYPASS
		// Debug bring-up defaults: keep missing external limit configuration from clamping everything to zero.
		internalConfig.lowerJointLimit[i] = -1.0e6;
		internalConfig.upperJointLimit[i] = 1.0e6;
		internalConfig.lowerJointVelocity[i] = -1.0e6;
		internalConfig.upperJointVelocity[i] = 1.0e6;
		internalConfig.lowerJointAcceleration[i] = -1.0e6;
		internalConfig.upperJointAcceleration[i] = 1.0e6;
		internalConfig.lowerJointTorque[i] = -1.0e6;
		internalConfig.upperJointTorque[i] = 1.0e6;
		internalConfig.maxJointTorque[i] = 1.0e6;
		internalConfig.lowerMotorCurrent[i] = -1000.0;
		internalConfig.upperMotorCurrent[i] = 1000.0;
		internalConfig.maxJointCurrent[i] = 1000.0;
#endif
	}

	std::shared_ptr<hic::HicControlCoordinator> coordinator(new hic::HicControlCoordinator());
	const int status = coordinator->initialize(internalConfig);
#ifdef HIC_ENABLE_DEBUG_PRINT
	std::fprintf(stderr,
		"[hic_initialize_control] group=%d jointCount=%d controlPeriod=%.9f robotType=%d status=%d\n",
		groupId,
		config->jointCount,
		config->controlPeriod,
		config->robotType,
		status);
#endif
	if (status != HIC_STATUS_OK)
	{
		std::lock_guard<std::mutex> lock(g_hicCoordinatorsMutex);
		g_hicCoordinators.erase(groupId);
		g_hicCoordinator.reset();
		return status;
	}

	{
		std::lock_guard<std::mutex> lock(g_hicCoordinatorsMutex);
		g_hicCoordinators[groupId] = coordinator;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[hic_initialize_control] stored coordinator group=%d activeGroupCount=%u\n",
			groupId,
			static_cast<unsigned>(g_hicCoordinators.size()));
#endif
	}
	g_hicCoordinator = coordinator;
	return status;
}

int hic_set_dynamics_linear_parameters(RTS_IEC_INT groupId, const double dynamic_params[HIC_MAX_DYNAMIC_PARAMS])
{
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}

#ifdef HIC_ENABLE_DEBUG_PRINT
	if (!torque_constant || !gear_ratio || !transmission_efficiency)
	{
		std::fprintf(stderr,
			"[hic_set_motor_torque_conversion_parameters] group=%d "
			"torque_constant=%p gear_ratio=%p transmission_efficiency=%p\n",
			groupId,
			static_cast<const void*>(torque_constant),
			static_cast<const void*>(gear_ratio),
			static_cast<const void*>(transmission_efficiency));
	}
	else
	{
		const int debug_joint_count = clampDebugJointCount(g_hicCoordinator->getJointCount());
		std::fprintf(stderr,
			"[hic_set_motor_torque_conversion_parameters] group=%d jointCount=%d\n",
			groupId,
			debug_joint_count);
		std::fflush(stderr);
		for (int i = 0; i < debug_joint_count; ++i)
		{
			std::fprintf(stderr,
				"  joint[%d] torque_constant=%.6f gear_ratio=%.6f transmission_efficiency=%.6f\n",
				i,
				torque_constant[i],
				gear_ratio[i],
				transmission_efficiency[i]);
		}
	}
#endif

	return g_hicCoordinator->setMotorTorqueConversionParameters(
		torque_constant, gear_ratio, transmission_efficiency);
}

int hic_set_joint_position_limits(RTS_IEC_INT groupId, const HicJointRangeLimits* limits)
{
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	if (!limits)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[hic_set_motor_current_limits] group=%d limits=null status=%d\n",
			groupId,
			static_cast<int>(HIC_STATUS_ERROR_NULL_POINTER));
#endif
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
#ifdef HIC_ENABLE_DEBUG_PRINT
	{
		const int debug_joint_count = clampDebugJointCount(g_hicCoordinator->getJointCount());
		std::fprintf(stderr,
			"[hic_set_motor_current_limits] group=%d jointCount=%d input limits:\n",
			groupId,
			debug_joint_count);
		std::fflush(stderr);
		for (int i = 0; i < debug_joint_count; ++i)
		{
			std::fprintf(stderr,
				"  current_limit[%d] lower=%.6f upper=%.6f\n",
				i,
				limits->lower[i],
				limits->upper[i]);
		}
	}
#endif
	const int status = g_hicCoordinator->setMotorCurrentLimits(limits->lower, limits->upper);
#ifdef HIC_ENABLE_DEBUG_PRINT
	std::fprintf(stderr,
		"[hic_set_motor_current_limits] group=%d status=%d\n",
		groupId,
		status);
#endif
	return status;
}
int hic_set_robot_mounting_angles(RTS_IEC_INT groupId, double rotation, double tilt)
{
	const int readyStatus = requireCoordinator(groupId);
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
	/// @brief 调试期高频状态输入接口。
	/// @note 当前主链路以 jointPosition 和 motorCurrent 为输入：
	/// 1. C API 将 jointPosition 从 deg 转为内部 rad；
	/// 2. coordinator 将 q/current/time 送入 robotStateObserver_；
	/// 3. robotStateObserver_ 估计 dq/ddq/motorEstimatedTorque；
	/// 4. coordinator 刷新 forceObserver_ 中的外力矩估计。
	if (!coordinatorReady(groupId))
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

#ifdef HIC_ENABLE_DEBUG_PRINT
	static int debug_count = 0;
	const bool should_debug_print = shouldDebugPrint(debug_count);
	if (should_debug_print)
	{
		const int configured_joint_count = g_hicCoordinator->getJointCount();
		const int debug_joint_count = clampDebugJointCount(configured_joint_count);
		std::fprintf(stderr,
			"[hic_update_state_estimates] group=%d t=%.6f inputJointCount=%d configuredJointCount=%d debugPrintCount=%d\n",
			groupId,
			input->currentTime,
			input->jointCount,
			configured_joint_count,
			debug_joint_count);
		std::fflush(stderr);
		for (int i = 0; i < debug_joint_count; ++i)
		{
			std::fprintf(stderr,
				"  joint[%d] q_deg=%.6f q_rad=%.6f current=%.6f\n",
				i,
				input->jointPosition[i],
				joint_position_rad[i],
				input->motorCurrent[i]);
		}
		std::fflush(stderr);
	}
#endif

	const int status = g_hicCoordinator->updateRobotState(
		joint_position_rad, input->motorCurrent, input->currentTime);

#ifdef HIC_ENABLE_DEBUG_PRINT
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[hic_update_state_estimates] updateRobotState returned status=%d\n",
			status);
		std::fflush(stderr);
		HicRobotState observedState = {};
		const int stateStatus = g_hicCoordinator->getRobotState(observedState);
		std::fprintf(stderr,
			"[hic_update_state_estimates] getRobotState status=%d motorEstimatedTorque:\n",
			stateStatus);
		const int debug_joint_count = clampDebugJointCount(g_hicCoordinator->getJointCount());
		for (int i = 0; i < debug_joint_count; ++i)
		{
			std::fprintf(stderr,
				"  tau_est[%d]=%.6f Nm filteredCurrent=%.6f A\n",
				i,
				observedState.motorEstimatedTorque[i],
				observedState.motorCurrent[i]);
		}
		std::fflush(stderr);
	}
#endif

	return status;
}

int hic_set_cartesian_impedance_gains(
	RTS_IEC_INT groupId,
	const HicImpedanceGains* gains)
{
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentJointPositionAsNullspaceTarget();
}

int hic_set_joint_impedance_config(
	RTS_IEC_INT groupId,
	const HicJointImpedanceConfig* config)
{
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!config)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicJointImpedanceConfig internalConfig = *config;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		internalConfig.targetPosition[i] = degToRad(config->targetPosition[i]);
	}
	return g_hicCoordinator->setJointImpedanceConfig(internalConfig);
}

int hic_capture_current_joint_position_as_impedance_target(RTS_IEC_INT groupId)
{
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentJointPositionAsImpedanceTarget();
}

int hic_set_cartesian_fixed_position_target(RTS_IEC_INT groupId, const double target_position[3])
{
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentPositionAsFixedTarget();
}

int hic_capture_current_pose_as_fixed_target(RTS_IEC_INT groupId)
{
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->captureCurrentPoseAsFixedTarget();
}

int hic_set_joint_torque_sensor_parameters(RTS_IEC_INT groupId, const HicTorqueSensorConfig* config)
{
	if (!coordinatorReady(groupId))
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
	/// @note 进入 forceObserver 后会完成标定、滤波和故障检查，
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}

#ifdef HIC_ENABLE_DEBUG_PRINT
	if (!raw_torque_by_hardware_channel)
	{
		std::fprintf(stderr,
			"[hic_update_raw_joint_torque_sensor] group=%d raw_torque=null\n",
			groupId);
	}
	else
	{
		static int debug_count = 0;
		if ((debug_count++ % 1000) == 0)
		{
			const int debug_joint_count = clampDebugJointCount(g_hicCoordinator->getJointCount());
			std::fprintf(stderr,
				"[hic_update_raw_joint_torque_sensor] group=%d jointCount=%d\n",
				groupId,
				debug_joint_count);
		std::fflush(stderr);
			for (int i = 0; i < debug_joint_count; ++i)
			{
				std::fprintf(stderr,
					"  raw_torque[%d]=%.6f\n",
					i,
					raw_torque_by_hardware_channel[i]);
			}
		}
	}
#endif

	return g_hicCoordinator->updateRawJointTorqueSensor(raw_torque_by_hardware_channel);
}

int hic_update_wrench_sensor_data(RTS_IEC_INT groupId, const HicWrenchSensorInput* input)
{
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	const int readyStatus = requireCoordinator(groupId);
	if (readyStatus != HIC_STATUS_OK)
	{
		return readyStatus;
	}
	return g_hicCoordinator->setGravityVector(gx, gy, gz);
}

int hic_set_state_filter_config(RTS_IEC_INT groupId, const HicStateFilterConfig* config)
{
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	const int readyStatus = requireCoordinator(groupId);
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
	if (!coordinatorReady(groupId))
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
	/// 1. robotStateObserver 内部保存的关节状态；
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->getLastStatus();
}

int hic_get_force_control_mode(RTS_IEC_INT groupId)
{
	if (!coordinatorReady(groupId))
	{
		return HIC_FORCE_CONTROL_MODE_NONE;
	}
	return static_cast<int>(g_hicCoordinator->getForceControlMode());
}

int hic_reset_control(RTS_IEC_INT groupId)
{
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return g_hicCoordinator->reset();
}

int hic_set_cartesian_fixed_pose_target_zyx_euler(
	RTS_IEC_INT groupId,
	const double target_pose_zyx_euler[HIC_CARTESIAN_DIM])
{
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	case HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE:
		return g_hicCoordinator->startForceControlJointImpedanceMode();
	case HIC_FORCE_CONTROL_MODE_NONE:
	default:
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}
}

int hic_prepare_stop_force_control_mode(RTS_IEC_INT groupId)
{
	if (!coordinatorReady(groupId))
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
	/// @brief 力控统一电流命令出口。
	/// @note 外部周期性调用该接口获取最终下发到电机电流环的目标电流。
	/// 内部链路为：C API -> computeForceControlCurrentCommand()
	/// -> computeForceControlTorqueCommand() -> 当前力控子模式的力矩计算函数
	/// -> convertTorqueToCurrent()。
	/// @note joint_protection_status 表示对应关节在力矩或电流限幅中被保护/截断。
	if (!motor_current_commands || !joint_protection_status)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (!coordinatorReady(groupId))
	{
		clearArray(motor_current_commands, HIC_MAX_JOINTS);
		clearArray(joint_protection_status, HIC_MAX_JOINTS);
		return HIC_STATUS_ERROR_INIT;
	}

	return g_hicCoordinator->computeForceControlCurrentCommand(
		motor_current_commands, joint_protection_status);
}

int hic_get_force_control_torque_commands(
	RTS_IEC_INT groupId,
	double joint_torque_commands[HIC_MAX_JOINTS],
	bool joint_protection_status[HIC_MAX_JOINTS])
{
	/// @brief 力控统一力矩命令出口。
	/// @note 外部可用该接口直接查看当前力控算法输出的关节侧目标力矩，单位 N.m。
	/// 与电流命令接口相比，本接口不会执行 torque -> current 换算。
	/// @note 在关节阻抗模式下，输出通常由阻抗力矩、重力补偿和可选科氏/离心补偿组成，
	/// 并且已经经过统一安全限幅。
	if (!joint_torque_commands || !joint_protection_status)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (!coordinatorReady(groupId))
	{
		clearArray(joint_torque_commands, HIC_MAX_JOINTS);
		clearArray(joint_protection_status, HIC_MAX_JOINTS);
		return HIC_STATUS_ERROR_INIT;
	}

	return g_hicCoordinator->computeForceControlTorqueCommand(
		joint_torque_commands, joint_protection_status);
}

int hic_get_robot_state(
	RTS_IEC_INT groupId,
	HicRobotState* state_out)
{
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
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
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	(void)enable;
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

int hic_enable_wrench_sensor_for_force_feedforward(RTS_IEC_INT groupId, bool enable)
{
	if (!coordinatorReady(groupId))
	{
		return HIC_STATUS_ERROR_INIT;
	}
	(void)enable;
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

} // extern "C"
