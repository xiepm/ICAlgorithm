#include "hic_controller/interface/hic_c_api.h"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kMToMm = 1000.0;

HicInitializeConfig makeDefaultInitializeConfig()
{
	HicInitializeConfig config = {};
	config.jointCount = 7;
	config.controlPeriod = 0.001;
	config.robotType = 20;

	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		config.kinematicDHParams[i] = 0.0;
	}

	config.kinematicDHParams[0] = 0.22;
	config.kinematicDHParams[1] = 0.42;
	config.kinematicDHParams[2] = 0.185;
	config.kinematicDHParams[3] = 0.38;

	return config;
}

HicControlConfig makeDefaultRuntimeConfig(const HicInitializeConfig& initConfig)
{
	HicControlConfig config = {};
	config.jointCount = initConfig.jointCount;
	config.controlPeriod = initConfig.controlPeriod;
	config.robotType = initConfig.robotType;
	config.enableGravityCompensation = true;
	config.enableCoriolisCompensation = false;
	config.enableTorqueRateLimit = true;
	config.enableCurrentLimit = true;

	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		config.kinematicParams[i] = initConfig.kinematicDHParams[i];
		config.torqueConstant[i] = 0.1;
		config.gearRatio[i] = 100.0;
		config.transmissionEfficiency[i] = 1.0;
		config.maxJointTorque[i] = 20.0;
		config.maxJointCurrent[i] = 8.0;
		config.maxTorqueRate[i] = 50.0;
		config.upperJointLimit[i] = 180.0;
		config.lowerJointLimit[i] = -180.0;
		config.upperJointVelocity[i] = 1.0 * kRadToDeg;
		config.lowerJointVelocity[i] = -1.0 * kRadToDeg;
		config.upperJointAcceleration[i] = 3.0 * kRadToDeg;
		config.lowerJointAcceleration[i] = -3.0 * kRadToDeg;
		config.upperJointTorque[i] = config.maxJointTorque[i];
		config.lowerJointTorque[i] = -config.maxJointTorque[i];
		config.upperMotorCurrent[i] = config.maxJointCurrent[i];
		config.lowerMotorCurrent[i] = -config.maxJointCurrent[i];
	}

	for (int i = 0; i < HIC_MAX_DYNAMIC_PARAMS; ++i)
	{
		config.dynamicParams[i] = 0.05;
	}

	return config;
}

HicTorqueSensorConfig makeTorqueSensorConfig(const HicInitializeConfig& config)
{
	HicTorqueSensorConfig torqueSensorConfig = {};
	torqueSensorConfig.jointCount = config.jointCount;
	std::snprintf(torqueSensorConfig.version, sizeof(torqueSensorConfig.version), "1.0");
	std::snprintf(torqueSensorConfig.unit, sizeof(torqueSensorConfig.unit), "N.m");
	std::snprintf(torqueSensorConfig.sensorLocation, sizeof(torqueSensorConfig.sensorLocation), "post_reducer");
	torqueSensorConfig.enableTorqueSensorFilter = true;
	torqueSensorConfig.enableExternalTorqueFilter = true;
	torqueSensorConfig.enableSaturationCheck = true;
	torqueSensorConfig.enableFaultCheck = true;

	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		torqueSensorConfig.joints[i].enabled = (i < config.jointCount);
		torqueSensorConfig.joints[i].jointIndex = i + 1;
		torqueSensorConfig.joints[i].hardwareChannel = i;
		torqueSensorConfig.joints[i].ratedCapacityNm = 120.0;
		torqueSensorConfig.joints[i].directionSign = 1;
		torqueSensorConfig.joints[i].zeroOffsetNm = 0.0;
		torqueSensorConfig.joints[i].scale = 1.0;
		torqueSensorConfig.joints[i].biasNm = 0.0;
		torqueSensorConfig.joints[i].maxValidTorqueNm = 120.0;
		torqueSensorConfig.torqueSensorFilterAlpha[i] = 0.2;
		torqueSensorConfig.externalTorqueFilterAlpha[i] = 0.2;
	}

	return torqueSensorConfig;
}

int applyDefaultRuntimeConfig(const HicControlConfig& config)
{
	int status = hic_set_dynamics_linear_parameters(0, config.dynamicParams);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_motor_torque_conversion_parameters(0, 
		config.torqueConstant, config.gearRatio, config.transmissionEfficiency);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	HicJointRangeLimits positionLimits = {};
	HicJointMaxLimits velocityLimits = {};
	HicJointMaxLimits accelerationLimits = {};
	HicJointRangeLimits torqueLimits = {};
	HicJointRangeLimits currentLimits = {};
	positionLimits.jointCount = config.jointCount;
	velocityLimits.jointCount = config.jointCount;
	accelerationLimits.jointCount = config.jointCount;
	torqueLimits.jointCount = config.jointCount;
	currentLimits.jointCount = config.jointCount;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		positionLimits.lower[i] = config.lowerJointLimit[i];
		positionLimits.upper[i] = config.upperJointLimit[i];
		velocityLimits.maxAbs[i] = std::max(std::fabs(config.lowerJointVelocity[i]), std::fabs(config.upperJointVelocity[i]));
		accelerationLimits.maxAbs[i] = std::max(std::fabs(config.lowerJointAcceleration[i]), std::fabs(config.upperJointAcceleration[i]));
		torqueLimits.lower[i] = config.lowerJointTorque[i];
		torqueLimits.upper[i] = config.upperJointTorque[i];
		currentLimits.lower[i] = config.lowerMotorCurrent[i];
		currentLimits.upper[i] = config.upperMotorCurrent[i];
	}
	status = hic_set_joint_position_limits(0, &positionLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_joint_velocity_limits(0, &velocityLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_joint_acceleration_limits(0, &accelerationLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_joint_torque_limits(0, &torqueLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	return hic_set_motor_current_limits(0, &currentLimits);
}

HicImpedanceGains makeDefaultGains()
{
	HicImpedanceGains gains = {};
	for (int i = 0; i < 3; ++i)
	{
		gains.stiffness[i] = 200.0;
		gains.damping[i] = 30.0;
	}
	for (int i = 3; i < HIC_CARTESIAN_DIM; ++i)
	{
		gains.stiffness[i] = 20.0;
		gains.damping[i] = 4.0;
	}
	return gains;
}

void printVector(const char* name, const double* data, int size)
{
	std::cout << name << ": ";
	for (int i = 0; i < size; ++i)
	{
		std::cout << data[i] << (i + 1 == size ? "" : ", ");
	}
	std::cout << std::endl;
}

void printBoolVector(const char* name, const bool* data, int size)
{
	std::cout << name << ": ";
	for (int i = 0; i < size; ++i)
	{
		std::cout << data[i] << (i + 1 == size ? "" : ", ");
	}
	std::cout << std::endl;
}

int updateState(
	const HicControlConfig& config,
	const double* jointPosition,
	const double* motorCurrent,
	double currentTime)
{
	HicStateEstimateInput input = {};
	input.jointCount = config.jointCount;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		input.jointPosition[i] = jointPosition ? (jointPosition[i] * kRadToDeg) : 0.0;
		input.motorCurrent[i] = motorCurrent ? motorCurrent[i] : 0.0;
	}
	input.currentTime = currentTime;
	return hic_update_state_estimates(0, &input);
}

int initializeDemoController(const HicInitializeConfig& initConfig, const HicControlConfig& runtimeConfig)
{
	int status = hic_initialize_control(0, &initConfig);
	std::cout << "init status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = applyDefaultRuntimeConfig(runtimeConfig);
	std::cout << "apply runtime config status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	HicTorqueSensorConfig torqueConfig = makeTorqueSensorConfig(initConfig);
	status = hic_set_joint_torque_sensor_parameters(0, &torqueConfig);
	std::cout << "set torque sensor config status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	double rawTorqueByHardwareChannel[HIC_MAX_JOINTS] = { 1.2, -0.8, 0.5, 0.1, -0.2, 0.3, -0.4 };
	status = hic_update_raw_joint_torque_sensor(0, rawTorqueByHardwareChannel);
	std::cout << "update raw torque sensor status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	HicImpedanceGains gains = makeDefaultGains();
	status = hic_set_cartesian_impedance_gains(0, &gains);
	std::cout << "set gains status: " << status << std::endl;
	return status;
}

int testFixedPointImpedance()
{
	std::cout << "\n=== Fixed Point Impedance ===" << std::endl;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig config = makeDefaultRuntimeConfig(initConfig);
	int status = initializeDemoController(initConfig, config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	double qCapture[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 0.15 };
	double motorCurrent[HIC_MAX_JOINTS] = { 0.05, 0.04, 0.03, 0.02, 0.01, 0.02, 0.03 };
	status = updateState(config, qCapture, motorCurrent, 0.0);
	std::cout << "capture state update status: " << status << std::endl;

	status = hic_capture_current_pose_as_fixed_target(0);
	std::cout << "capture current pose status: " << status << std::endl;

	double qDisturbed[HIC_MAX_JOINTS] = { 0.12, -0.18, 0.31, 0.22, -0.08, 0.11, 0.17 };
	status = updateState(config, qDisturbed, motorCurrent, 0.001);
	std::cout << "disturbed state update status: " << status << std::endl;

	status = hic_start_force_control_mode(0, HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
	std::cout << "start fixed impedance status: " << status << std::endl;

	double torqueCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };
	status = hic_get_force_control_torque_commands(0, torqueCmd, protection);
	std::cout << "torque command status: " << status << std::endl;
	printVector("torque command", torqueCmd, config.jointCount);
	printBoolVector("protection", protection, config.jointCount);

	HicCartesianState cartesianState = {};
	status = hic_get_current_cartesian_state(0, &cartesianState);
	std::cout << "get cartesian state status: " << status << std::endl;
	printVector("current pose", cartesianState.pose, HIC_POSE_DIM);
	printVector("current twist", cartesianState.twist, HIC_CARTESIAN_DIM);

	return hic_prepare_stop_force_control_mode(0);
}

int testFixedPositionWithNullspace()
{
	std::cout << "\n=== Fixed Position With Nullspace ===" << std::endl;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig config = makeDefaultRuntimeConfig(initConfig);
	int status = initializeDemoController(initConfig, config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	double qCapture[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 0.15 };
	double motorCurrent[HIC_MAX_JOINTS] = { 0.05, 0.04, 0.03, 0.02, 0.01, 0.02, 0.03 };
	status = updateState(config, qCapture, motorCurrent, 0.0);
	std::cout << "capture state update status: " << status << std::endl;

	HicNullspaceControlConfig nullspace = {};
	nullspace.enabled = true;
	for (int i = 0; i < config.jointCount; ++i)
	{
		nullspace.targetJointPosition[i] = qCapture[i];
		nullspace.stiffness[i] = 2.0;
		nullspace.damping[i] = 0.2;
	}
	status = hic_set_force_control_nullspace_config(0, &nullspace);
	std::cout << "set nullspace config status: " << status << std::endl;
	status = hic_capture_current_joint_position_as_nullspace_target(0);
	std::cout << "capture current joint as nullspace target status: " << status << std::endl;
	status = hic_capture_current_position_as_fixed_target(0);
	std::cout << "capture current position status: " << status << std::endl;

	double qDisturbed[HIC_MAX_JOINTS] = { 0.13, -0.17, 0.33, 0.18, -0.12, 0.13, 0.10 };
	status = updateState(config, qDisturbed, motorCurrent, 0.001);
	std::cout << "disturbed state update status: " << status << std::endl;
	status = hic_start_force_control_mode(0, HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION);
	std::cout << "start fixed position mode status: " << status << std::endl;

	double torqueCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };
	status = hic_get_force_control_torque_commands(0, torqueCmd, protection);
	std::cout << "fixed position torque status: " << status << std::endl;
	printVector("fixed position torque", torqueCmd, config.jointCount);
	printBoolVector("fixed position protection", protection, config.jointCount);

	return hic_prepare_stop_force_control_mode(0);
}

int testTrajectoryImpedance()
{
	std::cout << "\n=== Trajectory Impedance ===" << std::endl;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig config = makeDefaultRuntimeConfig(initConfig);
	int status = initializeDemoController(initConfig, config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = hic_start_force_control_mode(0, HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY);
	std::cout << "start trajectory impedance status: " << status << std::endl;

	double q[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 0.15 };
	double motorCurrent[HIC_MAX_JOINTS] = { 0.05, 0.04, 0.03, 0.02, 0.01, 0.02, 0.03 };
	double torqueCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };

	for (int step = 0; step < 5; ++step)
	{
		double targetPoseZyxEuler[HIC_CARTESIAN_DIM] = {
			(0.30 + 0.01 * step) * kMToMm,
			(-0.12 + 0.005 * step) * kMToMm,
			(0.42 + 0.002 * step) * kMToMm,
			0.0, 0.0, 0.0
		};
		double targetVelocity[HIC_CARTESIAN_DIM] = { 0.02 * kMToMm, 0.01 * kMToMm, 0.0, 0.0, 0.0, 0.0 };
		q[0] += 0.002;
		q[1] -= 0.001;

		status = updateState(config, q, motorCurrent, 0.001 * (step + 1));
		std::cout << "step " << step << " update state status: " << status << std::endl;
		status = hic_set_cartesian_trajectory_target_zyx_euler(0, targetPoseZyxEuler, targetVelocity);
		std::cout << "step " << step << " set online target status: " << status << std::endl;
		status = hic_get_force_control_torque_commands(0, torqueCmd, protection);
		std::cout << "step " << step << " torque status: " << status << std::endl;
		printVector("trajectory torque", torqueCmd, config.jointCount);
	}

	return hic_prepare_stop_force_control_mode(0);
}

int testZeroForceMode()
{
	std::cout << "\n=== Zero Force Mode ===" << std::endl;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig config = makeDefaultRuntimeConfig(initConfig);
	int status = initializeDemoController(initConfig, config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	double q0[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 0.15 };
	double q1[HIC_MAX_JOINTS] = { 0.101, -0.199, 0.2995, 0.2005, -0.0998, 0.1002, 0.1499 };
	double motorCurrent[HIC_MAX_JOINTS] = { 0.05, 0.04, 0.03, 0.02, 0.01, 0.02, 0.03 };

	status = updateState(config, q0, motorCurrent, 0.0);
	std::cout << "zero force state update #0 status: " << status << std::endl;
	status = updateState(config, q1, motorCurrent, 0.02);
	std::cout << "zero force state update #1 status: " << status << std::endl;

	status = hic_start_force_control_mode(0, HIC_FORCE_CONTROL_MODE_ZERO_FORCE);
	std::cout << "start zero force status: " << status << std::endl;

	double currentCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };
	status = hic_get_force_control_current_commands(0, currentCmd, protection);
	std::cout << "zero force current status: " << status << std::endl;
	printVector("zero force current", currentCmd, config.jointCount);
	printBoolVector("zero force protection", protection, config.jointCount);

	status = hic_prepare_stop_force_control_mode(0);
	std::cout << "prepare stop zero force status: " << status << std::endl;

	double qStop[HIC_MAX_JOINTS] = { 0.101, -0.199, 0.2995, 0.2005, -0.0998, 0.1002, 0.1499 };
	status = updateState(config, qStop, motorCurrent, 0.14);
	std::cout << "stop settle state update status: " << status << std::endl;
	status = hic_get_force_control_current_commands(0, currentCmd, protection);
	std::cout << "stop settle current status: " << status << std::endl;
	printVector("stop settle current", currentCmd, config.jointCount);
	printBoolVector("stop settle protection", protection, config.jointCount);

	return status;
}

int testZeroForceSoftBoundary()
{
	std::cout << "\n=== Zero Force Soft Boundary ===" << std::endl;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig config = makeDefaultRuntimeConfig(initConfig);
	int status = initializeDemoController(initConfig, config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	double q0[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 3.015 };
	double q1[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 3.018 };
	double motorCurrent[HIC_MAX_JOINTS] = { 0.05, 0.04, 0.03, 0.02, 0.01, 0.02, 0.03 };

	status = updateState(config, q0, motorCurrent, 0.0);
	std::cout << "soft boundary state update #0 status: " << status << std::endl;
	status = updateState(config, q1, motorCurrent, 0.02);
	std::cout << "soft boundary state update #1 status: " << status << std::endl;

	status = hic_start_force_control_mode(0, HIC_FORCE_CONTROL_MODE_ZERO_FORCE);
	std::cout << "start zero force soft boundary status: " << status << std::endl;

	double currentCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };
	status = hic_get_force_control_current_commands(0, currentCmd, protection);
	std::cout << "soft boundary zero force current status: " << status << std::endl;
	printVector("soft boundary zero force current", currentCmd, config.jointCount);
	printBoolVector("soft boundary zero force protection", protection, config.jointCount);

	return hic_prepare_stop_force_control_mode(0);
}

int testZeroForceRejectNearHardLimit()
{
	std::cout << "\n=== Zero Force Reject Near Hard Limit ===" << std::endl;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig config = makeDefaultRuntimeConfig(initConfig);
	int status = initializeDemoController(initConfig, config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	double qNearLimit[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 3.135 };
	double motorCurrent[HIC_MAX_JOINTS] = { 0.05, 0.04, 0.03, 0.02, 0.01, 0.02, 0.03 };
	status = updateState(config, qNearLimit, motorCurrent, 0.0);
	std::cout << "near limit state update status: " << status << std::endl;

	status = hic_start_force_control_mode(0, HIC_FORCE_CONTROL_MODE_ZERO_FORCE);
	std::cout << "start zero force near limit status: " << status << std::endl;
	return status;
}

}

int main()
{
	int status = testFixedPointImpedance();
	std::cout << "fixed point final status: " << status << std::endl;

	status = testFixedPositionWithNullspace();
	std::cout << "fixed position nullspace final status: " << status << std::endl;

	status = testTrajectoryImpedance();
	std::cout << "trajectory final status: " << status << std::endl;

	status = testZeroForceMode();
	std::cout << "zero force final status: " << status << std::endl;

	status = testZeroForceSoftBoundary();
	std::cout << "zero force soft boundary final status: " << status << std::endl;

	status = testZeroForceRejectNearHardLimit();
	std::cout << "zero force near limit final status: " << status << std::endl;

	return 0;
}
