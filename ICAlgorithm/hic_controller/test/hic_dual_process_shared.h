#pragma once

#include "hic_controller/interface/hic_c_api.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

namespace hic_test
{

static constexpr double kPi = 3.14159265358979323846;
static constexpr double kRadToDeg = 180.0 / kPi;

static constexpr const char* kStateSocketHost = "127.0.0.1";
static constexpr int kStateSocketPort = 34567;
static constexpr int kDemoJointCount = 7;
static constexpr double kControlPeriodSec = 0.02;

enum SimScenario
{
	SIM_SCENARIO_IDLE = 0,
	SIM_SCENARIO_DRAG = 1,
	SIM_SCENARIO_LIMIT = 2,
	SIM_SCENARIO_FAST = 3
};

enum ControlCommandType
{
	CONTROL_COMMAND_SET_SCENARIO = 1,
	CONTROL_COMMAND_SHUTDOWN = 2
};

struct StatePacket
{
	std::uint64_t sequence;
	int scenario;
	double currentTime;
	double jointPosition[HIC_MAX_JOINTS];
	double motorCurrent[HIC_MAX_JOINTS];
};

struct ControlPacket
{
	int type;
	int scenario;
};

inline const char* scenarioName(int scenario)
{
	switch (scenario)
	{
	case SIM_SCENARIO_IDLE:
		return "idle";
	case SIM_SCENARIO_DRAG:
		return "drag";
	case SIM_SCENARIO_LIMIT:
		return "limit";
	case SIM_SCENARIO_FAST:
		return "fast";
	default:
		return "unknown";
	}
}

inline int scenarioFromString(const std::string& name)
{
	if (name == "idle")
	{
		return SIM_SCENARIO_IDLE;
	}
	if (name == "drag")
	{
		return SIM_SCENARIO_DRAG;
	}
	if (name == "limit")
	{
		return SIM_SCENARIO_LIMIT;
	}
	if (name == "fast")
	{
		return SIM_SCENARIO_FAST;
	}
	return -1;
}

inline void generateScenarioState(
	int scenario,
	double timeSec,
	double* jointPosition,
	double* motorCurrent)
{
	static const double kBasePose[HIC_MAX_JOINTS] = { 0.10, -0.20, 0.30, 0.20, -0.10, 0.10, 0.15 };
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		jointPosition[i] = kBasePose[i];
		motorCurrent[i] = 0.02 * std::sin(timeSec + 0.3 * i);
	}

	switch (scenario)
	{
	case SIM_SCENARIO_IDLE:
		for (int i = 0; i < kDemoJointCount; ++i)
		{
			jointPosition[i] += 0.002 * std::sin(0.2 * timeSec + 0.1 * i);
		}
		break;
	case SIM_SCENARIO_DRAG:
		for (int i = 0; i < kDemoJointCount; ++i)
		{
			jointPosition[i] += 0.08 * std::sin(0.4 * timeSec + 0.2 * i);
		}
		break;
	case SIM_SCENARIO_LIMIT:
		jointPosition[6] = 3.02 + 0.02 * std::sin(0.25 * timeSec);
		break;
	case SIM_SCENARIO_FAST:
		for (int i = 0; i < kDemoJointCount; ++i)
		{
			jointPosition[i] += 0.18 * std::sin(2.8 * timeSec + 0.3 * i);
		}
		break;
	default:
		break;
	}
}

inline HicInitializeConfig makeDefaultInitializeConfig()
{
	HicInitializeConfig config = {};
	config.jointCount = kDemoJointCount;
	config.controlPeriod = kControlPeriodSec;
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

inline HicControlConfig makeDefaultRuntimeConfig(const HicInitializeConfig& initConfig)
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

inline HicTorqueSensorConfig makeTorqueSensorConfig(const HicInitializeConfig& config)
{
	HicTorqueSensorConfig torqueSensorConfig = {};
	std::snprintf(torqueSensorConfig.version, sizeof(torqueSensorConfig.version), "1.0");
	std::snprintf(torqueSensorConfig.unit, sizeof(torqueSensorConfig.unit), "N.m");
	std::snprintf(torqueSensorConfig.sensorLocation, sizeof(torqueSensorConfig.sensorLocation), "post_reducer");
	torqueSensorConfig.jointCount = config.jointCount;
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

inline int applyDefaultRuntimeConfig(const HicControlConfig& config)
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

inline HicImpedanceGains makeDefaultGains()
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

} // namespace hic_test
