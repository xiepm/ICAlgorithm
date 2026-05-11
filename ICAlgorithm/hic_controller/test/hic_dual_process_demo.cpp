#include "hic_controller/interface/hic_c_api.h"

#include <csignal>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

enum SimScenario
{
	SIM_SCENARIO_IDLE = 0,
	SIM_SCENARIO_DRAG = 1,
	SIM_SCENARIO_LIMIT = 2,
	SIM_SCENARIO_FAST = 3
};

struct SharedRobotFeed
{
	volatile std::uint64_t sequence;
	volatile int running;
	volatile int scenario;
	double jointPosition[HIC_MAX_JOINTS];
	double motorCurrent[HIC_MAX_JOINTS];
	double currentTime;
};

struct RobotFeedSnapshot
{
	double jointPosition[HIC_MAX_JOINTS];
	double motorCurrent[HIC_MAX_JOINTS];
	double currentTime;
	int scenario;
};

const char* scenarioName(int scenario)
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

HicInitializeConfig makeDefaultInitializeConfig()
{
	HicInitializeConfig config = {};
	config.jointCount = 7;
	config.controlPeriod = 0.02;
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

int applyDefaultRuntimeConfig(const HicControlConfig& config)
{
	int status = hic_set_dynamics_linear_parameters(config.dynamicParams);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_motor_torque_conversion_parameters(
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
	status = hic_set_joint_position_limits(&positionLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_joint_velocity_limits(&velocityLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_joint_acceleration_limits(&accelerationLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = hic_set_joint_torque_limits(&torqueLimits);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	return hic_set_motor_current_limits(&currentLimits);
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
		std::cout << std::fixed << std::setprecision(4) << data[i]
			  << (i + 1 == size ? "" : ", ");
	}
	std::cout << std::endl;
}

void printBoolVector(const char* name, const bool* data, int size)
{
	std::cout << name << ": ";
	for (int i = 0; i < size; ++i)
	{
		std::cout << (data[i] ? 1 : 0) << (i + 1 == size ? "" : ", ");
	}
	std::cout << std::endl;
}

void generateScenarioState(
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
		for (int i = 0; i < 7; ++i)
		{
			jointPosition[i] += 0.002 * std::sin(0.2 * timeSec + 0.1 * i);
		}
		break;
	case SIM_SCENARIO_DRAG:
		for (int i = 0; i < 7; ++i)
		{
			jointPosition[i] += 0.08 * std::sin(0.4 * timeSec + 0.2 * i);
		}
		break;
	case SIM_SCENARIO_LIMIT:
		jointPosition[6] = 3.02 + 0.02 * std::sin(0.25 * timeSec);
		break;
	case SIM_SCENARIO_FAST:
		for (int i = 0; i < 7; ++i)
		{
			jointPosition[i] += 0.18 * std::sin(2.8 * timeSec + 0.3 * i);
		}
		break;
	default:
		break;
	}
}

void runStateGenerator(SharedRobotFeed* shared)
{
	const double dt = 0.02;
	double t = 0.0;

	while (shared->running)
	{
		double jointPosition[HIC_MAX_JOINTS] = { 0.0 };
		double motorCurrent[HIC_MAX_JOINTS] = { 0.0 };
		generateScenarioState(shared->scenario, t, jointPosition, motorCurrent);

		++shared->sequence;
		for (int i = 0; i < HIC_MAX_JOINTS; ++i)
		{
			shared->jointPosition[i] = jointPosition[i];
			shared->motorCurrent[i] = motorCurrent[i];
		}
		shared->currentTime = t;
		++shared->sequence;

		t += dt;
		usleep(static_cast<useconds_t>(dt * 1.0e6));
	}
	::_exit(0);
}

RobotFeedSnapshot snapshotSharedState(const SharedRobotFeed* shared)
{
	RobotFeedSnapshot snapshot = {};
	for (;;)
	{
		const std::uint64_t begin = shared->sequence;
		if (begin & 1ULL)
		{
			continue;
		}
		for (int i = 0; i < HIC_MAX_JOINTS; ++i)
		{
			snapshot.jointPosition[i] = shared->jointPosition[i];
			snapshot.motorCurrent[i] = shared->motorCurrent[i];
		}
		snapshot.currentTime = shared->currentTime;
		snapshot.scenario = shared->scenario;
		const std::uint64_t end = shared->sequence;
		if (begin == end && ((end & 1ULL) == 0))
		{
			return snapshot;
		}
	}
}

int initializeController()
{
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig runtimeConfig = makeDefaultRuntimeConfig(initConfig);
	int status = hic_initialize_control(&initConfig);
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
	status = hic_set_joint_torque_sensor_parameters(&torqueConfig);
	std::cout << "set torque sensor config status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	HicImpedanceGains gains = makeDefaultGains();
	status = hic_set_cartesian_impedance_gains(&gains);
	std::cout << "set impedance gains status: " << status << std::endl;
	return status;
}

int syncControllerState(const RobotFeedSnapshot& snapshot)
{
	HicStateEstimateInput input = {};
	input.jointCount = 7;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		input.jointPosition[i] = snapshot.jointPosition[i] * kRadToDeg;
		input.motorCurrent[i] = snapshot.motorCurrent[i];
	}
	input.currentTime = snapshot.currentTime;
	return hic_update_state_estimates(&input);
}

void printObservedState()
{
	HicRobotState robotState = {};
	const int robotStateStatus = hic_get_robot_state(&robotState);
	std::cout << "robot state status: " << robotStateStatus << std::endl;
	if (robotStateStatus == HIC_STATUS_OK)
	{
		printVector("jointPosition", robotState.jointPosition, 7);
		printVector("jointVelocity", robotState.jointVelocity, 7);
		printVector("jointAcceleration", robotState.jointAcceleration, 7);
		printVector("motorCurrent", robotState.motorCurrent, 7);
		printVector("motorEstimatedTorque", robotState.motorEstimatedTorque, 7);
	}

	HicActiveControlState activeState = {};
	const int activeStateStatus = hic_get_active_control_state(&activeState);
	if (activeStateStatus == HIC_STATUS_OK &&
	    activeState.forceControlMode != HIC_FORCE_CONTROL_MODE_NONE)
	{
		double jointTorque[HIC_MAX_JOINTS] = { 0.0 };
		bool protection[HIC_MAX_JOINTS] = { false };
		const int torqueStatus = hic_get_force_control_torque_commands(jointTorque, protection);
		std::cout << "force control torque status: " << torqueStatus << std::endl;
		if (torqueStatus == HIC_STATUS_OK || torqueStatus == HIC_STATUS_ERROR_CURRENT_LIMIT)
		{
			printVector("jointTorqueCommand", jointTorque, 7);
			printBoolVector("jointTorqueProtectionStatus", protection, 7);
		}
	}
	else
	{
		std::cout << "force control torque status: no active force control mode" << std::endl;
	}
}

void printSharedSnapshot(const RobotFeedSnapshot& snapshot)
{
	std::cout << "shared scenario: " << scenarioName(snapshot.scenario)
		  << ", time: " << std::fixed << std::setprecision(3) << snapshot.currentTime
		  << std::endl;
	printVector("shared jointPosition", snapshot.jointPosition, 7);
	printVector("shared motorCurrent", snapshot.motorCurrent, 7);
}

void printModeOutput()
{
	const int mode = hic_get_force_control_mode();
	std::cout << "force control mode: " << mode << ", last status: " << hic_get_last_status() << std::endl;

	double motorCurrentCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };
	int outputStatus = HIC_STATUS_OK;

	HicActiveControlState activeState = {};
	const int activeStateStatus = hic_get_active_control_state(&activeState);
	if (activeStateStatus == HIC_STATUS_OK &&
	    mode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE &&
	    activeState.forceControlMode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		outputStatus = hic_get_force_control_current_commands(motorCurrentCmd, protection);
		std::cout << "zero force current status: " << outputStatus << std::endl;
	}
	else if (activeStateStatus == HIC_STATUS_OK &&
		 mode != HIC_FORCE_CONTROL_MODE_NONE &&
		 activeState.forceControlMode != HIC_FORCE_CONTROL_MODE_NONE)
	{
		outputStatus = hic_get_force_control_current_commands(motorCurrentCmd, protection);
		std::cout << "impedance current status: " << outputStatus << std::endl;
	}
	else
	{
		std::cout << "controller is idle, no active output command" << std::endl;
	}

	if (mode != HIC_CONTROL_MODE_IDLE)
	{
		printVector("motorCurrentCommand", motorCurrentCmd, 7);
		printBoolVector("jointProtectionStatus", protection, 7);
	}

	printObservedState();

	HicCartesianState cartesianState = {};
	const int cartesianStatus = hic_get_current_cartesian_state(&cartesianState);
	std::cout << "cartesian state status: " << cartesianStatus << std::endl;
	if (cartesianStatus == HIC_STATUS_OK)
	{
		printVector("currentPose", cartesianState.pose, HIC_POSE_DIM);
		printVector("currentTwist", cartesianState.twist, HIC_CARTESIAN_DIM);
	}
}

void printHelp()
{
	std::cout
		<< "commands:\n"
		<< "  help                     show this help\n"
		<< "  scenario idle|drag|limit|fast\n"
		<< "                           switch child process state generation pattern\n"
		<< "  sync                     read one shared state and feed hic_update_state_estimates\n"
		<< "  zero                     sync and start zero force mode\n"
		<< "  fixed                    sync, capture current pose, start fixed impedance mode\n"
		<< "  step                     sync once and print controller outputs\n"
		<< "  run N                    repeat step N times\n"
		<< "  stop                     request stop for current mode\n"
		<< "  state                    print current shared state + observed robot state\n"
		<< "  reset                    call hic_reset_control\n"
		<< "  quit                     exit demo\n";
}

int scenarioFromString(const std::string& name)
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
}

int main()
{
	SharedRobotFeed* shared = static_cast<SharedRobotFeed*>(
		mmap(nullptr, sizeof(SharedRobotFeed), PROT_READ | PROT_WRITE,
		     MAP_SHARED | MAP_ANONYMOUS, -1, 0));
	if (shared == MAP_FAILED)
	{
		std::perror("mmap");
		return 1;
	}
	std::memset(shared, 0, sizeof(*shared));
	shared->running = 1;
	shared->scenario = SIM_SCENARIO_IDLE;

	const pid_t childPid = fork();
	if (childPid < 0)
	{
		std::perror("fork");
		munmap(shared, sizeof(*shared));
		return 1;
	}
	if (childPid == 0)
	{
		runStateGenerator(shared);
	}

	const int initStatus = initializeController();
	if (initStatus != HIC_STATUS_OK)
	{
		shared->running = 0;
		waitpid(childPid, nullptr, 0);
		munmap(shared, sizeof(*shared));
		return initStatus;
	}

	printHelp();

	std::string line;
	while (true)
	{
		std::cout << "\nhic-dual> " << std::flush;
		if (!std::getline(std::cin, line))
		{
			break;
		}

		std::istringstream iss(line);
		std::string command;
		iss >> command;
		if (command.empty())
		{
			continue;
		}
		if (command == "help")
		{
			printHelp();
			continue;
		}
		if (command == "scenario")
		{
			std::string name;
			iss >> name;
			const int scenario = scenarioFromString(name);
			if (scenario < 0)
			{
				std::cout << "unknown scenario: " << name << std::endl;
				continue;
			}
			shared->scenario = scenario;
			std::cout << "scenario switched to " << scenarioName(scenario) << std::endl;
			continue;
		}

		RobotFeedSnapshot snapshot = snapshotSharedState(shared);

		if (command == "sync")
		{
			const int status = syncControllerState(snapshot);
			std::cout << "sync status: " << status << std::endl;
			continue;
		}
		if (command == "zero")
		{
			int status = syncControllerState(snapshot);
			std::cout << "sync status: " << status << std::endl;
			if (status == HIC_STATUS_OK)
			{
				status = hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_ZERO_FORCE);
				std::cout << "start zero force status: " << status << std::endl;
			}
			printModeOutput();
			continue;
		}
		if (command == "fixed")
		{
			int status = syncControllerState(snapshot);
			std::cout << "sync status: " << status << std::endl;
			if (status == HIC_STATUS_OK)
			{
				status = hic_capture_current_pose_as_fixed_target();
				std::cout << "capture fixed pose target status: " << status << std::endl;
			}
			if (status == HIC_STATUS_OK)
			{
				status = hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
				std::cout << "start fixed pose mode status: " << status << std::endl;
			}
			printModeOutput();
			continue;
		}
		if (command == "step")
		{
			const int status = syncControllerState(snapshot);
			std::cout << "sync status: " << status << std::endl;
			printModeOutput();
			continue;
		}
		if (command == "run")
		{
			int steps = 1;
			iss >> steps;
			if (steps <= 0)
			{
				steps = 1;
			}
			for (int i = 0; i < steps; ++i)
			{
				RobotFeedSnapshot loopSnapshot = snapshotSharedState(shared);
				const int status = syncControllerState(loopSnapshot);
				std::cout << "\nrun step " << i << " sync status: " << status << std::endl;
				printModeOutput();
				usleep(100000);
			}
			continue;
		}
		if (command == "stop")
		{
			const int mode = hic_get_force_control_mode();
			int status = HIC_STATUS_OK;
			HicActiveControlState activeState = {};
			const int activeStateStatus = hic_get_active_control_state(&activeState);
			if (activeStateStatus == HIC_STATUS_OK &&
			    mode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE &&
			    activeState.forceControlMode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
			{
				status = hic_prepare_stop_force_control_mode();
				std::cout << "prepare stop zero force status: " << status << std::endl;
			}
			else if (activeStateStatus == HIC_STATUS_OK &&
				 mode != HIC_FORCE_CONTROL_MODE_NONE &&
				 activeState.forceControlMode != HIC_FORCE_CONTROL_MODE_NONE)
			{
				status = hic_prepare_stop_force_control_mode();
				std::cout << "prepare stop impedance status: " << status << std::endl;
			}
			else
			{
				std::cout << "controller already idle" << std::endl;
			}
			continue;
		}
		if (command == "state")
		{
			printSharedSnapshot(snapshot);
			printObservedState();
			continue;
		}
		if (command == "reset")
		{
			const int status = hic_reset_control();
			std::cout << "reset status: " << status << std::endl;
			continue;
		}
		if (command == "quit" || command == "exit")
		{
			break;
		}

		std::cout << "unknown command: " << command << std::endl;
	}

	shared->running = 0;
	waitpid(childPid, nullptr, 0);
	munmap(shared, sizeof(*shared));
	return 0;
}
