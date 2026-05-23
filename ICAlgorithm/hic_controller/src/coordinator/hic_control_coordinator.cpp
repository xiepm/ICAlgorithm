#include "hic_controller/coordinator/hic_control_coordinator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace hic
{
namespace
{
// ============================================================
// 1. 这些函数只在本文件内部使用；
// ============================================================

int getExpectedJointCountFromRobotType(int robotType)
{
	switch (robotType)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 5:
	case 7:
	case 8:
	case 9:
	case 10:
	case 12:
		return 6;
	case 20:
		return 7;
	default:
		return -1;
	}
}

double defaultLargeLimit()
{
	return 1.0e12;
}

double maxAbsLimit(double lower, double upper)
{
	return std::max(std::fabs(lower), std::fabs(upper));
}

bool hasExplicitRange(double lower, double upper)
{
	return lower != 0.0 || upper != 0.0;
}

double configuredPositiveAbsLimit(double lower, double upper)
{
	if (!hasExplicitRange(lower, upper))
	{
		return 0.0;
	}
	return std::max(std::fabs(lower), std::fabs(upper));
}

double clampToConfiguredRange(double value, double lower, double upper, double symmetricAbs)
{
	if (hasExplicitRange(lower, upper))
	{
		if (lower > upper)
		{
			return 0.0;
		}
		return std::max(lower, std::min(upper, value));
	}
	if (symmetricAbs > 0.0)
	{
		return std::max(-symmetricAbs, std::min(symmetricAbs, value));
	}
	return value;
}

void fillRangeFromBoundsOrSymmetric(
	double lowerBound,
	double upperBound,
	double symmetricAbs,
	double* lowerOut,
	double* upperOut,
	double* maxAbsOut)
{
	if (!lowerOut || !upperOut || !maxAbsOut)
	{
		return;
	}

	if (hasExplicitRange(lowerBound, upperBound))
	{
		*lowerOut = lowerBound;
		*upperOut = upperBound;
		*maxAbsOut = maxAbsLimit(lowerBound, upperBound);
		return;
	}

	if (symmetricAbs > 0.0)
	{
		*lowerOut = -symmetricAbs;
		*upperOut = symmetricAbs;
		*maxAbsOut = symmetricAbs;
		return;
	}

	*lowerOut = -defaultLargeLimit();
	*upperOut = defaultLargeLimit();
	*maxAbsOut = defaultLargeLimit();
}

double defaultZeroForceDampingValue(int jointIndex)
{
	static const double kDefaultDamping[7] = { 1.5, 1.5, 1.2, 0.8, 0.5, 0.3, 0.2 };
	if (jointIndex >= 0 && jointIndex < 7)
	{
		return kDefaultDamping[jointIndex];
	}
	return 0.3;
}

double computeZeroForceEntryVelocityLimit(const HicControlConfig& config, int jointIndex)
{
	const double configuredLimit = configuredPositiveAbsLimit(
		config.lowerJointVelocity[jointIndex],
		config.upperJointVelocity[jointIndex]);
	if (configuredLimit > 0.0)
	{
		return std::max(0.05, std::min(0.3, 0.2 * configuredLimit));
	}
	return 0.2;
}

double computeZeroForceExitVelocityThreshold(const HicControlConfig& config, int jointIndex)
{
	return std::min(0.05, 0.5 * computeZeroForceEntryVelocityLimit(config, jointIndex));
}

bool hasValidJointLimitWindow(const HicControlConfig& config, int jointIndex)
{
	return hasExplicitRange(config.lowerJointLimit[jointIndex], config.upperJointLimit[jointIndex]) &&
		config.lowerJointLimit[jointIndex] < config.upperJointLimit[jointIndex];
}

double computeHardLimitMargin(const HicControlConfig& config, int jointIndex)
{
	if (!hasValidJointLimitWindow(config, jointIndex))
	{
		return 0.0;
	}
	const double range = config.upperJointLimit[jointIndex] - config.lowerJointLimit[jointIndex];
	return std::max(0.01, std::min(0.03, 0.05 * range));
}

double computeSoftLimitMargin(const HicControlConfig& config, int jointIndex)
{
	if (!hasValidJointLimitWindow(config, jointIndex))
	{
		return 0.0;
	}
	const double range = config.upperJointLimit[jointIndex] - config.lowerJointLimit[jointIndex];
	return std::max(0.08, std::min(0.2, 0.15 * range));
}

double computeSoftLimitDampingScale(double distanceToLimit, double hardMargin, double softMargin)
{
	if (softMargin <= hardMargin || distanceToLimit >= softMargin)
	{
		return 0.0;
	}
	if (distanceToLimit <= hardMargin)
	{
		return 1.0;
	}
	return (softMargin - distanceToLimit) / (softMargin - hardMargin);
}
}

// ============================================================
// 1. 生命周期与初始化
// ============================================================

std::shared_ptr<HicControlCoordinator> HicControlCoordinator::create(
	const HicControlConfig& config)
{
	std::shared_ptr<HicControlCoordinator> coordinator =
		std::make_shared<HicControlCoordinator>();
	if (!coordinator)
	{
		return nullptr;
	}

	if (coordinator->initialize(config) != HIC_STATUS_OK)
	{
		return nullptr;
	}

	return coordinator;
}

HicControlCoordinator::HicControlCoordinator()
	: initialized_(false),
	  controlMode_(HIC_CONTROL_MODE_IDLE),
	  forceControlMode_(HIC_FORCE_CONTROL_MODE_NONE),
	  currentTime_(0.0),
	  lastStatus_(HIC_STATUS_OK),
	  commandInputVersion_(0),
	  lastComputedVersion_(0),
	  commandCacheValid_(false),
	  zeroForceStopRequested_(false)
{
	std::memset(&config_, 0, sizeof(config_));
	std::memset(&nullspaceConfig_, 0, sizeof(nullspaceConfig_));
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
	std::fill(previousJointTorqueCommand_, previousJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousMotorCurrentCommand_, previousMotorCurrentCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointTorqueCommand_, lastJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointProtectionStatus_, lastJointProtectionStatus_ + HIC_MAX_JOINTS, false);
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		zeroForceDamping_[i] = defaultZeroForceDampingValue(i);
		zeroForceExitDamping_[i] = 3.0 * zeroForceDamping_[i];
	}
	std::fill(lastCurrentPose_, lastCurrentPose_ + HIC_POSE_DIM, 0.0);
	std::fill(lastCurrentTwist_, lastCurrentTwist_ + HIC_CARTESIAN_DIM, 0.0);
	std::memset(&nullspaceConfig_, 0, sizeof(nullspaceConfig_));
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
}

HicControlCoordinator::~HicControlCoordinator()
{
}

HicStatus HicControlCoordinator::initialize(const HicControlConfig& config)
{
#ifdef HIC_ENABLE_DEBUG_PRINT
	std::fprintf(stderr,
		"[HicControlCoordinator::initialize] input jointCount=%d controlPeriod=%.9f robotType=%d\n",
		config.jointCount,
		config.controlPeriod,
		config.robotType);
#endif
	// Comment split from executable statement.
	if (config.jointCount <= 0 || config.jointCount > HIC_MAX_JOINTS || config.controlPeriod <= 0.0)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] invalid base config: jointCount=%d range=(1,%d), controlPeriod=%.9f, status=%d\n",
			config.jointCount,
			HIC_MAX_JOINTS,
			config.controlPeriod,
			static_cast<int>(lastStatus_));
#endif
		return lastStatus_;
	}
	const int expectedJointCount = getExpectedJointCountFromRobotType(config.robotType);
	if (expectedJointCount > 0 && config.jointCount != expectedJointCount)
	{
#ifdef HIC_ALLOW_DEBUG_JOINT_COUNT_MISMATCH
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] WARNING debug bypass robotType/jointCount mismatch: robotType=%d expectedJointCount=%d actualJointCount=%d\n",
			config.robotType,
			expectedJointCount,
			config.jointCount);
#endif
#else
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] robotType/jointCount mismatch: robotType=%d expectedJointCount=%d actualJointCount=%d status=%d\n",
			config.robotType,
			expectedJointCount,
			config.jointCount,
			static_cast<int>(lastStatus_));
#endif
		return lastStatus_;
#endif
	}

	config_ = config;
	currentTime_ = 0.0;
	controlMode_ = HIC_CONTROL_MODE_IDLE;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
	std::fill(previousJointTorqueCommand_, previousJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousMotorCurrentCommand_, previousMotorCurrentCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointTorqueCommand_, lastJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointProtectionStatus_, lastJointProtectionStatus_ + HIC_MAX_JOINTS, false);
	std::fill(lastCurrentPose_, lastCurrentPose_ + HIC_POSE_DIM, 0.0);
	std::fill(lastCurrentTwist_, lastCurrentTwist_ + HIC_CARTESIAN_DIM, 0.0);
	lastStatus_ = HIC_STATUS_OK;
	commandInputVersion_ = 0;
	lastComputedVersion_ = 0;
	commandCacheValid_ = false;
	zeroForceStopRequested_ = false;

	HicStatus status = kinematicsAdapter_.initialize(config.robotType, config.jointCount, config.kinematicParams);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] kinematicsAdapter.initialize failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	status = dynamicsAdapter_.initialize(config.robotType, config.jointCount, config.dynamicParams);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] dynamicsAdapter.initialize failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	status = dynamicsAdapter_.setRobotKinematicParameters(config.kinematicParams);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] dynamicsAdapter.setRobotKinematicParameters failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	status = impedanceCore_.initialize(config.jointCount, config.controlPeriod);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] impedanceCore.initialize failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	status = jointImpedanceCore_.initialize(config.jointCount, config.controlPeriod);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] jointImpedanceCore.initialize failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}
	jointImpedanceCore_.reset();
	status = jointImpedanceCore_.setConfig(jointImpedanceConfig_);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] jointImpedanceCore.setConfig failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	HicRobotStateObserverConfig stateConfig = {};
	buildDefaultStateObserverConfig(&stateConfig);
	status = robotStateObserver_.initialize(stateConfig);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] robotStateObserver.initialize failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	status = forceObserver_.initialize(config.jointCount, config.controlPeriod);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
#ifdef HIC_ENABLE_DEBUG_PRINT
		std::fprintf(stderr,
			"[HicControlCoordinator::initialize] forceObserver.initialize failed status=%d\n",
			static_cast<int>(status));
#endif
		return status;
	}

	initialized_ = true;
	lastStatus_ = HIC_STATUS_OK;
#ifdef HIC_ENABLE_DEBUG_PRINT
	std::fprintf(stderr,
		"[HicControlCoordinator::initialize] success jointCount=%d controlPeriod=%.9f robotType=%d\n",
		config_.jointCount,
		config_.controlPeriod,
		config_.robotType);
#endif
	return lastStatus_;
}

// ============================================================
// 2. 配置构造与同步
// ============================================================

void HicControlCoordinator::buildDefaultStateObserverConfig(
	HicRobotStateObserverConfig* configOut) const
{
	if (!configOut)
	{
		return;
	}

	HicRobotStateObserverConfig& stateConfig = *configOut;
	stateConfig = {};
	stateConfig.jointCount = config_.jointCount;
	stateConfig.controlPeriod = config_.controlPeriod;
	stateConfig.enablePositionFilter = false;
	stateConfig.enableVelocityFilter = false;
	stateConfig.enableAccelerationFilter = false;
	stateConfig.enableMotorCurrentFilter = false;
	stateConfig.enableMeasuredTorqueFilter = false;
	stateConfig.enableCurrentToTorqueEstimate = true;

	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		stateConfig.positionFilterAlpha[i] = 1.0;
		stateConfig.velocityFilterAlpha[i] = 1.0;
		stateConfig.accelerationFilterAlpha[i] = 1.0;
		stateConfig.motorCurrentFilterAlpha[i] = 1.0;
		stateConfig.measuredTorqueFilterAlpha[i] = 1.0;

		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointLimit[i],
			config_.upperJointLimit[i],
			0.0,
			&stateConfig.lowerJointPosition[i],
			&stateConfig.upperJointPosition[i],
			&stateConfig.maxAbsJointPosition[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointVelocity[i],
			config_.upperJointVelocity[i],
			0.0,
			&stateConfig.lowerJointVelocity[i],
			&stateConfig.upperJointVelocity[i],
			&stateConfig.maxAbsJointVelocity[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointAcceleration[i],
			config_.upperJointAcceleration[i],
			0.0,
			&stateConfig.lowerJointAcceleration[i],
			&stateConfig.upperJointAcceleration[i],
			&stateConfig.maxAbsJointAcceleration[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxJointCurrent[i],
			&stateConfig.lowerMotorCurrent[i],
			&stateConfig.upperMotorCurrent[i],
			&stateConfig.maxAbsMotorCurrent[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointTorque[i],
			config_.upperJointTorque[i],
			config_.maxJointTorque[i],
			&stateConfig.lowerMeasuredTorque[i],
			&stateConfig.upperMeasuredTorque[i],
			&stateConfig.maxAbsMeasuredTorque[i]);

		stateConfig.torqueConstant[i] = config_.torqueConstant[i];
		stateConfig.gearRatio[i] = config_.gearRatio[i];
		stateConfig.transmissionEfficiency[i] = config_.transmissionEfficiency[i];
	}
}

HicStatus HicControlCoordinator::updateExternalTorqueEstimateFromRobotState()
{
	// 该函数把“电流反推外力矩”链路接到交互力观测器：
	// 1. robotStateObserver_ 提供滤波后的 q/dq 和由电机电流估算的关节力矩；
	// 2. dynamicsAdapter_ 计算当前状态下的模型力矩；
	// Comment split from executable statement.
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	double q[HIC_MAX_JOINTS] = { 0.0 };
	double dq[HIC_MAX_JOINTS] = { 0.0 };
	double motorEstimatedTorque[HIC_MAX_JOINTS] = { 0.0 };
	// Step 1: read filtered state from the state observer.
	HicStatus status = robotStateObserver_.getFilteredJointPosition(q);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = robotStateObserver_.getFilteredJointVelocity(dq);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = robotStateObserver_.getMotorEstimatedTorque(motorEstimatedTorque);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	// Step 2: compute model torques to subtract from motor-estimated torque.
	double gravityTorque[HIC_MAX_JOINTS] = { 0.0 };
	double coriolisTorque[HIC_MAX_JOINTS] = { 0.0 };
	double frictionTorque[HIC_MAX_JOINTS] = { 0.0 };

	status = dynamicsAdapter_.computeGravityTorque(q, gravityTorque);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = dynamicsAdapter_.computeCoriolisTorque(q, dq, coriolisTorque);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_NOT_IMPLEMENTED)
	{
		return status;
	}

	status = dynamicsAdapter_.computeFrictionTorque(dq, frictionTorque);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_NOT_IMPLEMENTED)
	{
		return status;
	}
	// Step 3: external torque estimate = motor estimate - model torque.
	double jointExternalTorque[HIC_MAX_JOINTS] = { 0.0 };
	for (int i = 0; i < config_.jointCount; ++i)
	{
		const double modelTorque = gravityTorque[i] + coriolisTorque[i] + frictionTorque[i];
		jointExternalTorque[i] = motorEstimatedTorque[i] - modelTorque;
	}

	// Comment split from executable statement.

	return forceObserver_.updateJointExternalTorque(jointExternalTorque);
}

HicStatus HicControlCoordinator::rebuildStateObserverConfig()
{
	// Comment split from executable statement.
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	HicRobotStateObserverConfig stateConfig = {};
	lastStatus_ = robotStateObserver_.getConfig(stateConfig);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}

	stateConfig.jointCount = config_.jointCount;
	stateConfig.controlPeriod = config_.controlPeriod;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointLimit[i],
			config_.upperJointLimit[i],
			0.0,
			&stateConfig.lowerJointPosition[i],
			&stateConfig.upperJointPosition[i],
			&stateConfig.maxAbsJointPosition[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxJointCurrent[i],
			&stateConfig.lowerMotorCurrent[i],
			&stateConfig.upperMotorCurrent[i],
			&stateConfig.maxAbsMotorCurrent[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointTorque[i],
			config_.upperJointTorque[i],
			config_.maxJointTorque[i],
			&stateConfig.lowerMeasuredTorque[i],
			&stateConfig.upperMeasuredTorque[i],
			&stateConfig.maxAbsMeasuredTorque[i]);
		stateConfig.torqueConstant[i] = config_.torqueConstant[i];
		stateConfig.gearRatio[i] = config_.gearRatio[i];
		stateConfig.transmissionEfficiency[i] = config_.transmissionEfficiency[i];
	}
	lastStatus_ = robotStateObserver_.setConfig(stateConfig);
	return lastStatus_;
}

// ============================================================
// ============================================================

HicStatus HicControlCoordinator::updateRobotState(
	const double* jointPosition,
	const double* motorCurrent,
	double currentTime)
{
#ifdef HIC_ENABLE_DEBUG_PRINT
	static int debug_update_state_count = 0;
	const bool should_debug_print = ((debug_update_state_count++ % 1000) == 0);
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicControlCoordinator::updateRobotState] entered, initialized=%d jointCount=%d t=%.6f\n",
			initialized_ ? 1 : 0,
			config_.jointCount,
			currentTime);
	}
#endif
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicControlCoordinator::updateRobotState] return status=%d before observer\n",
				static_cast<int>(lastStatus_));
		}
#endif
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.updateRobotState(
		jointPosition, motorCurrent, currentTime);
#ifdef HIC_ENABLE_DEBUG_PRINT
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicControlCoordinator::updateRobotState] robotStateObserver.updateRobotState status=%d\n",
			static_cast<int>(status));
	}
#endif
	if (status == HIC_STATUS_OK)
	{
		// Refresh external torque estimate after a successful state update.
		const HicStatus externalTorqueStatus = updateExternalTorqueEstimateFromRobotState();
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicControlCoordinator::updateRobotState] updateExternalTorqueEstimateFromRobotState status=%d\n",
				static_cast<int>(externalTorqueStatus));
		}
#endif
		if (externalTorqueStatus != HIC_STATUS_OK)
		{
			lastStatus_ = externalTorqueStatus;
			return lastStatus_;
		}

		// Comment split from executable statement.

		currentTime_ = currentTime;
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::updateJointState(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.updateJointState(
		jointPosition, jointVelocity, jointAcceleration);
	if (status == HIC_STATUS_OK)
	{
		currentTime_ += config_.controlPeriod;
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::updateJointMeasuredTorque(const double* jointMeasuredTorque)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.updateJointMeasuredTorque(jointMeasuredTorque);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

// ============================================================
// 4. 模式管理
// 2. 零力示教、定点阻抗、轨迹阻抗都属于 FORCE_CONTROL 内部子模式；
// ============================================================

HicStatus HicControlCoordinator::setControlMode(HicControlMode mode)
{
	controlMode_ = mode;
	if (mode != HIC_CONTROL_MODE_FORCE_CONTROL)
	{
		forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
		zeroForceStopRequested_ = false;
	}
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlZeroForceMode()
{
	// =========================
	// 零力示教模式入口
	// =========================
	//
	// 可以把它理解成零力示教的“准入门禁”：
	// 1. coordinator 本身必须已经初始化；
	// 4. 当前位置不能已经逼近硬限位；
	//
	// ZeroForceMode 的入口尽量保守：
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	HicRobotState state = {};
	const HicStatus status = validateZeroForceEntry(&state);
	if (status != HIC_STATUS_OK)
	{
		//
			// 这里不保留任何“半进入”痕迹，避免上层误以为当前已经进入零力模式，
			controlMode_ = HIC_CONTROL_MODE_IDLE;
			forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
			zeroForceStopRequested_ = false;
			lastStatus_ = status;
			return lastStatus_;
		}

	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_ZERO_FORCE;
	zeroForceStopRequested_ = false;
	// 一旦模式切换成功，后续输出语义就变了：
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::prepareStopForceControlMode()
{
	// =========================
	// =========================
	//
	// 从而造成不够平滑的行为。当前第一版采用一个更保守的策略：
	// 2. 后续控制周期继续输出“带更强阻尼的零力命令”；
	// 真正退出发生在 computeZeroForceCurrentCommand() 里，
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL)
	{
		lastStatus_ = HIC_STATUS_OK;
		return lastStatus_;
	}

	if (forceControlMode_ != HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		controlMode_ = HIC_CONTROL_MODE_IDLE;
		forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
		invalidateCommandCache();
		lastStatus_ = HIC_STATUS_OK;
		return lastStatus_;
	}

	zeroForceStopRequested_ = true;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlCartesianFixedPositionMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	lastStatus_ = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}
	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlCartesianFixedPoseMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	lastStatus_ = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}
	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlCartesianTrajectoryMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	lastStatus_ = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}
	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlJointImpedanceMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

// ============================================================
// 1. 这部分负责阻抗参数、动力学参数、安全边界和传感器配置；
// ============================================================

HicStatus HicControlCoordinator::setCartesianImpedanceGains(const HicImpedanceGains& gains)
{
	const HicStatus status = impedanceCore_.setGains(gains);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setNullspaceConfig(const HicNullspaceControlConfig& config)
{
	nullspaceConfig_ = config;
	const HicStatus status = impedanceCore_.setNullspaceConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentJointPositionAsNullspaceTarget()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	HicRobotState state = {};
	HicStatus status = robotStateObserver_.getRobotState(state);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.captureCurrentJointPositionAsNullspaceTarget(state.jointPosition);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setJointImpedanceConfig(const HicJointImpedanceConfig& config)
{
	jointImpedanceConfig_ = config;
	const HicStatus status = jointImpedanceCore_.setConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentJointPositionAsImpedanceTarget()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	double jointPosition[HIC_MAX_JOINTS] = { 0.0 };
	HicStatus status = robotStateObserver_.getFilteredJointPosition(jointPosition);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = jointImpedanceCore_.captureTargetPosition(jointPosition);
	if (status == HIC_STATUS_OK)
	{
		for (int i = 0; i < config_.jointCount; ++i)
		{
			jointImpedanceConfig_.targetPosition[i] = jointPosition[i];
		}
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setFixedTargetPosition(const double targetPosition[3])
{
	HicStatus status = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.setFixedTargetPosition(targetPosition);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setFixedTargetPose(const double targetPose[HIC_POSE_DIM])
{
	HicStatus status = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.setFixedTargetPose(targetPose);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentPositionAsFixedTarget()
{
	double currentPose[HIC_POSE_DIM] = { 0.0 };
	double currentTwist[HIC_CARTESIAN_DIM] = { 0.0 };
	HicStatus status = getCurrentCartesianState(currentPose, currentTwist);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = setFixedTargetPosition(currentPose);
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentPoseAsFixedTarget()
{
	double currentPose[HIC_POSE_DIM] = { 0.0 };
	double currentTwist[HIC_CARTESIAN_DIM] = { 0.0 };
	HicStatus status = getCurrentCartesianState(currentPose, currentTwist);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = setFixedTargetPose(currentPose);
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setOnlineTargetPose(
	const double targetPose[HIC_POSE_DIM],
	const double targetVelocity[HIC_CARTESIAN_DIM])
{
	HicStatus status = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.setOnlineTargetPose(targetPose, targetVelocity);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setDynamicsLinearParameters(const double* dynamicParams)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!dynamicParams)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	std::memcpy(config_.dynamicParams, dynamicParams, sizeof(config_.dynamicParams));
	lastStatus_ = dynamicsAdapter_.setDynamicParameters(config_.dynamicParams);
	if (lastStatus_ == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setPayloadMassProperties(
	double mass,
	const double centerOfMass[3])
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!centerOfMass || mass < 0.0)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	lastStatus_ = dynamicsAdapter_.setPayloadMassProperties(mass, centerOfMass);
	if (lastStatus_ == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setGravityVector(double gx, double gy, double gz)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	lastStatus_ = dynamicsAdapter_.setGravityVector(gx, gy, gz);
	if (lastStatus_ == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setMotorTorqueConversionParameters(
	const double* torqueConstant,
	const double* gearRatio,
	const double* transmissionEfficiency)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!torqueConstant || !gearRatio || !transmissionEfficiency)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		double kt = torqueConstant[i];
		double ratio = gearRatio[i];
		double efficiency = transmissionEfficiency[i];
#ifdef HIC_ALLOW_DEBUG_PARAM_VALIDATION_BYPASS
		if (!std::isfinite(kt) || kt <= 0.0)
		{
#ifdef HIC_ENABLE_DEBUG_PRINT
			std::fprintf(stderr,
				"[HicControlCoordinator::setMotorTorqueConversionParameters] WARNING debug bypass torqueConstant joint=%d input=%.6f use=1.0\n",
				i,
				torqueConstant[i]);
#endif
			kt = 1.0;
		}
		if (!std::isfinite(ratio) || ratio <= 0.0)
		{
#ifdef HIC_ENABLE_DEBUG_PRINT
			std::fprintf(stderr,
				"[HicControlCoordinator::setMotorTorqueConversionParameters] WARNING debug bypass gearRatio joint=%d input=%.6f use=1.0\n",
				i,
				gearRatio[i]);
#endif
			ratio = 1.0;
		}
		if (!std::isfinite(efficiency) || efficiency <= 0.0)
		{
#ifdef HIC_ENABLE_DEBUG_PRINT
			std::fprintf(stderr,
				"[HicControlCoordinator::setMotorTorqueConversionParameters] WARNING debug bypass transmissionEfficiency joint=%d input=%.6f use=1.0\n",
				i,
				transmissionEfficiency[i]);
#endif
			efficiency = 1.0;
		}
#else
		if (!std::isfinite(kt) || !std::isfinite(ratio) || !std::isfinite(efficiency) ||
			kt <= 0.0 || ratio <= 0.0 || efficiency <= 0.0)
		{
			lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
			return lastStatus_;
		}
#endif
		config_.torqueConstant[i] = kt;
		config_.gearRatio[i] = ratio;
		config_.transmissionEfficiency[i] = efficiency;
	}
	lastStatus_ = rebuildStateObserverConfig();
	if (lastStatus_ == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointPositionLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointLimit[i] = lowerLimits[i];
		config_.upperJointLimit[i] = upperLimits[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointVelocityLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointVelocity[i] = lowerLimits[i];
		config_.upperJointVelocity[i] = upperLimits[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointAccelerationLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointAcceleration[i] = lowerLimits[i];
		config_.upperJointAcceleration[i] = upperLimits[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointTorqueLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointTorque[i] = lowerLimits[i];
		config_.upperJointTorque[i] = upperLimits[i];
		config_.maxJointTorque[i] = maxAbsLimit(lowerLimits[i], upperLimits[i]);
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setMotorCurrentLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	constexpr double kDebugDefaultMotorCurrentLimit = 1000.0;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		double lower = lowerLimits[i];
		double upper = upperLimits[i];

#ifdef HIC_ALLOW_DEBUG_PARAM_VALIDATION_BYPASS
		const bool invalidLimit =
			!std::isfinite(lower) ||
			!std::isfinite(upper) ||
			lower > upper ||
			(lower == 0.0 && upper == 0.0);
		if (invalidLimit)
		{
#ifdef HIC_ENABLE_DEBUG_PRINT
			std::fprintf(stderr,
				"[HicControlCoordinator::setMotorCurrentLimits] WARNING debug bypass invalid current limit joint=%d input=[%.6f, %.6f], use=[%.6f, %.6f]\n",
				i,
				lowerLimits[i],
				upperLimits[i],
				-kDebugDefaultMotorCurrentLimit,
				kDebugDefaultMotorCurrentLimit);
#endif
			lower = -kDebugDefaultMotorCurrentLimit;
			upper = kDebugDefaultMotorCurrentLimit;
		}
#else
		if (!std::isfinite(lower) || !std::isfinite(upper) || lower > upper)
		{
			lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
			return lastStatus_;
		}
#endif

		config_.lowerMotorCurrent[i] = lower;
		config_.upperMotorCurrent[i] = upper;
		config_.maxJointCurrent[i] = maxAbsLimit(lower, upper);
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}
HicStatus HicControlCoordinator::updateJointExternalTorque(const double* jointExternalTorque)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = forceObserver_.updateJointExternalTorque(jointExternalTorque);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setRobotStateObserverConfig(const HicRobotStateObserverConfig& config)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.setConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::getRobotStateObserverConfig(HicRobotStateObserverConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return robotStateObserver_.getConfig(configOut);
}

HicStatus HicControlCoordinator::setTorqueSensorConfig(const HicTorqueSensorConfig& config)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = forceObserver_.setTorqueSensorConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::getTorqueSensorConfig(HicTorqueSensorConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return forceObserver_.getTorqueSensorConfig(configOut);
}

HicStatus HicControlCoordinator::loadTorqueSensorConfigFromFile(const char* filePath)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	lastStatus_ = forceObserver_.loadTorqueSensorConfigFromFile(filePath);
	return lastStatus_;
}

HicStatus HicControlCoordinator::updateRawJointTorqueSensor(const double* rawTorqueByHardwareChannel)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = forceObserver_.updateRawJointTorqueSensor(rawTorqueByHardwareChannel);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::getCalibratedJointTorqueSensor(double* calibratedJointTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return forceObserver_.getCalibratedJointTorqueSensor(calibratedJointTorque);
}

HicStatus HicControlCoordinator::getTorqueSensorFaultStatus(bool* faultStatus) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return forceObserver_.getTorqueSensorFaultStatus(faultStatus);
}

// ============================================================
// ============================================================

HicStatus HicControlCoordinator::computeZeroForceTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// ==============================
	// ==============================
	// 1. 调试零力控制律本身；
	// 2. 仿真或支持直接力矩驱动的执行器；
	//
	// 2. 重力补偿
	// 3. 速度阻尼
	// 5. 力矩安全限幅
	//
	// 它不做：
	// - externalWrench 解释
	// - targetPose 跟踪
	// - 复杂摩擦补偿
	//
	// 因此，这里实现的是“安全优先的关节层零力示教”，
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL ||
	    forceControlMode_ != HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	HicRobotState state = {};
	HicStatus status = validateZeroForceEntry(&state);
	if (status != HIC_STATUS_OK)
	{
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	if (zeroForceStopRequested_ && isZeroForceExitReady(state.jointVelocity))
	{
		// 当前处理策略是：
		// 3. 使缓存失效；
		controlMode_ = HIC_CONTROL_MODE_IDLE;
		forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
		zeroForceStopRequested_ = false;
		invalidateCommandCache();
		lastStatus_ = HIC_STATUS_OK;
		return lastStatus_;
	}

	if (config_.enableGravityCompensation)
	{
		// 零力示教的第一主项是重力补偿：
		// 目标不是“把关节力矩做成 0”，而是尽量抵消机器人自身重力造成的静态负担，
		status = dynamicsAdapter_.computeGravityTorque(state.jointPosition, jointTorqueCommand);
		if (status != HIC_STATUS_OK)
		{
			clearTorqueCommand(jointTorqueCommand);
			lastStatus_ = status;
			return lastStatus_;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		const double damping = zeroForceStopRequested_ ? zeroForceExitDamping_[i] : zeroForceDamping_[i];
		// jointTorqueCommand += -d * dq
		//
		// 它的作用不是“把机器人拉回某个位置”，
		//
		jointTorqueCommand[i] += -damping * state.jointVelocity[i];
	}

	status = applyZeroForceSoftBoundaryDamping(
		state.jointPosition,
		state.jointVelocity,
		jointTorqueCommand,
		jointProtectionStatus,
		zeroForceStopRequested_);
	if (status == HIC_STATUS_ERROR_JOINT_LIMIT)
	{
		// 一旦已经进入硬限位保护区，就不再尝试继续输出“修正型命令”，
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	const HicStatus safetyStatus = applySafetyLimits(
		state.jointPosition, jointTorqueCommand, jointProtectionStatus);
	if (safetyStatus != HIC_STATUS_OK && safetyStatus != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = safetyStatus;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		lastJointTorqueCommand_[i] = jointTorqueCommand[i];
		lastJointProtectionStatus_[i] = jointProtectionStatus[i];
	}

	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	lastStatus_ = (safetyStatus == HIC_STATUS_OK) ? status : safetyStatus;
	return lastStatus_;
}

HicStatus HicControlCoordinator::computeZeroForceCurrentCommand(
	double* motorCurrentCommand,
	bool* jointProtectionStatus)
{
	// ==============================
	// ==============================
	// 这里不重新实现一套零力逻辑，而是复用上面的力矩环主链路：
	// 2. 再把关节力矩换算成电机电流；
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!motorCurrentCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearCurrentCommand(motorCurrentCommand);
	clearProtectionStatus(jointProtectionStatus);

	double jointTorqueCommand[HIC_MAX_JOINTS] = { 0.0 };
	HicStatus status = computeZeroForceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	const HicStatus convertStatus = convertTorqueToCurrent(jointTorqueCommand, motorCurrentCommand);
	if (convertStatus != HIC_STATUS_OK)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = convertStatus;
		return lastStatus_;
	}

	HicStatus currentStatus = status;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		// convertTorqueToCurrent() 已经做过一轮电流限幅；
		const double limitedCurrent = clampToConfiguredRange(
			motorCurrentCommand[i],
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxJointCurrent[i]);
		if (limitedCurrent != motorCurrentCommand[i])
		{
			motorCurrentCommand[i] = limitedCurrent;
			jointProtectionStatus[i] = true;
			currentStatus = HIC_STATUS_ERROR_CURRENT_LIMIT;
		}
		previousMotorCurrentCommand_[i] = motorCurrentCommand[i];
	}

	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	lastStatus_ = currentStatus;
	return lastStatus_;
}

HicStatus HicControlCoordinator::validateZeroForceEntry(HicRobotState* stateOut) const
{
	// 这个函数同时服务于“进入时校验”和“运行时持续校验”：
	if (!stateOut)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	const HicStatus stateStatus = robotStateObserver_.getRobotState(*stateOut);
	if (stateStatus != HIC_STATUS_OK || !robotStateObserver_.isStateValid())
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}
	if (hasNaNOrInf(stateOut->jointPosition, config_.jointCount) ||
		hasNaNOrInf(stateOut->jointVelocity, config_.jointCount) ||
		hasNaNOrInf(stateOut->motorCurrent, config_.jointCount))
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (std::fabs(stateOut->jointVelocity[i]) > computeZeroForceEntryVelocityLimit(config_, i))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}

		if (hasValidJointLimitWindow(config_, i))
		{
			const double lower = config_.lowerJointLimit[i];
			const double upper = config_.upperJointLimit[i];
			const double hardMargin = computeHardLimitMargin(config_, i);
			if (stateOut->jointPosition[i] <= lower + hardMargin ||
				stateOut->jointPosition[i] >= upper - hardMargin)
			{
				return HIC_STATUS_ERROR_JOINT_LIMIT;
			}
		}

		const double torqueConstant = config_.torqueConstant[i];
		const double gearRatio = config_.gearRatio[i];
		const double transmissionEfficiency = config_.transmissionEfficiency[i];
		if (!std::isfinite(torqueConstant) || !std::isfinite(gearRatio) ||
			!std::isfinite(transmissionEfficiency) || torqueConstant <= 0.0 ||
			gearRatio <= 0.0 || transmissionEfficiency <= 0.0)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
	}

	return HIC_STATUS_OK;
}

HicStatus HicControlCoordinator::applyZeroForceSoftBoundaryDamping(
	const double* jointPosition,
	const double* jointVelocity,
	double* jointTorqueCommand,
	bool* jointProtectionStatus,
	bool exitDampingActive) const
{
	if (!jointPosition || !jointVelocity || !jointTorqueCommand || !jointProtectionStatus)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicStatus status = HIC_STATUS_OK;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (!hasValidJointLimitWindow(config_, i))
		{
			continue;
		}

		const double lower = config_.lowerJointLimit[i];
		const double upper = config_.upperJointLimit[i];
		const double hardMargin = computeHardLimitMargin(config_, i);
		const double softMargin = std::max(computeSoftLimitMargin(config_, i), hardMargin);
		const double q = jointPosition[i];
		const double dq = jointVelocity[i];

		if (q <= lower + hardMargin || q >= upper - hardMargin)
		{
			jointProtectionStatus[i] = true;
			jointTorqueCommand[i] = 0.0;
			status = HIC_STATUS_ERROR_JOINT_LIMIT;
			continue;
		}

		const double baseDamping = exitDampingActive ? zeroForceExitDamping_[i] : zeroForceDamping_[i];
		const double boundaryDamping = std::max(2.0 * baseDamping, baseDamping + 1.0);
		const double lowerDistance = q - lower;
		const double upperDistance = upper - q;

		if (dq < 0.0)
		{
			const double scale = computeSoftLimitDampingScale(lowerDistance, hardMargin, softMargin);
			if (scale > 0.0)
			{
				jointTorqueCommand[i] += -boundaryDamping * (1.0 + 3.0 * scale) * dq;
			}
		}
		if (dq > 0.0)
		{
			const double scale = computeSoftLimitDampingScale(upperDistance, hardMargin, softMargin);
			if (scale > 0.0)
			{
				jointTorqueCommand[i] += -boundaryDamping * (1.0 + 3.0 * scale) * dq;
			}
		}
	}

	return status;
}

bool HicControlCoordinator::isZeroForceExitReady(const double* jointVelocity) const
{
	if (!jointVelocity)
	{
		return false;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (std::fabs(jointVelocity[i]) > computeZeroForceExitVelocityThreshold(config_, i))
		{
			return false;
		}
	}
	return true;
}

// ============================================================
// 1. 支持输出关节力矩和电机电流两种形式；
// ============================================================

HicStatus HicControlCoordinator::computeCartesianImpedanceTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL ||
	    (forceControlMode_ != HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION &&
	     forceControlMode_ != HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE &&
	     forceControlMode_ != HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY))
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	if (commandCacheValid_ && lastComputedVersion_ == commandInputVersion_)
	{
		for (int i = 0; i < config_.jointCount; ++i)
		{
			jointTorqueCommand[i] = lastJointTorqueCommand_[i];
			jointProtectionStatus[i] = lastJointProtectionStatus_[i];
		}
		return lastStatus_;
	}

	const HicStatus status = runCartesianImpedanceStep(jointTorqueCommand, jointProtectionStatus);
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::computeCartesianImpedanceCurrentCommand(
	double* motorCurrentCommand,
	bool* jointProtectionStatus)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!motorCurrentCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	double jointTorqueCommand[HIC_MAX_JOINTS] = { 0.0 };
	HicStatus status = computeCartesianImpedanceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = status;
		return status;
	}

	const HicStatus convertStatus = convertTorqueToCurrent(jointTorqueCommand, motorCurrentCommand);
	if (convertStatus != HIC_STATUS_OK)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = convertStatus;
		return convertStatus;
	}

	lastStatus_ = status;
	return status;
}

// ============================================================
// 8. 状态读取、复位与调试
// ============================================================

HicStatus HicControlCoordinator::computeForceControlTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// Comment split from executable statement.
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	switch (forceControlMode_)
	{
	case HIC_FORCE_CONTROL_MODE_ZERO_FORCE:
		return computeZeroForceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION:
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE:
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY:
		return computeCartesianImpedanceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	case HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE:
		return computeJointImpedanceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	case HIC_FORCE_CONTROL_MODE_NONE:
	default:
		clearTorqueCommand(jointTorqueCommand);
		clearProtectionStatus(jointProtectionStatus);
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}
}

HicStatus HicControlCoordinator::computeForceControlCurrentCommand(
	double* motorCurrentCommand,
	bool* jointProtectionStatus)
{
	// hic_get_force_control_current_commands()
	//   -> computeForceControlCurrentCommand()
	//   -> computeForceControlTorqueCommand()
	//   -> computeJointImpedanceTorqueCommand()
	//   -> convertTorqueToCurrent()
	if (!initialized_)
	{
		// Comment split from executable statement.
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!motorCurrentCommand || !jointProtectionStatus)
	{
		// Comment split from executable statement.
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	// Comment split from executable statement.

	clearCurrentCommand(motorCurrentCommand);
	clearProtectionStatus(jointProtectionStatus);

	double jointTorqueCommand[HIC_MAX_JOINTS] = { 0.0 };
	// Step 1: compute torque command for the active force-control sub-mode.
	HicStatus status = computeForceControlTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		// Comment split from executable statement.
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = status;
		return lastStatus_;
	}
	// Step 2: convert joint torque command to motor current command.
	const HicStatus convertStatus = convertTorqueToCurrent(jointTorqueCommand, motorCurrentCommand);
	if (convertStatus != HIC_STATUS_OK)
	{
		// Comment split from executable statement.
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = convertStatus;
		return lastStatus_;
	}

	// Comment split from executable statement.

	lastStatus_ = status;
	return lastStatus_;
}

HicStatus HicControlCoordinator::getCurrentCartesianState(
	double* currentPose,
	double* currentTwist)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!currentPose || !currentTwist)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicRobotState state = {};
	HicStatus status = robotStateObserver_.getRobotState(state);
	if (status != HIC_STATUS_OK || !robotStateObserver_.isStateValid())
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	status = kinematicsAdapter_.computeForwardKinematics(state.jointPosition, currentPose);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	return kinematicsAdapter_.computeEndEffectorTwist(
		state.jointPosition, state.jointVelocity, currentTwist);
}

HicControlMode HicControlCoordinator::getControlMode() const
{
	return controlMode_;
}

HicForceControlMode HicControlCoordinator::getForceControlMode() const
{
	return forceControlMode_;
}

bool HicControlCoordinator::isNullspaceEnabled() const
{
	return nullspaceConfig_.enabled;
}

int HicControlCoordinator::getJointCount() const
{
	return config_.jointCount;
}

HicStatus HicControlCoordinator::getRobotState(HicRobotState& stateOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return robotStateObserver_.getRobotState(stateOut);
}

HicStatus HicControlCoordinator::getConfigSnapshot(HicControlConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	configOut = config_;
	return HIC_STATUS_OK;
}

HicStatus HicControlCoordinator::getLastCartesianWrenchCommand(double wrenchCommand[HIC_CARTESIAN_DIM]) const
{
	return impedanceCore_.getLastCartesianWrenchCommand(wrenchCommand);
}

HicStatus HicControlCoordinator::getLastJointTorqueCommand(double* jointTorqueCommand) const
{
	if (!jointTorqueCommand)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] = lastJointTorqueCommand_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicControlCoordinator::getLastStatus() const
{
	return lastStatus_;
}

HicStatus HicControlCoordinator::reset()
{
	controlMode_ = HIC_CONTROL_MODE_IDLE;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
	currentTime_ = 0.0;
	std::fill(previousJointTorqueCommand_, previousJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousMotorCurrentCommand_, previousMotorCurrentCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointTorqueCommand_, lastJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointProtectionStatus_, lastJointProtectionStatus_ + HIC_MAX_JOINTS, false);
	std::fill(lastCurrentPose_, lastCurrentPose_ + HIC_POSE_DIM, 0.0);
	std::fill(lastCurrentTwist_, lastCurrentTwist_ + HIC_CARTESIAN_DIM, 0.0);
	std::memset(&nullspaceConfig_, 0, sizeof(nullspaceConfig_));
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
	robotStateObserver_.reset();
	forceObserver_.reset();
	impedanceCore_.reset();
	jointImpedanceCore_.reset();
	commandInputVersion_ = 0;
	lastComputedVersion_ = 0;
	commandCacheValid_ = false;
	zeroForceStopRequested_ = false;
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

// ============================================================
// 9. 笛卡尔阻抗内部求解与通用安全工具
// 1. 这一组函数是控制核心的公共底座；
// ============================================================

HicStatus HicControlCoordinator::runCartesianImpedanceStep(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// 该函数是阻抗模式的一次完整控制周期：
	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	double currentPose[HIC_POSE_DIM] = { 0.0 };
	double currentTwist[HIC_CARTESIAN_DIM] = { 0.0 };
	double gravityTorque[HIC_MAX_JOINTS] = { 0.0 };
	double jacobianRowMajor[HIC_MAX_JACOBIAN_SIZE] = { 0.0 };
	HicRobotState state = {};

	HicStatus status = robotStateObserver_.getRobotState(state);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	if (!robotStateObserver_.isStateValid())
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	status = kinematicsAdapter_.computeForwardKinematics(state.jointPosition, currentPose);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = kinematicsAdapter_.computeJacobian(state.jointPosition, jacobianRowMajor);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = kinematicsAdapter_.computeEndEffectorTwist(state.jointPosition, state.jointVelocity, currentTwist);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	if (config_.enableGravityCompensation)
	{
		status = dynamicsAdapter_.computeGravityTorque(state.jointPosition, gravityTorque);
		if (status != HIC_STATUS_OK)
		{
			return status;
		}
	}

	status = impedanceCore_.computeJointTorque(
		currentPose,
		currentTwist,
		state.jointPosition,
		state.jointVelocity,
		jacobianRowMajor,
		jointTorqueCommand);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] += gravityTorque[i];
	}

	status = applySafetyLimits(state.jointPosition, jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearTorqueCommand(jointTorqueCommand);
		return status;
	}

	for (int i = 0; i < HIC_POSE_DIM; ++i)
	{
		lastCurrentPose_[i] = currentPose[i];
	}
	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		lastCurrentTwist_[i] = currentTwist[i];
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		lastJointTorqueCommand_[i] = jointTorqueCommand[i];
		lastJointProtectionStatus_[i] = jointProtectionStatus[i];
	}
	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	return status == HIC_STATUS_OK ? HIC_STATUS_OK : status;
}

HicStatus HicControlCoordinator::computeJointImpedanceTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// Comment split from executable statement.
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL ||
	    forceControlMode_ != HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	if (commandCacheValid_ && lastComputedVersion_ == commandInputVersion_)
	{
		for (int i = 0; i < config_.jointCount; ++i)
		{
			jointTorqueCommand[i] = lastJointTorqueCommand_[i];
			jointProtectionStatus[i] = lastJointProtectionStatus_[i];
		}
		return lastStatus_;
	}

	double q[HIC_MAX_JOINTS] = { 0.0 };
	double dq[HIC_MAX_JOINTS] = { 0.0 };
	// Step 1: read filtered state from the state observer.
	HicStatus status = robotStateObserver_.getFilteredJointPosition(q);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}
	status = robotStateObserver_.getFilteredJointVelocity(dq);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}
	if (!robotStateObserver_.isStateValid())
	{
		lastStatus_ = HIC_STATUS_ERROR_ROBOT_STATE;
		return lastStatus_;
	}

	double tauExt[HIC_MAX_JOINTS] = { 0.0 };
	const double* tauExtPtr = nullptr;
	if (jointImpedanceConfig_.enableExternalTorqueCompensation)
	{
		// Comment split from executable statement.
		status = forceObserver_.getFilteredJointExternalTorque(tauExt);
		if (status != HIC_STATUS_OK)
		{
			lastStatus_ = status;
			return lastStatus_;
		}
		tauExtPtr = tauExt;
	}

	double tauImp[HIC_MAX_JOINTS] = { 0.0 };
	// Comment split from executable statement.
	status = jointImpedanceCore_.computeJointTorque(q, dq, tauExtPtr, tauImp);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}

	double tauGravity[HIC_MAX_JOINTS] = { 0.0 };
	double tauCoriolis[HIC_MAX_JOINTS] = { 0.0 };
	// Comment split from executable statement.
	status = dynamicsAdapter_.computeGravityTorque(q, tauGravity);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}
	if (config_.enableCoriolisCompensation)
	{
		status = dynamicsAdapter_.computeCoriolisTorque(q, dq, tauCoriolis);
		if (status != HIC_STATUS_OK)
		{
			lastStatus_ = status;
			return lastStatus_;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] = tauImp[i] + tauGravity[i] + tauCoriolis[i];
	}

	// Comment split from executable statement.

	status = applySafetyLimits(q, jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		lastJointTorqueCommand_[i] = jointTorqueCommand[i];
		lastJointProtectionStatus_[i] = jointProtectionStatus[i];
	}
	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	lastStatus_ = status;
	return lastStatus_;
}

HicStatus HicControlCoordinator::applySafetyLimits(
	const double* jointPosition,
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	if (!jointPosition || !jointTorqueCommand || !jointProtectionStatus)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (hasNaNOrInf(jointTorqueCommand, config_.jointCount))
	{
		std::fill(jointTorqueCommand, jointTorqueCommand + config_.jointCount, 0.0);
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	HicStatus status = HIC_STATUS_OK;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		const bool jointLimitEnabled =
			(config_.upperJointLimit[i] != 0.0 || config_.lowerJointLimit[i] != 0.0);
		if (jointLimitEnabled &&
			(jointPosition[i] > config_.upperJointLimit[i] || jointPosition[i] < config_.lowerJointLimit[i]))
		{
			jointProtectionStatus[i] = true;
			jointTorqueCommand[i] = 0.0;
			status = HIC_STATUS_ERROR_JOINT_LIMIT;
			continue;
		}

		if (config_.enableTorqueRateLimit && config_.maxTorqueRate[i] > 0.0)
		{
			// 斜率限制针对的是“本周期相对上一周期的变化量”，
			const double delta = jointTorqueCommand[i] - previousJointTorqueCommand_[i];
			const double limit = config_.maxTorqueRate[i] * config_.controlPeriod;
			jointTorqueCommand[i] = previousJointTorqueCommand_[i] +
				std::max(-limit, std::min(limit, delta));
		}

		const double clampedTorque = clampToConfiguredRange(
			jointTorqueCommand[i],
			config_.lowerJointTorque[i],
			config_.upperJointTorque[i],
			config_.maxJointTorque[i]);
		if (clampedTorque != jointTorqueCommand[i])
		{
			jointTorqueCommand[i] = clampedTorque;
			jointProtectionStatus[i] = true;
			status = HIC_STATUS_ERROR_CURRENT_LIMIT;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		previousJointTorqueCommand_[i] = jointTorqueCommand[i];
	}
	return status;
}

HicStatus HicControlCoordinator::convertTorqueToCurrent(
	const double* jointTorqueCommand,
	double* motorCurrentCommand) const
{
	if (!jointTorqueCommand || !motorCurrentCommand)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		const double denom =
			config_.torqueConstant[i] * config_.gearRatio[i] * config_.transmissionEfficiency[i];
		if (denom <= 0.0)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}

		double current = jointTorqueCommand[i] / denom;
		if (config_.enableCurrentLimit)
		{
			current = clampToConfiguredRange(
				current,
				config_.lowerMotorCurrent[i],
				config_.upperMotorCurrent[i],
				config_.maxJointCurrent[i]);
		}
		motorCurrentCommand[i] = current;
	}
	return HIC_STATUS_OK;
}

// ============================================================
// ============================================================

void HicControlCoordinator::clearCurrentCommand(double* motorCurrentCommand) const
{
	if (!motorCurrentCommand)
	{
		return;
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		motorCurrentCommand[i] = 0.0;
	}
}

void HicControlCoordinator::clearTorqueCommand(double* jointTorqueCommand) const
{
	if (!jointTorqueCommand)
	{
		return;
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] = 0.0;
	}
}

void HicControlCoordinator::clearProtectionStatus(bool* jointProtectionStatus) const
{
	if (!jointProtectionStatus)
	{
		return;
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointProtectionStatus[i] = false;
	}
}

bool HicControlCoordinator::hasNaNOrInf(const double* data, int size) const
{
	for (int i = 0; i < size; ++i)
	{
		if (!std::isfinite(data[i]))
		{
			return true;
		}
	}
	return false;
}

void HicControlCoordinator::invalidateCommandCache()
{
	++commandInputVersion_;
	commandCacheValid_ = false;
}

} // namespace hic
