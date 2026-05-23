#include "hic_controller/state/hic_robot_state_observer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace hic
{
namespace
{
double clampAlpha(double alpha)
{
	return std::max(0.0, std::min(1.0, alpha));
}

bool hasExplicitRange(double lower, double upper)
{
	return lower != 0.0 || upper != 0.0;
}

bool outOfRange(double value, double lower, double upper, double maxAbs)
{
	if (hasExplicitRange(lower, upper))
	{
		return value < lower || value > upper;
	}
	if (maxAbs > 0.0)
	{
		return std::fabs(value) > maxAbs;
	}
	return false;
}

bool shouldDebugPrint(int& counter)
{
	return ((counter++ % 1000) == 0);
}
}

HicRobotStateObserver::HicRobotStateObserver()
	: initialized_(false),
	  stateValid_(false),
	  previousTimestamp_(0.0),
	  hasPreviousState_(false)
{
	std::memset(&config_, 0, sizeof(config_));
	reset();
}

HicRobotStateObserver::~HicRobotStateObserver()
{
}

HicStatus HicRobotStateObserver::initialize(const HicRobotStateObserverConfig& config)
{
	const HicStatus status = validateConfig(config);
	if (status != HIC_STATUS_OK)
	{
		initialized_ = false;
		stateValid_ = false;
		return status;
	}

	config_ = config;
	reset();
	initialized_ = true;
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::setConfig(const HicRobotStateObserverConfig& config)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	const HicStatus status = validateConfig(config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	config_ = config;
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getConfig(HicRobotStateObserverConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	configOut = config_;
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::updateJointState(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration)
{
#ifdef HIC_ENABLE_DEBUG_PRINT
	static int debug_update_joint_state_count = 0;
	const bool should_debug_print = shouldDebugPrint(debug_update_joint_state_count);
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateJointState] entered, initialized=%d jointCount=%d\n",
			initialized_ ? 1 : 0,
			config_.jointCount);
		std::fflush(stderr);
	}
#endif
	if (!initialized_)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateJointState] return status=%d: not initialized\n",
				static_cast<int>(HIC_STATUS_ERROR_INIT));
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !jointVelocity || !jointAcceleration)
	{
		stateValid_ = false;
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateJointState] return status=%d: null input\n",
				static_cast<int>(HIC_STATUS_ERROR_NULL_POINTER));
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicStatus status = validateArrayFinite(jointPosition, config_.jointCount);
	if (status != HIC_STATUS_OK)
	{
		stateValid_ = false;
		return status;
	}
	status = validateArrayFinite(jointVelocity, config_.jointCount);
	if (status != HIC_STATUS_OK)
	{
		stateValid_ = false;
		return status;
	}
	status = validateArrayFinite(jointAcceleration, config_.jointCount);
	if (status != HIC_STATUS_OK)
	{
		stateValid_ = false;
		return status;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		rawJointPosition_[i] = jointPosition[i];
		rawJointVelocity_[i] = jointVelocity[i];
		rawJointAcceleration_[i] = jointAcceleration[i];
	}

	applyFirstOrderFilter(rawJointPosition_, filteredJointPosition_, config_.positionFilterAlpha,
		config_.jointCount, config_.enablePositionFilter);
	applyFirstOrderFilter(rawJointVelocity_, filteredJointVelocity_, config_.velocityFilterAlpha,
		config_.jointCount, config_.enableVelocityFilter);
	applyFirstOrderFilter(rawJointAcceleration_, filteredJointAcceleration_, config_.accelerationFilterAlpha,
		config_.jointCount, config_.enableAccelerationFilter);

	status = validateStateRange();
	stateValid_ = (status == HIC_STATUS_OK);
#ifdef HIC_ENABLE_DEBUG_PRINT
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateJointState] validateStateRange status=%d stateValid=%d\n",
			static_cast<int>(status),
			stateValid_ ? 1 : 0);
		std::fflush(stderr);
	}
#endif
	return status;
}

HicStatus HicRobotStateObserver::updateMotorCurrent(const double* motorCurrent)
{
#ifdef HIC_ENABLE_DEBUG_PRINT
	static int debug_update_current_count = 0;
	const bool should_debug_print = shouldDebugPrint(debug_update_current_count);
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateMotorCurrent] entered, initialized=%d jointCount=%d\n",
			initialized_ ? 1 : 0,
			config_.jointCount);
		std::fflush(stderr);
	}
#endif
	if (!initialized_)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateMotorCurrent] return status=%d: not initialized\n",
				static_cast<int>(HIC_STATUS_ERROR_INIT));
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_INIT;
	}
	if (!motorCurrent)
	{
		stateValid_ = false;
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateMotorCurrent] return status=%d: motorCurrent=null\n",
				static_cast<int>(HIC_STATUS_ERROR_NULL_POINTER));
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	const HicStatus status = validateArrayFinite(motorCurrent, config_.jointCount);
	if (status != HIC_STATUS_OK)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateMotorCurrent] motorCurrent invalid, status=%d\n",
				static_cast<int>(status));
			std::fflush(stderr);
		}
#endif
		stateValid_ = false;
		return status;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		rawMotorCurrent_[i] = motorCurrent[i];
	}

	applyFirstOrderFilter(
		rawMotorCurrent_, filteredMotorCurrent_, config_.motorCurrentFilterAlpha,
		config_.jointCount, config_.enableMotorCurrentFilter);

	const HicStatus torqueStatus = estimateMotorTorque();
	if (torqueStatus != HIC_STATUS_OK)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateMotorCurrent] estimateMotorTorque status=%d\n",
				static_cast<int>(torqueStatus));
			std::fflush(stderr);
		}
#endif
		stateValid_ = false;
		return torqueStatus;
	}

	const HicStatus rangeStatus = validateStateRange();
	stateValid_ = (rangeStatus == HIC_STATUS_OK);
#ifdef HIC_ENABLE_DEBUG_PRINT
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateMotorCurrent] validateStateRange status=%d stateValid=%d\n",
			static_cast<int>(rangeStatus),
			stateValid_ ? 1 : 0);
		for (int i = 0; i < config_.jointCount; ++i)
		{
			std::fprintf(stderr,
				"  current[%d] raw=%.6f filtered=%.6f tauEst=%.6f\n",
				i,
				rawMotorCurrent_[i],
				filteredMotorCurrent_[i],
				motorEstimatedTorque_[i]);
		}
		std::fflush(stderr);
	}
#endif
	return rangeStatus;
}

HicStatus HicRobotStateObserver::updateJointMeasuredTorque(const double* jointMeasuredTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointMeasuredTorque)
	{
		stateValid_ = false;
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	const HicStatus status = validateArrayFinite(jointMeasuredTorque, config_.jointCount);
	if (status != HIC_STATUS_OK)
	{
		stateValid_ = false;
		return status;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		rawJointMeasuredTorque_[i] = jointMeasuredTorque[i];
	}

	applyFirstOrderFilter(
		rawJointMeasuredTorque_, filteredJointMeasuredTorque_, config_.measuredTorqueFilterAlpha,
		config_.jointCount, config_.enableMeasuredTorqueFilter);

	const HicStatus rangeStatus = validateStateRange();
	stateValid_ = (rangeStatus == HIC_STATUS_OK);
	return rangeStatus;
}

HicStatus HicRobotStateObserver::updateRobotState(
	const double* jointPosition,
	const double* motorCurrent,
	double currentTime)
{
#ifdef HIC_ENABLE_DEBUG_PRINT
	static int debug_update_robot_state_count = 0;
	const bool should_debug_print = shouldDebugPrint(debug_update_robot_state_count);
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateRobotState] entered, initialized=%d jointCount=%d t=%.6f hasPrevious=%d\n",
			initialized_ ? 1 : 0,
			config_.jointCount,
			currentTime,
			hasPreviousState_ ? 1 : 0);
		std::fflush(stderr);
	}
#endif
	if (!initialized_)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateRobotState] return status=%d: not initialized\n",
				static_cast<int>(HIC_STATUS_ERROR_INIT));
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !motorCurrent)
	{
		stateValid_ = false;
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateRobotState] return status=%d: jointPosition=%p motorCurrent=%p\n",
				static_cast<int>(HIC_STATUS_ERROR_NULL_POINTER),
				static_cast<const void*>(jointPosition),
				static_cast<const void*>(motorCurrent));
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (!std::isfinite(currentTime))
	{
		stateValid_ = false;
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateRobotState] return status=%d: currentTime invalid %.6f\n",
				static_cast<int>(HIC_STATUS_ERROR_INVALID_PARAM),
				currentTime);
			std::fflush(stderr);
		}
#endif
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	const HicStatus positionStatus = validateArrayFinite(jointPosition, config_.jointCount);
	if (positionStatus != HIC_STATUS_OK)
	{
		stateValid_ = false;
#ifdef HIC_ENABLE_DEBUG_PRINT
		if (should_debug_print)
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::updateRobotState] jointPosition invalid status=%d\n",
				static_cast<int>(positionStatus));
			std::fflush(stderr);
		}
#endif
		return positionStatus;
	}

	double estimatedVelocity[HIC_MAX_JOINTS] = { 0.0 };
	double estimatedAcceleration[HIC_MAX_JOINTS] = { 0.0 };
	double dt = config_.controlPeriod;
	if (hasPreviousState_)
	{
		dt = currentTime - previousTimestamp_;
		if (!std::isfinite(dt) || dt <= 0.0)
		{
			dt = config_.controlPeriod;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (!hasPreviousState_)
		{
			estimatedVelocity[i] = 0.0;
			estimatedAcceleration[i] = 0.0;
			continue;
		}
		estimatedVelocity[i] = (jointPosition[i] - previousJointPosition_[i]) / dt;
		estimatedAcceleration[i] = (estimatedVelocity[i] - previousJointVelocity_[i]) / dt;
	}

	HicStatus status = updateJointState(jointPosition, estimatedVelocity, estimatedAcceleration);
#ifdef HIC_ENABLE_DEBUG_PRINT
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateRobotState] updateJointState status=%d dt=%.9f\n",
			static_cast<int>(status),
			dt);
		std::fflush(stderr);
	}
#endif
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = updateMotorCurrent(motorCurrent);
#ifdef HIC_ENABLE_DEBUG_PRINT
	if (should_debug_print)
	{
		std::fprintf(stderr,
			"[HicRobotStateObserver::updateRobotState] updateMotorCurrent status=%d\n",
			static_cast<int>(status));
		std::fflush(stderr);
	}
#endif
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		previousJointPosition_[i] = jointPosition[i];
		previousJointVelocity_[i] = estimatedVelocity[i];
	}
	previousTimestamp_ = currentTime;
	hasPreviousState_ = true;
	stateValid_ = true;
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getRobotState(HicRobotState& stateOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		stateOut.jointPosition[i] = filteredJointPosition_[i];
		stateOut.jointVelocity[i] = filteredJointVelocity_[i];
		stateOut.jointAcceleration[i] = filteredJointAcceleration_[i];
		stateOut.motorCurrent[i] = filteredMotorCurrent_[i];
		stateOut.jointMeasuredTorque[i] = filteredJointMeasuredTorque_[i];
		stateOut.motorEstimatedTorque[i] = motorEstimatedTorque_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getFilteredJointPosition(double* jointPosition) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointPosition[i] = filteredJointPosition_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getFilteredJointVelocity(double* jointVelocity) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointVelocity)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointVelocity[i] = filteredJointVelocity_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getFilteredJointAcceleration(double* jointAcceleration) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointAcceleration)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointAcceleration[i] = filteredJointAcceleration_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getFilteredMotorCurrent(double* motorCurrent) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!motorCurrent)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		motorCurrent[i] = filteredMotorCurrent_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getFilteredJointMeasuredTorque(double* jointMeasuredTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointMeasuredTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointMeasuredTorque[i] = filteredJointMeasuredTorque_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::getMotorEstimatedTorque(double* motorEstimatedTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!motorEstimatedTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		motorEstimatedTorque[i] = motorEstimatedTorque_[i];
	}
	return HIC_STATUS_OK;
}

bool HicRobotStateObserver::isStateValid() const
{
	return initialized_ && stateValid_;
}

void HicRobotStateObserver::reset()
{
	std::fill(rawJointPosition_, rawJointPosition_ + HIC_MAX_JOINTS, 0.0);
	std::fill(rawJointVelocity_, rawJointVelocity_ + HIC_MAX_JOINTS, 0.0);
	std::fill(rawJointAcceleration_, rawJointAcceleration_ + HIC_MAX_JOINTS, 0.0);
	std::fill(rawMotorCurrent_, rawMotorCurrent_ + HIC_MAX_JOINTS, 0.0);
	std::fill(rawJointMeasuredTorque_, rawJointMeasuredTorque_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredJointPosition_, filteredJointPosition_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredJointVelocity_, filteredJointVelocity_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredJointAcceleration_, filteredJointAcceleration_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredMotorCurrent_, filteredMotorCurrent_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredJointMeasuredTorque_, filteredJointMeasuredTorque_ + HIC_MAX_JOINTS, 0.0);
	std::fill(motorEstimatedTorque_, motorEstimatedTorque_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousJointPosition_, previousJointPosition_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousJointVelocity_, previousJointVelocity_ + HIC_MAX_JOINTS, 0.0);
	previousTimestamp_ = 0.0;
	hasPreviousState_ = false;
	stateValid_ = false;
}

HicStatus HicRobotStateObserver::validateConfig(const HicRobotStateObserverConfig& config) const
{
	if (config.jointCount <= 0 || config.jointCount > HIC_MAX_JOINTS || config.controlPeriod <= 0.0)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	for (int i = 0; i < config.jointCount; ++i)
	{
		const double alphaValues[5] = {
			config.positionFilterAlpha[i],
			config.velocityFilterAlpha[i],
			config.accelerationFilterAlpha[i],
			config.motorCurrentFilterAlpha[i],
			config.measuredTorqueFilterAlpha[i]
		};
		for (double alpha : alphaValues)
		{
			if (!std::isfinite(alpha) || alpha < 0.0 || alpha > 1.0)
			{
				return HIC_STATUS_ERROR_INVALID_PARAM;
			}
		}

		if (!std::isfinite(config.maxAbsJointPosition[i]) ||
			!std::isfinite(config.lowerJointPosition[i]) ||
			!std::isfinite(config.upperJointPosition[i]) ||
			!std::isfinite(config.maxAbsJointVelocity[i]) ||
			!std::isfinite(config.lowerJointVelocity[i]) ||
			!std::isfinite(config.upperJointVelocity[i]) ||
			!std::isfinite(config.maxAbsJointAcceleration[i]) ||
			!std::isfinite(config.lowerJointAcceleration[i]) ||
			!std::isfinite(config.upperJointAcceleration[i]) ||
			!std::isfinite(config.maxAbsMotorCurrent[i]) ||
			!std::isfinite(config.lowerMotorCurrent[i]) ||
			!std::isfinite(config.upperMotorCurrent[i]) ||
			!std::isfinite(config.maxAbsMeasuredTorque[i]) ||
			!std::isfinite(config.lowerMeasuredTorque[i]) ||
			!std::isfinite(config.upperMeasuredTorque[i]) ||
			!std::isfinite(config.torqueConstant[i]) ||
			!std::isfinite(config.gearRatio[i]) ||
			!std::isfinite(config.transmissionEfficiency[i]))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}

		if ((hasExplicitRange(config.lowerJointPosition[i], config.upperJointPosition[i]) &&
			 config.lowerJointPosition[i] > config.upperJointPosition[i]) ||
			(hasExplicitRange(config.lowerJointVelocity[i], config.upperJointVelocity[i]) &&
			 config.lowerJointVelocity[i] > config.upperJointVelocity[i]) ||
			(hasExplicitRange(config.lowerJointAcceleration[i], config.upperJointAcceleration[i]) &&
			 config.lowerJointAcceleration[i] > config.upperJointAcceleration[i]) ||
			(hasExplicitRange(config.lowerMotorCurrent[i], config.upperMotorCurrent[i]) &&
			 config.lowerMotorCurrent[i] > config.upperMotorCurrent[i]) ||
			(hasExplicitRange(config.lowerMeasuredTorque[i], config.upperMeasuredTorque[i]) &&
			 config.lowerMeasuredTorque[i] > config.upperMeasuredTorque[i]))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
	}

	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::validateArrayFinite(const double* data, int length) const
{
	if (!data)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < length; ++i)
	{
		if (!std::isfinite(data[i]))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}
	}
	return HIC_STATUS_OK;
}

void HicRobotStateObserver::applyFirstOrderFilter(
	const double* input,
	double* output,
	const double* alpha,
	int length,
	bool enabled)
{
	for (int i = 0; i < length; ++i)
	{
		if (!enabled)
		{
			output[i] = input[i];
			continue;
		}

		const double alphaValue = clampAlpha(alpha[i]);
		output[i] = (1.0 - alphaValue) * output[i] + alphaValue * input[i];
	}
}

HicStatus HicRobotStateObserver::estimateMotorTorque()
{
	// 电机电流到关节侧力矩的估算链路。
	//
	// 物理含义：
	// 1. filteredMotorCurrent_[i] 是第 i 关节电机电流，单位 A；
	// 2. torqueConstant[i] 是电机力矩常数 Kt，单位 N.m/A，表示电机侧每安培产生的力矩；
	// 3. gearRatio[i] 是减速比，用于从电机侧力矩折算到关节侧；
	// 4. transmissionEfficiency[i] 是传动效率，用于考虑减速器/传动链损失；
	// 5. 输出 motorEstimatedTorque_[i] 是关节侧电机估计力矩，单位 N.m。
	//
	// 当前公式：
	// motorEstimatedTorque = filteredMotorCurrent * torqueConstant * gearRatio * transmissionEfficiency
	//
	// 符号方向依赖外部电流方向、Kt 符号和关节正方向约定；如果打印出来发现方向反了，
	// 应优先检查 torqueConstant 或电流采样方向是否需要取负。
	if (!config_.enableCurrentToTorqueEstimate)
	{
#ifdef HIC_ENABLE_DEBUG_PRINT
		static int debug_disabled_count = 0;
		if (shouldDebugPrint(debug_disabled_count))
		{
			std::fprintf(stderr,
				"[HicRobotStateObserver::estimateMotorTorque] skipped: enableCurrentToTorqueEstimate=false\n");
			std::fflush(stderr);
		}
#endif
		// 禁用电流反推时，显式清零估计力矩，避免旧值被 coordinator 用于外力矩估计。
		std::fill(motorEstimatedTorque_, motorEstimatedTorque_ + HIC_MAX_JOINTS, 0.0);
		return HIC_STATUS_OK;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		const double torqueConstant = config_.torqueConstant[i];
		const double gearRatio = config_.gearRatio[i];
		const double efficiency = config_.transmissionEfficiency[i];
		if (!std::isfinite(torqueConstant) || !std::isfinite(gearRatio) || !std::isfinite(efficiency))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		motorEstimatedTorque_[i] = filteredMotorCurrent_[i] * torqueConstant * gearRatio * efficiency;
	}

#ifdef HIC_ENABLE_DEBUG_PRINT
	static int debug_count = 0;
	if (shouldDebugPrint(debug_count))
	{
		std::fprintf(stderr, "[HicRobotStateObserver::estimateMotorTorque] jointCount=%d\n", config_.jointCount);
		for (int i = 0; i < config_.jointCount; ++i)
		{
			std::fprintf(stderr,
				"  joint[%d] rawCurrent=%.6f A filteredCurrent=%.6f A "
				"Kt=%.6f Nm/A gearRatio=%.6f efficiency=%.6f estimatedTorque=%.6f Nm\n",
				i,
				rawMotorCurrent_[i],
				filteredMotorCurrent_[i],
				config_.torqueConstant[i],
				config_.gearRatio[i],
				config_.transmissionEfficiency[i],
				motorEstimatedTorque_[i]);
		}
		std::fflush(stderr);
	}
#endif

	return HIC_STATUS_OK;
}

HicStatus HicRobotStateObserver::validateStateRange() const
{
	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (outOfRange(
			filteredJointPosition_[i],
			config_.lowerJointPosition[i],
			config_.upperJointPosition[i],
			config_.maxAbsJointPosition[i]))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}
		if (outOfRange(
			filteredJointVelocity_[i],
			config_.lowerJointVelocity[i],
			config_.upperJointVelocity[i],
			config_.maxAbsJointVelocity[i]))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}
		if (outOfRange(
			filteredJointAcceleration_[i],
			config_.lowerJointAcceleration[i],
			config_.upperJointAcceleration[i],
			config_.maxAbsJointAcceleration[i]))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}
		if (outOfRange(
			filteredMotorCurrent_[i],
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxAbsMotorCurrent[i]))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}
		if (outOfRange(
			filteredJointMeasuredTorque_[i],
			config_.lowerMeasuredTorque[i],
			config_.upperMeasuredTorque[i],
			config_.maxAbsMeasuredTorque[i]))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}
	}
	return HIC_STATUS_OK;
}

} // namespace hic
