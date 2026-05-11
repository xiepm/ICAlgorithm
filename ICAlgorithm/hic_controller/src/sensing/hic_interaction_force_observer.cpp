#include "hic_controller/sensing/hic_interaction_force_observer.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace hic
{
namespace
{
double clampAlpha(double alpha)
{
	return std::max(0.0, std::min(1.0, alpha));
}

void copyCString(char* dst, std::size_t dstSize, const char* src)
{
	if (!dst || dstSize == 0)
	{
		return;
	}

	std::memset(dst, 0, dstSize);
	if (!src)
	{
		return;
	}

	std::strncpy(dst, src, dstSize - 1);
}
}

HicInteractionForceObserver::HicInteractionForceObserver()
	: jointCount_(0),
	  controlPeriod_(0.0),
	  initialized_(false)
{
	reset();
}

HicInteractionForceObserver::~HicInteractionForceObserver()
{
}

HicStatus HicInteractionForceObserver::initialize(int jointCount, double controlPeriod)
{
	if (jointCount <= 0 || jointCount > HIC_MAX_JOINTS || controlPeriod <= 0.0)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	jointCount_ = jointCount;
	controlPeriod_ = controlPeriod;
	reset();

	std::memset(&torqueSensorConfig_, 0, sizeof(torqueSensorConfig_));
	torqueSensorConfig_.jointCount = jointCount_;
	copyCString(torqueSensorConfig_.version, sizeof(torqueSensorConfig_.version), "1.0");
	copyCString(torqueSensorConfig_.unit, sizeof(torqueSensorConfig_.unit), "N.m");
	copyCString(torqueSensorConfig_.sensorLocation, sizeof(torqueSensorConfig_.sensorLocation), "unknown");
	torqueSensorConfig_.enableTorqueSensorFilter = true;
	torqueSensorConfig_.enableExternalTorqueFilter = true;
	torqueSensorConfig_.enableSaturationCheck = true;
	torqueSensorConfig_.enableFaultCheck = true;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		torqueSensorConfig_.joints[i].enabled = false;
		torqueSensorConfig_.joints[i].jointIndex = i + 1;
		torqueSensorConfig_.joints[i].hardwareChannel = i;
		torqueSensorConfig_.joints[i].ratedCapacityNm = 0.0;
		torqueSensorConfig_.joints[i].directionSign = 1;
		torqueSensorConfig_.joints[i].zeroOffsetNm = 0.0;
		torqueSensorConfig_.joints[i].scale = 1.0;
		torqueSensorConfig_.joints[i].biasNm = 0.0;
		torqueSensorConfig_.joints[i].maxValidTorqueNm = 0.0;
		torqueSensorConfig_.torqueSensorFilterAlpha[i] = 0.2;
		torqueSensorConfig_.externalTorqueFilterAlpha[i] = 0.2;
	}
	initialized_ = true;
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::setTorqueSensorConfig(const HicTorqueSensorConfig& config)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	const HicStatus status = validateTorqueSensorConfig(config);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	torqueSensorConfig_ = config;
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::getTorqueSensorConfig(HicTorqueSensorConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	configOut = torqueSensorConfig_;
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::loadTorqueSensorConfigFromFile(const char* filePath)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!filePath)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

HicStatus HicInteractionForceObserver::updateRawJointTorqueSensor(const double* rawTorqueByHardwareChannel)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!rawTorqueByHardwareChannel)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		if (!std::isfinite(rawTorqueByHardwareChannel[i]))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		rawTorqueByHardwareChannel_[i] = rawTorqueByHardwareChannel[i];
	}

	const HicStatus status = calibrateJointTorqueSensor(rawTorqueByHardwareChannel);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	applyTorqueSensorFilter();
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::updateJointExternalTorque(const double* jointExternalTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointExternalTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		if (!std::isfinite(jointExternalTorque[i]))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		jointExternalTorque_[i] = jointExternalTorque[i];
	}
	applyExternalTorqueFilter();
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::getCalibratedJointTorqueSensor(double* calibratedJointTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!calibratedJointTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		calibratedJointTorque[i] = calibratedJointTorqueSensor_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::getFilteredJointTorqueSensor(double* filteredJointTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!filteredJointTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		filteredJointTorque[i] = filteredJointTorqueSensor_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::getFilteredJointExternalTorque(double* jointExternalTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointExternalTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	for (int i = 0; i < jointCount_; ++i)
	{
		jointExternalTorque[i] = filteredJointExternalTorque_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::getTorqueSensorFaultStatus(bool* faultStatus) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!faultStatus)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		faultStatus[i] = torqueSensorFaultStatus_[i];
	}
	return HIC_STATUS_OK;
}

void HicInteractionForceObserver::reset()
{
	std::fill(rawTorqueByHardwareChannel_, rawTorqueByHardwareChannel_ + HIC_MAX_JOINTS, 0.0);
	std::fill(calibratedJointTorqueSensor_, calibratedJointTorqueSensor_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredJointTorqueSensor_, filteredJointTorqueSensor_ + HIC_MAX_JOINTS, 0.0);
	std::fill(jointExternalTorque_, jointExternalTorque_ + HIC_MAX_JOINTS, 0.0);
	std::fill(filteredJointExternalTorque_, filteredJointExternalTorque_ + HIC_MAX_JOINTS, 0.0);
	std::fill(torqueSensorFaultStatus_, torqueSensorFaultStatus_ + HIC_MAX_JOINTS, false);
}

HicStatus HicInteractionForceObserver::validateTorqueSensorConfig(const HicTorqueSensorConfig& config) const
{
	if (config.jointCount <= 0 || config.jointCount > HIC_MAX_JOINTS || config.jointCount != jointCount_)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	for (int i = 0; i < config.jointCount; ++i)
	{
		if (!std::isfinite(config.torqueSensorFilterAlpha[i]) ||
			!std::isfinite(config.externalTorqueFilterAlpha[i]) ||
			config.torqueSensorFilterAlpha[i] < 0.0 ||
			config.torqueSensorFilterAlpha[i] > 1.0 ||
			config.externalTorqueFilterAlpha[i] < 0.0 ||
			config.externalTorqueFilterAlpha[i] > 1.0)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}

		const HicJointTorqueSensorConfig& jointConfig = config.joints[i];
		if (!jointConfig.enabled)
		{
			continue;
		}

		if (jointConfig.jointIndex <= 0 || jointConfig.jointIndex > jointCount_)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		if (jointConfig.hardwareChannel < 0 || jointConfig.hardwareChannel >= jointCount_)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		if (jointConfig.directionSign != 1 && jointConfig.directionSign != -1)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		if (!std::isfinite(jointConfig.scale) || jointConfig.scale == 0.0)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		if (config.enableFaultCheck &&
			(!std::isfinite(jointConfig.maxValidTorqueNm) || jointConfig.maxValidTorqueNm <= 0.0))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
		if (!std::isfinite(jointConfig.zeroOffsetNm) ||
			!std::isfinite(jointConfig.biasNm) ||
			!std::isfinite(jointConfig.ratedCapacityNm))
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
	}

	return HIC_STATUS_OK;
}

HicStatus HicInteractionForceObserver::calibrateJointTorqueSensor(const double* rawTorqueByHardwareChannel)
{
	std::fill(calibratedJointTorqueSensor_, calibratedJointTorqueSensor_ + HIC_MAX_JOINTS, 0.0);
	std::fill(torqueSensorFaultStatus_, torqueSensorFaultStatus_ + HIC_MAX_JOINTS, false);

	for (int configIndex = 0; configIndex < jointCount_; ++configIndex)
	{
		const HicJointTorqueSensorConfig& jointConfig = torqueSensorConfig_.joints[configIndex];
		if (!jointConfig.enabled)
		{
			continue;
		}

		const int jointArrayIndex = jointConfig.jointIndex - 1;
		const int hardwareChannel = jointConfig.hardwareChannel;
		if (jointArrayIndex < 0 || jointArrayIndex >= jointCount_ ||
			hardwareChannel < 0 || hardwareChannel >= jointCount_)
		{
			if (torqueSensorConfig_.enableFaultCheck)
			{
				torqueSensorFaultStatus_[configIndex] = true;
				return HIC_STATUS_ERROR_INVALID_PARAM;
			}
			continue;
		}

		const double raw = rawTorqueByHardwareChannel[hardwareChannel];
		const double calibratedTorque =
			static_cast<double>(jointConfig.directionSign) *
			(raw - jointConfig.zeroOffsetNm) * jointConfig.scale + jointConfig.biasNm;
		calibratedJointTorqueSensor_[jointArrayIndex] = calibratedTorque;

		if (torqueSensorConfig_.enableSaturationCheck &&
			jointConfig.maxValidTorqueNm > 0.0 &&
			std::fabs(calibratedTorque) > jointConfig.maxValidTorqueNm)
		{
			torqueSensorFaultStatus_[jointArrayIndex] = true;
		}
	}

	return HIC_STATUS_OK;
}

void HicInteractionForceObserver::applyTorqueSensorFilter()
{
	for (int i = 0; i < jointCount_; ++i)
	{
		if (!torqueSensorConfig_.enableTorqueSensorFilter)
		{
			filteredJointTorqueSensor_[i] = calibratedJointTorqueSensor_[i];
			continue;
		}

		const double alpha = clampAlpha(torqueSensorConfig_.torqueSensorFilterAlpha[i]);
		filteredJointTorqueSensor_[i] =
			(1.0 - alpha) * filteredJointTorqueSensor_[i] + alpha * calibratedJointTorqueSensor_[i];
	}
}

void HicInteractionForceObserver::applyExternalTorqueFilter()
{
	for (int i = 0; i < jointCount_; ++i)
	{
		if (!torqueSensorConfig_.enableExternalTorqueFilter)
		{
			filteredJointExternalTorque_[i] = jointExternalTorque_[i];
			continue;
		}

		const double alpha = clampAlpha(torqueSensorConfig_.externalTorqueFilterAlpha[i]);
		filteredJointExternalTorque_[i] =
			(1.0 - alpha) * filteredJointExternalTorque_[i] + alpha * jointExternalTorque_[i];
	}
}

} // namespace hic
