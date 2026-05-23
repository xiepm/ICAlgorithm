#include "hic_controller/control/hic_joint_impedance_core.h"

#include <algorithm>
#include <cstring>

namespace hic
{

HicJointImpedanceCore::HicJointImpedanceCore()
	: jointCount_(0),
	  controlPeriod_(0.0),
	  initialized_(false)
{
	reset();
}

HicJointImpedanceCore::~HicJointImpedanceCore()
{
}

HicStatus HicJointImpedanceCore::initialize(int jointCount, double controlPeriod)
{
	if (jointCount <= 0 || jointCount > HIC_MAX_JOINTS || controlPeriod <= 0.0)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	jointCount_ = jointCount;
	controlPeriod_ = controlPeriod;
	initialized_ = true;
	return HIC_STATUS_OK;
}

HicStatus HicJointImpedanceCore::setConfig(const HicJointImpedanceConfig& config)
{
	config_ = config;
	return HIC_STATUS_OK;
}

HicStatus HicJointImpedanceCore::captureTargetPosition(const double* jointPosition)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		config_.targetPosition[i] = jointPosition[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicJointImpedanceCore::computeJointTorque(
	const double* jointPosition,
	const double* jointVelocity,
	const double* externalTorque,
	double* jointTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !jointVelocity || !jointTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		const double positionError = config_.targetPosition[i] - jointPosition[i];
		const double velocityError = config_.targetVelocity[i] - jointVelocity[i];
		double torque = config_.stiffness[i] * positionError + config_.damping[i] * velocityError;
		if (config_.enableExternalTorqueCompensation && externalTorque)
		{
			torque -= externalTorque[i];
		}

		lastPositionError_[i] = positionError;
		lastVelocityError_[i] = velocityError;
		jointTorque[i] = torque;
	}
	return HIC_STATUS_OK;
}

HicStatus HicJointImpedanceCore::getLastPositionError(double* positionError) const
{
	if (!positionError)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		positionError[i] = lastPositionError_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicJointImpedanceCore::getLastVelocityError(double* velocityError) const
{
	if (!velocityError)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < jointCount_; ++i)
	{
		velocityError[i] = lastVelocityError_[i];
	}
	return HIC_STATUS_OK;
}

void HicJointImpedanceCore::reset()
{
	std::memset(&config_, 0, sizeof(config_));
	std::fill(lastPositionError_, lastPositionError_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastVelocityError_, lastVelocityError_ + HIC_MAX_JOINTS, 0.0);
}

} // namespace hic
