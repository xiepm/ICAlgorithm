#pragma once

#include "hic_controller/interface/hic_c_api.h"

namespace hic
{

class HicJointImpedanceCore
{
public:
	HicJointImpedanceCore();
	~HicJointImpedanceCore();

	HicStatus initialize(int jointCount, double controlPeriod);
	HicStatus setConfig(const HicJointImpedanceConfig& config);
	HicStatus captureTargetPosition(const double* jointPosition);
	HicStatus computeJointTorque(
		const double* jointPosition,
		const double* jointVelocity,
		const double* externalTorque,
		double* jointTorque);
	HicStatus getLastPositionError(double* positionError) const;
	HicStatus getLastVelocityError(double* velocityError) const;
	void reset();

private:
	int jointCount_;
	double controlPeriod_;
	bool initialized_;
	HicJointImpedanceConfig config_;
	double lastPositionError_[HIC_MAX_JOINTS];
	double lastVelocityError_[HIC_MAX_JOINTS];
};

} // namespace hic
