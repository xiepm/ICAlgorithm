#pragma once

#include "hic_common_types.h"

#include <memory>

using namespace KDL;

class dynamicsBase
{
public:
	virtual ~dynamicsBase() = default;

	virtual void setRobotDHParameters(
		const EcRealVector& kinematcisParam) = 0;

	virtual void setGravityVector(
		const EcReal gx, const EcReal gy, const EcReal gz) = 0;

	virtual void calculateGravityJointTorques(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau) = 0;

	virtual EcBoolean calculateEstimateJointToqrues(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau) = 0;

	virtual void calculateJointFricition(
		const EcRealVector& dq,
		EcReal coeffColomb,
		EcReal coeffViscous,
		EcRealVector& tau)
	{
		(void)dq;
		(void)coeffColomb;
		(void)coeffViscous;
		(void)tau;
	}

	virtual EcBoolean setPayloadMassProperties(
		const EcReal,
		const EcRealVector&)
	{
		return false;
	}

	virtual void calculateMomentumEstimatedJointTorques(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau) = 0;

	static EcReal sign(const EcReal x)
	{
		if (fabs(x) < 0.0113)
		{
			return x / 0.0113;
		}
		return (x > 0) ? 1.0 : -1.0;
	}
};

typedef std::shared_ptr<dynamicsBase> dynBasePtr;
