#include "hic_controller/dynamics/hic_dynamics_adapter.h"

#include "hic_controller/dynamics/anthorDynamics.h"
#include "hic_controller/dynamics/dsDynamics.h"
#include "hic_controller/dynamics/elfinDynamics.h"
#include "hic_controller/dynamics/palletDynamics.h"
#include "hic_controller/dynamics/sevendofDynamics.h"
#include "hic_controller/dynamics/urDynamics.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace hic
{
namespace
{
const int kSevenDofRobotType = 20;
const int kSevenDofModelJointCount = 7;

dynBasePtr* backendPtr(void* raw)
{
	return static_cast<dynBasePtr*>(raw);
}

dynBasePtr createBackend(int robotType)
{
	switch (robotType)
	{
	default:
	case 0:
	case 2:
	case 7:
	case 9:
		return dynBasePtr(new elfinDynamics());
	case 1:
	case 3:
	case 8:
	case 10:
		return dynBasePtr(new urDynamics());
	case 5:
		return dynBasePtr(new palletDynamics());
	case 12:
		return dynBasePtr(new dsDynamics());
	case 20:
		// 7轴机型当前绑定到 sevendofDynamics。
		// 后续所有通过 dynamics adapter 调用的重力/估计扭矩接口，
		// 在 robotType == 20 时都会动态派发到 sevendofDynamics。
		return dynBasePtr(new sevendofDynamics());
	}
}
}

HicDynamicsAdapter::HicDynamicsAdapter()
	: robotType_(0),
	  jointCount_(0),
	  modelJointCount_(0),
	  initialized_(false),
	  backend_(0),
	  payloadMass_(0.0)
{
	std::fill(dynamicParams_current_, dynamicParams_current_ + HIC_MAX_DYNAMIC_PARAMS, 0.0);
	std::fill(dynamicParams_sensor_, dynamicParams_sensor_ + HIC_MAX_DYNAMIC_PARAMS, 0.0);
	std::fill(payloadCenterOfMass_, payloadCenterOfMass_ + 3, 0.0);
}

HicDynamicsAdapter::~HicDynamicsAdapter()
{
	reset();
}

HicStatus HicDynamicsAdapter::initialize(
	int robotType,
	int jointCount,
	const double* dynamicParams)
{
	if (!dynamicParams || jointCount <= 0 || jointCount > HIC_MAX_JOINTS)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}
	if (robotType == kSevenDofRobotType && jointCount < kSevenDofModelJointCount)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	reset();
	backend_ = new dynBasePtr(createBackend(robotType));
	if (!backendPtr(backend_)->get())
	{
		reset();
		return HIC_STATUS_ERROR_INIT;
	}

	std::memcpy(dynamicParams_current_, dynamicParams, sizeof(double) * HIC_MAX_DYNAMIC_PARAMS);
	std::memcpy(dynamicParams_sensor_, dynamicParams, sizeof(double) * HIC_MAX_DYNAMIC_PARAMS);
	(*backendPtr(backend_))->setGravityVector(0.0, 0.0, -9.81);
	robotType_ = robotType;
	jointCount_ = jointCount;
	modelJointCount_ = (robotType == kSevenDofRobotType) ? kSevenDofModelJointCount : jointCount;
	initialized_ = true;
#ifdef HIC_ENABLE_DEBUG_PRINT
	std::fprintf(stderr,
		"[HicDynamicsAdapter::initialize] robotType=%d externalJointCount=%d modelJointCount=%d\n",
		robotType_,
		jointCount_,
		modelJointCount_);
	std::fflush(stderr);
#endif
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::setDynamicParameters(const double* dynamicParams)
{
	HicStatus status = setDynamicParametersTo(dynamicParams_current_, dynamicParams);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	return setDynamicParametersTo(dynamicParams_sensor_, dynamicParams);
}

HicStatus HicDynamicsAdapter::setDynamicParameters_current(const double* dynamicParams)
{
	HicStatus status = setDynamicParametersTo(dynamicParams_current_, dynamicParams);
	return status;
}

HicStatus HicDynamicsAdapter::setDynamicParameters_sensor(const double* dynamicParams)
{
	return setDynamicParametersTo(dynamicParams_sensor_, dynamicParams);
}

HicStatus HicDynamicsAdapter::setRobotKinematicParameters(const double* kinematicParams)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!kinematicParams)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	EcRealVector params(HIC_MAX_JOINTS, 0.0);
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		params[i] = kinematicParams[i];
	}
	(*backendPtr(backend_))->setRobotDHParameters(params);
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::setGravityVector(
	double gx,
	double gy,
	double gz)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz))
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	(*backendPtr(backend_))->setGravityVector(gx, gy, gz);
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::setPayloadMassProperties(
	double mass,
	const double centerOfMass[3])
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!centerOfMass || mass < 0.0)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	payloadMass_ = mass;
	for (int i = 0; i < 3; ++i)
	{
		payloadCenterOfMass_[i] = centerOfMass[i];
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

HicStatus HicDynamicsAdapter::computeGravityTorque(
	const double* jointPosition,
	double* gravityTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !gravityTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	std::fill(gravityTorque, gravityTorque + jointCount_, 0.0);

	EcRealVector q(modelJointCount_, 0.0), tau(modelJointCount_, 0.0), params(modelJointCount_ * 13, 0.0);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		q[i] = jointPosition[i];
	}
	for (int i = 0; i < modelJointCount_ * 13; ++i)
	{
		params[i] = dynamicParams_current_[i];
	}

	(*backendPtr(backend_))->calculateGravityJointTorques(q, params, tau);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		gravityTorque[i] = tau[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::computeCoriolisTorque(
	const double* jointPosition,
	const double* jointVelocity,
	double* coriolisTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !jointVelocity || !coriolisTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	std::fill(coriolisTorque, coriolisTorque + jointCount_, 0.0);

	EcRealVector q(modelJointCount_, 0.0), dq(modelJointCount_, 0.0), tau(modelJointCount_, 0.0);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		q[i] = jointPosition[i];
		dq[i] = jointVelocity[i];
	}

	(*backendPtr(backend_))->computeCoriolisTorque(q, dq, tau);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		coriolisTorque[i] = tau[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::computeMassMatrix(
	const double*,
	double* massMatrixRowMajor)
{
	if (!massMatrixRowMajor)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	for (int i = 0; i < jointCount_ * jointCount_; ++i)
	{
		massMatrixRowMajor[i] = 0.0;
	}
	return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
}

HicStatus HicDynamicsAdapter::computeFrictionTorque(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration,
	double* frictionTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !jointVelocity || !jointAcceleration || !frictionTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	std::fill(frictionTorque, frictionTorque + jointCount_, 0.0);

	EcRealVector q(modelJointCount_, 0.0), dq(modelJointCount_, 0.0), ddq(modelJointCount_, 0.0), tau(modelJointCount_, 0.0);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		q[i] = jointPosition[i];
		dq[i] = jointVelocity[i];
		ddq[i] = jointAcceleration[i];
	}

	(*backendPtr(backend_))->computeFrictionTorque(q, dq, ddq, tau);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		frictionTorque[i] = tau[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::computeModelTorque_current(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration,
	double* modelTorque)
{
	return computeModelTorqueWithParameters(
		dynamicParams_current_,
		jointPosition,
		jointVelocity,
		jointAcceleration,
		modelTorque);
}

HicStatus HicDynamicsAdapter::computeModelTorque_sensor(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration,
	double* modelTorque)
{
	return computeModelTorqueWithParameters(
		dynamicParams_sensor_,
		jointPosition,
		jointVelocity,
		jointAcceleration,
		modelTorque);
}

HicStatus HicDynamicsAdapter::setDynamicParametersTo(double* destination, const double* dynamicParams)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!destination || !dynamicParams)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	std::memcpy(destination, dynamicParams, sizeof(double) * HIC_MAX_DYNAMIC_PARAMS);
	return HIC_STATUS_OK;
}

HicStatus HicDynamicsAdapter::computeModelTorqueWithParameters(
	const double* dynamicParams,
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration,
	double* modelTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!dynamicParams || !jointPosition || !jointVelocity || !jointAcceleration || !modelTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	std::fill(modelTorque, modelTorque + jointCount_, 0.0);
	EcRealVector q(modelJointCount_, 0.0);
	EcRealVector dq(modelJointCount_, 0.0);
	EcRealVector ddq(modelJointCount_, 0.0);
	EcRealVector params(modelJointCount_ * 13, 0.0);
	EcRealVector tau(modelJointCount_, 0.0);
	for (int i = 0; i < modelJointCount_; ++i)
	{
		q[i] = jointPosition[i];
		dq[i] = jointVelocity[i];
		ddq[i] = jointAcceleration[i];
	}
	for (int i = 0; i < modelJointCount_ * 13; ++i)
	{
		params[i] = dynamicParams[i];
	}

	const EcBoolean ok = (*backendPtr(backend_))->calculateEstimateJointToqrues(q, dq, ddq, params, tau);
	if (!ok)
	{
		return HIC_STATUS_ERROR_NOT_IMPLEMENTED;
	}
	for (int i = 0; i < modelJointCount_; ++i)
	{
		modelTorque[i] = tau[i];
	}
	return HIC_STATUS_OK;
}

void HicDynamicsAdapter::reset()
{
	if (backend_)
	{
		delete backendPtr(backend_);
		backend_ = 0;
	}
	initialized_ = false;
	robotType_ = 0;
	jointCount_ = 0;
	modelJointCount_ = 0;
	std::fill(dynamicParams_current_, dynamicParams_current_ + HIC_MAX_DYNAMIC_PARAMS, 0.0);
	std::fill(dynamicParams_sensor_, dynamicParams_sensor_ + HIC_MAX_DYNAMIC_PARAMS, 0.0);
	payloadMass_ = 0.0;
	std::fill(payloadCenterOfMass_, payloadCenterOfMass_ + 3, 0.0);
}

} // namespace hic
