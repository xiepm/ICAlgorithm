#include "hic_controller/control/hic_cartesian_impedance_core.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace hic
{
namespace
{
void setIdentityPose(double pose[HIC_POSE_DIM])
{
	pose[0] = 0.0;
	pose[1] = 0.0;
	pose[2] = 0.0;
	pose[3] = 1.0;
	pose[4] = 0.0;
	pose[5] = 0.0;
	pose[6] = 0.0;
}

Eigen::Quaterniond toQuaternion(const double pose[HIC_POSE_DIM])
{
	return Eigen::Quaterniond(pose[3], pose[4], pose[5], pose[6]).normalized();
}
}

HicCartesianImpedanceCore::HicCartesianImpedanceCore()
	: jointCount_(0),
	  controlPeriod_(0.0),
	  initialized_(false),
	  forceControlMode_(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE)
{
	reset();
}

HicCartesianImpedanceCore::~HicCartesianImpedanceCore()
{
}

HicStatus HicCartesianImpedanceCore::initialize(int jointCount, double controlPeriod)
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

HicStatus HicCartesianImpedanceCore::setGains(const HicImpedanceGains& gains)
{
	gains_ = gains;
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::setForceControlMode(HicForceControlMode forceControlMode)
{
	if (forceControlMode != HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION &&
		forceControlMode != HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE &&
		forceControlMode != HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}
	forceControlMode_ = forceControlMode;
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::setNullspaceConfig(const HicNullspaceControlConfig& config)
{
	nullspaceConfig_ = config;
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::captureCurrentJointPositionAsNullspaceTarget(const double* jointPosition)
{
	if (!jointPosition)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	for (int i = 0; i < jointCount_; ++i)
	{
		nullspaceConfig_.targetJointPosition[i] = jointPosition[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::setFixedTargetPosition(const double targetPosition[3])
{
	if (!targetPosition)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	for (int i = 0; i < 3; ++i)
	{
		targetPose_[i] = targetPosition[i];
	}
	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		targetVelocity_[i] = 0.0;
	}
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::setFixedTargetPose(const double targetPose[HIC_POSE_DIM])
{
	if (!targetPose)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	for (int i = 0; i < HIC_POSE_DIM; ++i)
	{
		targetPose_[i] = targetPose[i];
	}
	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		targetVelocity_[i] = 0.0;
	}
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::setOnlineTargetPose(
	const double targetPose[HIC_POSE_DIM],
	const double targetVelocity[HIC_CARTESIAN_DIM])
{
	if (!targetPose || !targetVelocity)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	for (int i = 0; i < HIC_POSE_DIM; ++i)
	{
		targetPose_[i] = targetPose[i];
	}
	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		targetVelocity_[i] = targetVelocity[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::computeJointTorque(
	const double currentPose[HIC_POSE_DIM],
	const double currentTwist[HIC_CARTESIAN_DIM],
	const double* jointPosition,
	const double* jointVelocity,
	const double* jacobianRowMajor,
	double* jointTorque)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!currentPose || !currentTwist || !jointPosition || !jointVelocity ||
	    !jacobianRowMajor || !jointTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	double cartesianError[HIC_CARTESIAN_DIM] = { 0.0 };
	double wrenchCommand[HIC_CARTESIAN_DIM] = { 0.0 };
	HicStatus status = computeCartesianError(currentPose, cartesianError);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = computeCartesianWrench(cartesianError, currentTwist, wrenchCommand);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		lastCartesianError_[i] = cartesianError[i];
		lastCartesianWrenchCommand_[i] = wrenchCommand[i];
	}

	Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > jacobian(
		jacobianRowMajor, 6, jointCount_);
	Eigen::Map<const Eigen::Matrix<double, 6, 1> > wrench(wrenchCommand);
	const Eigen::VectorXd torque = jacobian.transpose() * wrench;

	for (int i = 0; i < jointCount_; ++i)
	{
		jointTorque[i] = torque(i);
	}
	return applyNullspaceTorque(jointPosition, jointVelocity, jacobianRowMajor, jointTorque);
}

HicStatus HicCartesianImpedanceCore::getLastCartesianWrenchCommand(double wrenchCommand[HIC_CARTESIAN_DIM]) const
{
	if (!wrenchCommand)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		wrenchCommand[i] = lastCartesianWrenchCommand_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::getLastCartesianError(double cartesianError[HIC_CARTESIAN_DIM]) const
{
	if (!cartesianError)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		cartesianError[i] = lastCartesianError_[i];
	}
	return HIC_STATUS_OK;
}

void HicCartesianImpedanceCore::reset()
{
	HicImpedanceGains gains = {};
	gains_ = gains;
	HicNullspaceControlConfig nullspace = {};
	nullspaceConfig_ = nullspace;
	std::fill(targetVelocity_, targetVelocity_ + HIC_CARTESIAN_DIM, 0.0);
	std::fill(lastCartesianWrenchCommand_, lastCartesianWrenchCommand_ + HIC_CARTESIAN_DIM, 0.0);
	std::fill(lastCartesianError_, lastCartesianError_ + HIC_CARTESIAN_DIM, 0.0);
	setIdentityPose(targetPose_);
}

HicStatus HicCartesianImpedanceCore::computeCartesianError(
	const double currentPose[HIC_POSE_DIM],
	double cartesianError[HIC_CARTESIAN_DIM]) const
{
	if (!currentPose || !cartesianError)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < 3; ++i)
	{
		cartesianError[i] = targetPose_[i] - currentPose[i];
	}

	if (forceControlMode_ == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION)
	{
		cartesianError[3] = 0.0;
		cartesianError[4] = 0.0;
		cartesianError[5] = 0.0;
		return HIC_STATUS_OK;
	}

	// 姿态误差第一版集中在这里封装，当前使用相对四元数的角轴近似，
	// 后续可无缝替换为严格的 SO(3) log map。
	const Eigen::Quaterniond currentQuat = toQuaternion(currentPose);
	const Eigen::Quaterniond targetQuat = toQuaternion(targetPose_);
	const Eigen::Quaterniond deltaQuat = targetQuat * currentQuat.conjugate();
	Eigen::AngleAxisd deltaAxis(deltaQuat);
	const Eigen::Vector3d rotError = deltaAxis.axis() * deltaAxis.angle();
	cartesianError[3] = rotError.x();
	cartesianError[4] = rotError.y();
	cartesianError[5] = rotError.z();
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::computeCartesianWrench(
	const double cartesianError[HIC_CARTESIAN_DIM],
	const double currentTwist[HIC_CARTESIAN_DIM],
	double wrenchCommand[HIC_CARTESIAN_DIM]) const
{
	if (!cartesianError || !currentTwist || !wrenchCommand)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		const double velocityError = targetVelocity_[i] - currentTwist[i];
		if (forceControlMode_ == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION && i >= 3)
		{
			wrenchCommand[i] = 0.0;
			continue;
		}
		wrenchCommand[i] = gains_.stiffness[i] * cartesianError[i] + gains_.damping[i] * velocityError;
	}
	return HIC_STATUS_OK;
}

HicStatus HicCartesianImpedanceCore::applyNullspaceTorque(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jacobianRowMajor,
	double* jointTorque) const
{
	if (!nullspaceConfig_.enabled)
	{
		return HIC_STATUS_OK;
	}
	if (!jointPosition || !jointVelocity || !jacobianRowMajor || !jointTorque)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	const int taskDim = getTaskDimension();
	if (jointCount_ <= taskDim)
	{
		return HIC_STATUS_OK;
	}

	Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > fullJacobian(
		jacobianRowMajor, HIC_CARTESIAN_DIM, jointCount_);
	const Eigen::MatrixXd taskJacobian = fullJacobian.topRows(taskDim);
	const Eigen::MatrixXd jjT = taskJacobian * taskJacobian.transpose();
	const Eigen::MatrixXd damping = 1e-6 * Eigen::MatrixXd::Identity(taskDim, taskDim);
	const Eigen::MatrixXd jjTInv = (jjT + damping).inverse();
	const Eigen::MatrixXd projector =
		Eigen::MatrixXd::Identity(jointCount_, jointCount_) -
		taskJacobian.transpose() * jjTInv * taskJacobian;

	Eigen::VectorXd desiredJointTorque(jointCount_);
	for (int i = 0; i < jointCount_; ++i)
	{
		const double positionError = nullspaceConfig_.targetJointPosition[i] - jointPosition[i];
		const double dampingTerm = -nullspaceConfig_.damping[i] * jointVelocity[i];
		desiredJointTorque(i) = nullspaceConfig_.stiffness[i] * positionError + dampingTerm;
	}

	const Eigen::VectorXd nullspaceTorque = projector * desiredJointTorque;
	for (int i = 0; i < jointCount_; ++i)
	{
		jointTorque[i] += nullspaceTorque(i);
	}
	return HIC_STATUS_OK;
}

int HicCartesianImpedanceCore::getTaskDimension() const
{
	return (forceControlMode_ == HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION) ? 3 : 6;
}

} // namespace hic
