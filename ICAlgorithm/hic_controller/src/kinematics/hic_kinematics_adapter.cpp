#include "hic_controller/kinematics/hic_kinematics_adapter.h"

#include "hic_controller/kinematics/DSKinematics.h"
#include "hic_controller/kinematics/URKinematics.h"
#include "hic_controller/kinematics/elfinKinematics.h"
#include "hic_controller/kinematics/palletKinematics.h"

#include <algorithm>
#include <Eigen/Dense>
#include <memory>

namespace hic
{
namespace
{
kinBasePtr* backendPtr(void* raw)
{
	return static_cast<kinBasePtr*>(raw);
}

const kinBasePtr* backendPtrConst(const void* raw)
{
	return static_cast<const kinBasePtr*>(raw);
}

kinBasePtr createBackend(int robotType)
{
	switch (robotType)
	{
	default:
	case 0:
	case 2:
	case 7:
	case 9:
		return kinBasePtr(new elfinKinematics());
	case 1:
	case 3:
	case 8:
	case 10:
		return kinBasePtr(new URKinematics());
	case 5:
		return kinBasePtr(new palletKinematics());
	case 12:
		return kinBasePtr(new DSKinematics());
	case 20:
		// 按 robotType 分发规则，7轴机型当前在运动学侧绑定到 DSKinematics。
		// 这样可与旧工程的 m_kinBase.reset(new DSKinematics) 行为保持一致。
		return kinBasePtr(new DSKinematics());
	}
}
}

HicKinematicsAdapter::HicKinematicsAdapter()
	: robotType_(0),
	  jointCount_(0),
	  initialized_(false),
	  useMockModel_(false),
	  backend_(0)
{
}

HicKinematicsAdapter::~HicKinematicsAdapter()
{
	reset();
}

HicStatus HicKinematicsAdapter::initialize(
	int robotType,
	int jointCount,
	const double* kinematicParams)
{
	if (!kinematicParams || jointCount <= 0 || jointCount > HIC_MAX_JOINTS)
	{
		return HIC_STATUS_ERROR_INVALID_PARAM;
	}

	reset();

	backend_ = new kinBasePtr(createBackend(robotType));
	if (!backendPtr(backend_)->get())
	{
		reset();
		return HIC_STATUS_ERROR_INIT;
	}

	EcRealVector params(HIC_MAX_JOINTS, 0.0);
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		params[i] = kinematicParams[i];
	}
	(*backendPtr(backend_))->setRobotDHParameters(params);

	robotType_ = robotType;
	jointCount_ = jointCount;
	initialized_ = true;
	useMockModel_ = false;
	return HIC_STATUS_OK;
}

HicStatus HicKinematicsAdapter::computeForwardKinematics(
	const double* jointPosition,
	double pose[HIC_POSE_DIM])
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !pose)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	if (useMockModel_)
	{
		// 占位 FK：位置由前 3 个关节做线性映射，姿态固定为单位四元数。
		pose[0] = 0.30 + 0.05 * jointPosition[0];
		pose[1] = -0.10 + 0.05 * jointPosition[1];
		pose[2] = 0.40 + 0.04 * jointPosition[2];
		pose[3] = 1.0;
		pose[4] = 0.0;
		pose[5] = 0.0;
		pose[6] = 0.0;
		return HIC_STATUS_OK;
	}

	EcRealVector q(jointCount_, 0.0);
	for (int i = 0; i < jointCount_; ++i)
	{
		q[i] = jointPosition[i];
	}

	const EcFrame frame = (*backendPtr(backend_))->forwardKinematics(q);
	double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
	frame.M.GetQuaternion(qx, qy, qz, qw);

	pose[0] = frame.p[0];
	pose[1] = frame.p[1];
	pose[2] = frame.p[2];
	pose[3] = qw;
	pose[4] = qx;
	pose[5] = qy;
	pose[6] = qz;
	return HIC_STATUS_OK;
}

HicStatus HicKinematicsAdapter::computeJacobian(
	const double* jointPosition,
	double* jacobianRowMajor)
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !jacobianRowMajor)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	if (useMockModel_)
	{
		std::fill(jacobianRowMajor, jacobianRowMajor + HIC_CARTESIAN_DIM * jointCount_, 0.0);
		// 占位 Jacobian：前三轴映射平移，后 3 轴映射角速度，第 7 轴给一个较小耦合项。
		jacobianRowMajor[0 * jointCount_ + 0] = 0.05;
		jacobianRowMajor[1 * jointCount_ + 1] = 0.05;
		jacobianRowMajor[2 * jointCount_ + 2] = 0.04;
		jacobianRowMajor[3 * jointCount_ + 3] = 1.0;
		jacobianRowMajor[4 * jointCount_ + 4] = 1.0;
		jacobianRowMajor[5 * jointCount_ + 5] = 1.0;
		jacobianRowMajor[2 * jointCount_ + 6] = 0.01;
		jacobianRowMajor[5 * jointCount_ + 6] = 0.10;
		return HIC_STATUS_OK;
	}

	EcRealVector q(jointCount_, 0.0);
	for (int i = 0; i < jointCount_; ++i)
	{
		q[i] = jointPosition[i];
	}

	Eigen::MatrixXd jacobian;
	(*backendPtr(backend_))->getJacobianMatrix(q, jacobian);
	if (jacobian.rows() != HIC_CARTESIAN_DIM || jacobian.cols() != jointCount_)
	{
		for (int i = 0; i < HIC_CARTESIAN_DIM * jointCount_; ++i)
		{
			jacobianRowMajor[i] = 0.0;
		}
		return HIC_STATUS_ERROR_KINEMATICS;
	}

	for (int r = 0; r < HIC_CARTESIAN_DIM; ++r)
	{
		for (int c = 0; c < jointCount_; ++c)
		{
			jacobianRowMajor[r * jointCount_ + c] = jacobian(r, c);
		}
	}
	return HIC_STATUS_OK;
}

HicStatus HicKinematicsAdapter::computeEndEffectorTwist(
	const double* jointPosition,
	const double* jointVelocity,
	double twist[HIC_CARTESIAN_DIM])
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!jointPosition || !jointVelocity || !twist)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	if (useMockModel_)
	{
		double jacobianRowMajor[HIC_MAX_JACOBIAN_SIZE] = { 0.0 };
		computeJacobian(jointPosition, jacobianRowMajor);
		for (int row = 0; row < HIC_CARTESIAN_DIM; ++row)
		{
			twist[row] = 0.0;
			for (int col = 0; col < jointCount_; ++col)
			{
				twist[row] += jacobianRowMajor[row * jointCount_ + col] * jointVelocity[col];
			}
		}
		return HIC_STATUS_OK;
	}

	EcRealVector q(jointCount_, 0.0), dq(jointCount_, 0.0), eeTwist;
	for (int i = 0; i < jointCount_; ++i)
	{
		q[i] = jointPosition[i];
		dq[i] = jointVelocity[i];
	}

	(*backendPtr(backend_))->calcJacobianEndEffectorVelocity(q, dq, eeTwist);
	if (eeTwist.size() != HIC_CARTESIAN_DIM)
	{
		for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
		{
			twist[i] = 0.0;
		}
		return HIC_STATUS_ERROR_KINEMATICS;
	}

	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		twist[i] = eeTwist[i];
	}
	return HIC_STATUS_OK;
}

void HicKinematicsAdapter::reset()
{
	if (backend_)
	{
		delete backendPtr(backend_);
		backend_ = 0;
	}
	initialized_ = false;
	useMockModel_ = false;
	robotType_ = 0;
	jointCount_ = 0;
}

} // namespace hic
