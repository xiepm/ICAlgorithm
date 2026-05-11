#pragma once

#include "hic_common_types.h"

#include <memory>

using namespace KDL;
using namespace Eigen;

const double EPSILON1 = 1.0e-6;
const double EPSILON2 = 1.0e-12;
const double INFINITE = 9.0e9;

class kinematicsBase
{
public:
	kinematicsBase();
	virtual ~kinematicsBase() = default;

	virtual void setRobotDHParameters(
		const EcRealVector& kinematcisParam) = 0;

	virtual void setJointMotionLimit(
		const EcRealVector& upperJointLimit,
		const EcRealVector& lowerJointLimit) = 0;

	virtual void setToolCoordinateSystem(
		const EcFrame& tool) = 0;

	virtual EcFrame forwardKinematics(
		const EcRealVector& jointPositions) = 0;

	virtual void forwardKinematics2(
		const EcRealVector& jointPositions,
		EcFrame& fkFrame) = 0;

	virtual int getRobotType() = 0;

	virtual ENInverseKineState inverseKinematics(
		const EcFrame& target,
		const EcRealVector& refJoint,
		EcRealVector& acs) = 0;

	virtual ENInverseKineState inverseKinematicsWithConfigHolding(
		const Frame& targetPose,
		const EcRealVector& refJoint,
		EcRealVector& targetJoint)
	{
		(void)targetPose;
		(void)refJoint;
		(void)targetJoint;
		return ikState_normal;
	}

	virtual void setIdentityErrorOfDHParams(const DHParams& errorDHParams)
	{
		(void)errorDHParams;
	}

	virtual void setReducerParams(
		const EcRealVectorVector& reducerParamsOfJoints,
		const EcReal& rigidityRatio)
	{
		(void)reducerParamsOfJoints;
		(void)rigidityRatio;
	}

	virtual void setCompensationStrategy(const int& strategy = 0)
	{
		(void)strategy;
	}

	virtual void updateFlexibilityError(const EcRealVector& torque)
	{
		(void)torque;
	}

	virtual EcReal getJacobianMatrix(
		const EcRealVector& jointPosition,
		Eigen::MatrixXd& Jacobian) = 0;

	virtual void getJacobianWithToolPointMatrix(
		const VectorXd& jointPosition,
		Eigen::MatrixXd& Jacobian) = 0;

	virtual void getJacobianDotWithToolMatrix(
		const VectorXd& jointPosition,
		const VectorXd& jointVel,
		Eigen::MatrixXd& JacobianDot) = 0;

	virtual EcReal calcJacobiandeterminant(const EcRealVector& jointPosition)
	{
		EcRealMatrixX Jacobian(6, 6);
		getJacobianMatrix(jointPosition, Jacobian);
		return Jacobian.determinant();
	}

	virtual EcReal getJacobianConditionNum(const EcRealVector& jointPosition)
	{
		EcRealMatrixX Jacobian(6, 6);
		getJacobianMatrix(jointPosition, Jacobian);
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jacobian);
		EcReal minSingularValue = svd.singularValues()(svd.singularValues().size() - 1);
		return (minSingularValue == 0) ? INFINITE : (svd.singularValues()(0) / minSingularValue);
	}

	EcBoolean checkSingularity(const EcRealVector& jointPosition);

	EcReal calcJacobianJointsVelocity(
		EcRealVector jointPosition,
		EcRealVector EndEffectorVelocity,
		EcRealVector& jointsVelocity);

	void calcJacobianEndEffectorVelocity(
		const EcRealVector& jointPosition,
		const EcRealVector& jointVelocity,
		EcRealVector& endEffectorVelocity);

	void calcJacobianJointTorque(
		const EcRealVector& jointPosition,
		const EcRealVector& endEffectorForce,
		EcRealVector& jointTorque);

	void calcSpatialVel2JointVel(
		const VectorXd& jointPosition,
		const VectorXd& spatialVel,
		VectorXd& jointVel);

	void calcSpatialAcc2JointAcc(
		const VectorXd& jointPosition,
		const VectorXd& spatialVel,
		const VectorXd& spatialAcc,
		VectorXd& jointVel,
		VectorXd& jointAcc);

	virtual void calcSpatialJerk2JointJerk(
		const EcReal deltaT,
		const VectorXd& jointPosition,
		const VectorXd& spatialVel,
		const VectorXd& spatialAcc,
		const VectorXd& spatialJerk,
		VectorXd& jointVel,
		VectorXd& jointAcc,
		VectorXd& jointJerk)
	{
		MatrixXd J(NUMOFJOINTS6, NUMOFJOINTS6), Jdot(NUMOFJOINTS6, NUMOFJOINTS6), Jdotdot(NUMOFJOINTS6, NUMOFJOINTS6);
		getJacobianWithToolPointMatrix(jointPosition, J);
		MatrixXd Jinv = J.inverse();
		jointVel = Jinv * spatialVel;

		getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
		jointAcc = Jinv * (spatialAcc - Jdot * jointVel);

		Jdotdot = (Jdot - m_preJdot) / deltaT;
		jointJerk = Jinv * (spatialJerk - Jdotdot * jointVel - 2 * Jdot * jointAcc);

		m_preJdot = Jdot;
	}

	void calcJointVel2SpatialVel(
		const VectorXd& jointPosition,
		const VectorXd& jointVel,
		VectorXd& spatialVel);

	void calcJointAcc2SpatialAcc(
		const VectorXd& jointPosition,
		const VectorXd& jointVel,
		const VectorXd& jointAcc,
		VectorXd& spatialVel,
		VectorXd& spatialAcc);

	void calcJointJerk2SpatialJerk(
		const EcReal deltaT,
		const VectorXd& jointPosition,
		const VectorXd& jointVel,
		const VectorXd& jointAcc,
		const VectorXd& jointJerk,
		VectorXd& spatialVel,
		VectorXd& spatialAcc,
		VectorXd& spatialJerk);

	Eigen::MatrixXd pseudoInverse(
		const Eigen::MatrixXd& Jacobian,
		double tolerance = 1e-6)
	{
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(
			Jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::MatrixXd singularValuesInv =
			Eigen::MatrixXd::Zero(Jacobian.cols(), Jacobian.rows());
		for (int i = 0; i < min(Jacobian.rows(), Jacobian.cols()); ++i)
		{
			singularValuesInv(i, i) =
				(svd.singularValues()(i) > tolerance) ? (1.0 / svd.singularValues()(i)) : 0.0;
		}
		return svd.matrixV() * singularValuesInv * svd.matrixU().transpose();
	}

private:
	Eigen::MatrixXd m_jacobian;
	Eigen::MatrixXd m_preJdot;
};

typedef std::shared_ptr<kinematicsBase> kinBasePtr;
