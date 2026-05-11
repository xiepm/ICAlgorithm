#include "hic_controller/kinematics/hic_kinematics_base.h"


kinematicsBase::kinematicsBase()
{

}


EcBoolean kinematicsBase::checkSingularity(const EcRealVector& jointPosition)
{
	// add singularity recognition, compare the determinant of current and solved joint position;
	EcReal det_current = calcJacobiandeterminant(jointPosition);
	EcBoolean isSingularity = false;
	if (fabs(det_current) < 0.001)
	{
		isSingularity = true;
	}

	return isSingularity;
}

/*
EcReal kinematicsBase::calcJacobiandeterminant(const EcRealVector& jointPosition)
{
	EcRealMatrixX Jacobian(6, 6);
	getJacobianMatrix(jointPosition, Jacobian);
	return Jacobian.determinant();

}
*/


EcReal kinematicsBase::calcJacobianJointsVelocity(EcRealVector jointPosition, EcRealVector EndEffectorVelocity, EcRealVector& jointsVelocity)
{
	int n = jointPosition.size();
	Eigen::VectorXd vJointPosition = VectorXd::Map(&jointPosition[0], n);
	getJacobianWithToolPointMatrix(vJointPosition, m_jacobian);
	VectorXd jointsVel(n), eeVel(6);
	eeVel << EndEffectorVelocity[0], EndEffectorVelocity[1], EndEffectorVelocity[2], EndEffectorVelocity[3], EndEffectorVelocity[4], EndEffectorVelocity[5];
	EcReal det = 1;
	int robotType = getRobotType();
	if (robotType != 5 && n == 6)
	{
		det = m_jacobian.determinant();
		if (fabs(det) > 0.001)
			jointsVel = m_jacobian.inverse() * eeVel;
		
		else
			jointsVel.setZero();			//λãĬùؽٶΪ0
		for (int i = 0; i < 6; i++)
			jointsVelocity[i] = jointsVel[i];
	}
	else if (robotType == 5)
	{
		VectorXd jointsVel2(5);
		MatrixXd J = (m_jacobian.transpose() * m_jacobian);
		det = J.determinant();
		if(fabs(det)>0.001)
			jointsVel2 = J.inverse()*m_jacobian.transpose() * eeVel;
		else
			jointsVel2.setZero();

		for (int i = 0; i < 5; i++)
			jointsVelocity[i] = jointsVel2[i];
	}
	else
	{
		// Generalized pseudo-inverse for redundant or other robots
		MatrixXd Jinv = pseudoInverse(m_jacobian);
		jointsVel = Jinv * eeVel;
		jointsVelocity.assign(n, 0.0);
		for (int i = 0; i < n; i++)
			jointsVelocity[i] = jointsVel[i];
		det = (m_jacobian * m_jacobian.transpose()).determinant(); // Measure of manipulability
	}

	return det;
}

void kinematicsBase::calcJacobianEndEffectorVelocity(const EcRealVector& jointPosition, const EcRealVector& jointVelocity, EcRealVector& endEffectorVelocity)
{
	int n = jointPosition.size();
	Eigen::VectorXd vJointPosition = VectorXd::Map(&jointPosition[0], n);
	getJacobianWithToolPointMatrix(vJointPosition, m_jacobian);
	VectorXd jointsVel = VectorXd::Map(&jointVelocity[0], jointVelocity.size());
	VectorXd eeVel = m_jacobian * jointsVel;
	
	endEffectorVelocity.assign(6, 0.0);
	for (int i = 0; i < 6; i++)
	{
		endEffectorVelocity[i] = eeVel[i];
	}

}

void kinematicsBase::calcJacobianJointTorque(const EcRealVector& jointPosition, const EcRealVector& endEffectorForce, EcRealVector& jointTorque)
{
	int n = jointPosition.size();
	Eigen::VectorXd vJointPosition = VectorXd::Map(&jointPosition[0], n);
	getJacobianWithToolPointMatrix(vJointPosition, m_jacobian);
	
	VectorXd eeForce(6);
	eeForce << endEffectorForce[0], endEffectorForce[1], endEffectorForce[2], endEffectorForce[3], endEffectorForce[4], endEffectorForce[5];
	
	VectorXd torque = m_jacobian.transpose() * eeForce;

	jointTorque.assign(n, 0.0);
	for (int i = 0; i < n; i++)
	{
		jointTorque[i] = torque[i];
	}
}



void kinematicsBase::calcSpatialVel2JointVel(const VectorXd& jointPosition, const VectorXd& spatialVel, VectorXd& jointVel)
{
	getJacobianWithToolPointMatrix(jointPosition, m_jacobian);
	if (jointPosition.size() == 6)
		jointVel = m_jacobian.inverse() * spatialVel;
	else
		jointVel = pseudoInverse(m_jacobian) * spatialVel;

}

void kinematicsBase::calcSpatialAcc2JointAcc(const VectorXd& jointPosition, const VectorXd& spatialVel, const VectorXd& spatialAcc, VectorXd& jointVel, VectorXd& jointAcc)
{
	getJacobianWithToolPointMatrix(jointPosition, m_jacobian);
	MatrixXd Jinv = (jointPosition.size() == 6) ? m_jacobian.inverse().eval() : pseudoInverse(m_jacobian).eval();

	jointVel = Jinv * spatialVel;

	MatrixXd Jdot;
	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	jointAcc = Jinv * (spatialAcc - Jdot * jointVel);
}


void kinematicsBase::calcJointVel2SpatialVel(const VectorXd& jointPosition, const VectorXd& jointVel,
	VectorXd& spatialVel)
{
	MatrixXd J;
	getJacobianWithToolPointMatrix(jointPosition, J);
	spatialVel = J * jointVel;

}

void kinematicsBase::calcJointAcc2SpatialAcc(const VectorXd& jointPosition, const VectorXd& jointVel, const VectorXd& jointAcc,
	VectorXd& spatialVel, VectorXd& spatialAcc)
{
	MatrixXd J, Jdot;
	getJacobianWithToolPointMatrix(jointPosition, J);
	spatialVel = J * jointVel;

	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	spatialAcc = Jdot * jointVel + J * jointAcc;
}

void kinematicsBase::calcJointJerk2SpatialJerk(const EcReal deltaT, const VectorXd& jointPosition, const VectorXd& jointVel, const VectorXd& jointAcc, const VectorXd& jointJerk,
	VectorXd& spatialVel, VectorXd& spatialAcc, VectorXd& spatialJerk)
{
	MatrixXd J, Jdot, Jdotdot;
	getJacobianWithToolPointMatrix(jointPosition, J);
	spatialVel = J * jointVel;

	getJacobianDotWithToolMatrix(jointPosition, jointVel, Jdot);
	spatialAcc = Jdot * jointVel + J * jointAcc;

	Jdotdot = (Jdot - m_preJdot) / deltaT;
	spatialJerk = Jdotdot * spatialVel + 2 * Jdot * spatialAcc + J * spatialJerk;

	m_preJdot = Jdot;

}
