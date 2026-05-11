/*
**  1. 完成正逆解计算；
**  2. 雅可比矩阵；
**  3. 逆雅可比矩阵，已知关节速度求空间速度；
*/
#pragma once
#include "hic_common_types.h"
#include "hic_frames.hpp"
#include "hic_controller/kinematics/hic_kinematics_base.h"
//#include "hic_common_constants.h"



using namespace KDL;
using namespace Eigen;



class elfinKinematics : public kinematicsBase
{
public:
	elfinKinematics();
	virtual ~elfinKinematics();

	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	virtual void setJointMotionLimit
	(
		const EcRealVector& upperJointLimit,
		const EcRealVector& lowerJointLimit
	);

	virtual void setToolCoordinateSystem
	(
		const EcFrame& tool
	);

	virtual Frame forwardKinematics
	(
		const EcRealVector& jointPositions
	);

	virtual void forwardKinematics2
	(
		const EcRealVector& jointPositions,
		EcFrame& fkFrame
	);


	virtual ENInverseKineState inverseKinematics
	(
		const Frame& target,
		const EcRealVector& current,
		EcRealVector& acs
	);

	// 根据臂型选择逆解; refJoint作用: 指定臂型，且奇异时提供q1/q4的参考值
	virtual ENInverseKineState inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint);

	virtual EcReal getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian);

	// 计算Tool Point的Jacobian；
	virtual void getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian);

	// 计算Tool Point的JacobianDot；
	virtual void getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot);

	virtual int getRobotType() { return  0; };

private:
	virtual void elfinDHParameters();
	int mk2Solve4Theta23(const double& q1, Vector& pw_vector, double q2[2], double q3[2]);
	int mk2Solve4wrist(double q[6], Frame& T, const int& wrist);
	int normalize(const EcRealVector& current, double q[][6], EcRealVectorVector& qVector); // 限制关节至关节极限任意设置的多圈范围
	double vectorMultiplyOperator(const Vector& v1, const Vector& v2);
	Vector vectorCrossOperator(const Vector& v1, const Vector& v2);
	EcBoolean isInJointLimit(const EcReal& jointPosition, const int& orderNum)  // 判定关节角是否满足其对应的关节极限
	{
		return ((jointPosition < m_upperJointLimit[orderNum]) && (jointPosition > m_lowerJointLimit[orderNum]));
	};

	void configuration(const EcRealVector& refJoint, int globalConf[3]); // 确定当前的臂型
	int normalizePI(double q[NUMOFJOINTS6]); // 限制关节至(-PI,PI]
	int normalize4ConfigHolding(const EcRealVector& current, double q[NUMOFJOINTS6], EcRealVector& qVector);

private:
	EN_RobotDH      m_mk2DH;
	EcReal			m_a2, m_d1, m_d4, m_d6;
	EcReal			m_compTheta2, m_compTheta3;


	EcRealMatrixX			m_jacobian;
	EcReal					m_determinant;
	EcRealVector			m_upperJointLimit;
	EcRealVector			m_lowerJointLimit;
	EcFrame					m_toolFrame;					// 对于这个全局变量，可能需要加锁，避免运动时计算出错；

	std::vector<KDL::Vector> m_zAxis, m_pAxis, m_velAxis;

	EcReal					m_preJacDet;
	EcRealVector			m_preJointPosition;


	EcVectorVector			m_pdotAxis;
	EcVectorVector			m_omega;

};

