#pragma once
#include "hic_common_types.h"
#include "hic_frames.hpp"
#include "hic_controller/kinematics/hic_kinematics_base.h"

using namespace KDL;
using namespace Eigen;

const EcReal     dShoulderSingularityS05 = 0.053;  // S05接近肩部奇异的边界值

class URKinematics : public kinematicsBase
{
public:
	URKinematics();

	virtual ~URKinematics();

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

	// 根据臂型选择逆解; refJoint作用: 指定臂型
	virtual ENInverseKineState inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint);

	virtual EcReal getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian);


	// 计算Tool Point的Jacobian；
	virtual void getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian);

	// 计算Tool Point的JacobianDot；
	virtual void getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot);

	virtual int getRobotType() { return  1; };

private:
	EN_RobotDH m_mk2DH;
	virtual void URDHParameters();
	Vector vectorCrossOperator(const Vector& v1, const Vector& v2);
	int normalize(const EcRealVector& current, double q[][6], EcRealVectorVector& qVector); // 考虑关节极限任意设置的多圈情况
	EcBoolean isInJointLimit(const EcReal& jointPosition, const int& orderNum)  // 判定关节角是否满足其对应的关节极限
	{
		return ((jointPosition < m_upperJointLimit[orderNum]) && (jointPosition > m_lowerJointLimit[orderNum]));
	};

	EcBoolean isCloseShoulderSingularity(const EcRealVector& current)  // 判定当前机械臂是否接近肩部奇异
	{
		EcReal d = m_a2 * KDL::cos(current[1]) + m_a3 * KDL::cos(current[1] + current[2]) + m_d5 * KDL::sin(current[1] + current[2] + current[3]); // 5/6轴交点到1/2轴平面的距离
		return (fabs(d) <= dShoulderSingularityS05);
	};

	void configuration(const EcRealVector& refJoint, int globalConf[3]); // 确定当前的臂型
	int normalize4ConfigHolding(const EcRealVector& current, double q[NUMOFJOINTS6], EcRealVector& qVector);
private:
	EcReal					m_a2, m_a3, m_d1, m_d4, m_d5, m_d6;
	EcRealVector			m_upperJointLimit;
	EcRealVector			m_lowerJointLimit;
	EcFrame					m_toolFrame;					// 对于这个全局变量，可能需要加锁，避免运动时计算出错；


	std::vector<KDL::Vector> m_zAxis, m_pAxis, m_velAxis;
	EcVectorVector			m_pdotAxis;
	EcVectorVector			m_omega;
};

