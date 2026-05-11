#pragma once
#include "hic_controller/kinematics/hic_kinematics_base.h"


using namespace KDL;
using namespace Eigen;

class palletKinematics : public kinematicsBase
{
public:
	palletKinematics();
	virtual ~palletKinematics();

	/**
	 * @brief 设置机械臂的DH参数.
	 * 
	 * @param kinematcisParam 5DOF的DH参数: [d1 d2 d3 d4 d5 d6 a2 a3]
	 */
	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	/**
	 * @brief 设置机械臂的关节极限.
	 * 
	 * @param upperJointLimit 关节上极限
	 * @param lowerJointLimit 关节下极限
	 * 
	 * @note 对于入参upperJointLimit及lowerJointLimit，仅前5个元素有效
	 */
	virtual void setJointMotionLimit
	(
		const EcRealVector& upperJointLimit,
		const EcRealVector& lowerJointLimit
	);

	/**
	 * @brief 设置Tool系.
	 *
	 * @param tool Tool系在Flange系的安装位姿
	 */
	virtual void setToolCoordinateSystem
	(
		const EcFrame& tool
	);

	/**
	 * @brief 机械臂的正运动学.
	 *
	 * @param jointPositions 关节位置矢量
	 * @return Tool系在Base系的位姿
	 * 
	 * @note 对于入参jointPositions，仅前5个元素有效
	 */
	virtual Frame forwardKinematics
	(
		const EcRealVector& jointPositions
	);

	/**
	 * @brief 机械臂的正运动学.
	 *
	 * @param jointPositions 关节位置矢量
	 * @param fkFrame Tool系在Base系的位姿
	 * 
	 * @note 对于入参jointPositions，仅前5个元素有效
	 */
	virtual void forwardKinematics2
	(
		const EcRealVector& jointPositions,
		EcFrame& fkFrame
	);

	/**
	 * @brief 机械臂的逆运动学(基于邻近原则选解).
	 *
	 * @param targetPose Tool系在Base系的位姿
	 * @param refJoint 关节位置参考值
	 * @param targetJoint 逆解结果
	 * @return 逆解状态
	 * 
	 * @note 对于入参refJoint及targetJoint，仅前5个元素有效
	 */
	virtual ENInverseKineState inverseKinematics
	(
		const Frame& targetPose,
		const EcRealVector& refJoint,
		EcRealVector& targetJoint
	);

	/**
	 * @brief 获取机械臂的雅克比矩阵.
	 *
	 * @param jointPosition 关节位置矢量
	 * @param Jacobian 5×5的Gramian矩阵
	 * @return Gramian矩阵的行列式的平方根
	 */
	virtual EcReal getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian);

	/**
	 * @brief 获取机械臂的雅克比条件数.
	 *
	 * @param jointPosition 关节位置矢量
	 * @return Gramian矩阵的条件数的平方根
	 */
	virtual EcReal getJacobianConditionNum(const EcRealVector& jointPosition);

	// 计算Tool Point的Jacobian；
	virtual void getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian);

	// 计算Tool Point的JacobianDot；
	virtual void getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot);

	virtual int getRobotType() { return  5; };

private:
	virtual void palletDHParameters();
	Vector vectorCrossOperator(const Vector& v1, const Vector& v2);
	void normalize(const EcRealVector& refJoint, EcReal qArray[][NUMOFJOINTS5], EcRealVectorVector& qVector); // 考虑关节极限任意设置的多圈情况
	EcBoolean isInJointLimit(const EcReal& jointPosition, const int& orderNum)  // 判定关节角是否满足其对应的关节极限
	{
		return ((jointPosition < m_upperJointLimit[orderNum]) && (jointPosition > m_lowerJointLimit[orderNum]));
	};

private:
	EN_RobotDH              m_mk2DH;  // 保留6DOF的DH结构，但连杆6的参数置零且不使用
	EcReal					m_a2, m_a3, m_d1, m_d4, m_d5, m_d6;  // m_d6保留，但未使用
	EcRealVector			m_upperJointLimit;
	EcRealVector			m_lowerJointLimit;
	EcFrame					m_toolFrame;					// 对于这个全局变量，可能需要加锁，避免运动时计算出错；


	std::vector<KDL::Vector> m_zAxis, m_pAxis, m_velAxis;
	EcVectorVector			m_pdotAxis;
	EcVectorVector			m_omega;
};

