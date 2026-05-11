#pragma once
#include "hic_controller/kinematics/hic_kinematics_base.h"


using namespace KDL;
using namespace Eigen;

class DSKinematics : public kinematicsBase
{
public:
	DSKinematics();
	virtual ~DSKinematics();

	/**
	 * @brief 设置机械臂的DH参数.
	 * 
	 * @param kinematcisParam E系列的DH参数: [d1, d4, d6, a2]
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
	 */
	virtual ENInverseKineState inverseKinematics
	(
		const Frame& targetPose,
		const EcRealVector& refJoint,
		EcRealVector& targetJoint
	);

	/**
	 * @brief 机械臂的逆运动学(基于臂型一致原则选解).
	 * 
	 * @param targetPose Tool系在Base系的位姿
	 * @param refJoint 关节位置参考值
	 * @param targetJoint 逆解结果
	 * @return 逆解状态
	 */
	virtual ENInverseKineState inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint);

	/**
	 * @brief 获取机械臂的雅克比矩阵.
	 * 
	 * @param jointPosition 关节位置矢量
	 * @param Jacobian 6×6的雅克比矩阵
	 * @return 雅克比行列式
	 */
	virtual EcReal getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian);

	/**
	 * @brief 获取机械臂Arm部分的雅克比矩阵.
	 *
	 * @param jointPosition 关节位置矢量
	 * @return 3×3的雅克比矩阵
	 */
	virtual Eigen::MatrixXd getJacobianMatrixOfArm(const EcRealVector& jointPosition);

	/**
	 * @brief 获取机械臂Wrist部分的雅克比矩阵.
	 *
	 * @param jointPosition 关节位置矢量
	 * @return 3×3的雅克比矩阵
	 */
	virtual Eigen::MatrixXd getJacobianMatrixOfWrist(const EcRealVector& jointPosition);

	// 计算Tool Point的Jacobian；
	virtual void getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian);

	// 计算Tool Point的JacobianDot；
	virtual void getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot);

	virtual int getRobotType() { return  12; };

private:
	virtual void DSDHParameters();
	double vectorMultiplyOperator(const Vector& v1, const Vector& v2);
	Vector vectorCrossOperator(const Vector& v1, const Vector& v2);
	EcBoolean isInJointLimit(const EcReal& jointPosition, const int& orderNum)  // 判定关节角是否满足其对应的关节极限
	{
		return ((jointPosition < m_upperJointLimit[orderNum]) && (jointPosition > m_lowerJointLimit[orderNum]));
	};

	void configuration(const EcRealVector& wrappedJoint, int globalConf[3]); // 确定当前的臂型
	void normalize4ConfigHolding(const EcRealVector& refJoint, const EcRealVector& wrappedJoint, EcRealVector& normalizedJoint);
	void wrapToPi(const EcRealVector& refJoint, EcRealVector& wrappedJoint); // 关节角映射至[-PI,PI]

private:
	EN_RobotDH      m_mk2DH;
	EcReal			m_a2, m_d1, m_d4, m_d6;


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

