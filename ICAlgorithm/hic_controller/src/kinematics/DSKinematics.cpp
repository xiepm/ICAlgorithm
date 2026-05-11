#include "hic_controller/kinematics/DSKinematics.h"
#include <iostream>


DSKinematics::DSKinematics()
{
	m_d1 = 0.2625;
	m_d4 = 0.6800;
	m_d6 = 0.1345;
	m_a2 = 0.6200;
	DSDHParameters();
	m_jacobian = EcRealMatrixX(6, 6);

	m_upperJointLimit.assign(6, 200 * PI);
	m_lowerJointLimit.assign(6, -200 * PI);
	m_toolFrame = Frame::Identity();

	m_preJacDet = 1.0;
	m_preJointPosition.assign(6, 0.0);

	KDL::Vector v;
	m_zAxis.assign(NUMOFJOINTS6 + 1, v);
	m_pAxis.assign(NUMOFJOINTS6 + 1, v);
	m_velAxis.assign(NUMOFJOINTS6 + 1, v);

	m_pdotAxis.assign(NUMOFJOINTS6 + 1, v);
	m_omega.assign(NUMOFJOINTS6 + 1, v);
}

DSKinematics::~DSKinematics()
{

}

void DSKinematics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

	DSDHParameters();
}

void DSKinematics::setJointMotionLimit
(
	const EcRealVector& upperJointLimit,
	const EcRealVector& lowerJointLimit
)
{
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		m_upperJointLimit[i] = upperJointLimit[i] + epsilon;
		m_lowerJointLimit[i] = lowerJointLimit[i] - epsilon;
	}
	//std::cout << "joint motion limit:" << upperJointLimit[0] << "," << upperJointLimit[1] << "," << upperJointLimit[2] << "," << upperJointLimit[3] << "," << upperJointLimit[4] << "," << upperJointLimit[5] << std::endl;
}

void DSKinematics::DSDHParameters()
{
	//EN_RobotDH dh;

	m_mk2DH.theta[0] = 0;
	m_mk2DH.theta[1] = -PI / 2;
	m_mk2DH.theta[2] = PI / 2;
	m_mk2DH.theta[3] = 0;
	m_mk2DH.theta[4] = 0;
	m_mk2DH.theta[5] = 0;

	m_mk2DH.d[0] = m_d1;
	m_mk2DH.d[1] = 0;
	m_mk2DH.d[2] = 0;
	m_mk2DH.d[3] = m_d4;
	m_mk2DH.d[4] = 0;
	m_mk2DH.d[5] = m_d6;

	m_mk2DH.a[0] = 0;
	m_mk2DH.a[1] = m_a2;
	m_mk2DH.a[2] = 0;
	m_mk2DH.a[3] = 0;
	m_mk2DH.a[4] = 0;
	m_mk2DH.a[5] = 0;

	m_mk2DH.alpha[0] = -PI / 2;
	m_mk2DH.alpha[1] = 0;
	m_mk2DH.alpha[2] = PI / 2;
	m_mk2DH.alpha[3] = -PI / 2;
	m_mk2DH.alpha[4] = PI / 2;
	m_mk2DH.alpha[5] = 0;
}

double DSKinematics::vectorMultiplyOperator(const Vector& v1, const Vector& v2)
{
	return (v1(0) * v2(0) + v1(1) * v2(1) + v1(2) * v2(2));
}

Vector DSKinematics::vectorCrossOperator(const Vector& v1, const Vector& v2)
{
	return{ v1(1) * v2(2) - v2(1) * v1(2), v1(2) * v2(0) - v1(0) * v2(2), v1(0) * v2(1) - v2(0) * v1(1) };
}

void DSKinematics::setToolCoordinateSystem
(
	const EcFrame& tool
)
{
	m_toolFrame = tool;
}

Frame DSKinematics::forwardKinematics(const EcRealVector& jointPositions)
{
	Frame T;
	double calcAcs[6];
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPositions[i] + m_mk2DH.theta[i];
	}

	for (int i = 0; i < 6; i++)
	{
		T = T * Frame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], calcAcs[i]);
	}
	T = T * m_toolFrame;
	return T;
}

void DSKinematics::forwardKinematics2(const EcRealVector& q, EcFrame& fkFrame)
{
	fkFrame = forwardKinematics(q);
}

ENInverseKineState DSKinematics::inverseKinematics(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint)
{
	return inverseKinematicsWithConfigHolding(targetPose, refJoint,targetJoint);
}

ENInverseKineState DSKinematics::inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint)
{
	targetJoint = refJoint;

	// 预处理关节角
	EcRealVector wrappedRefJoint(6, 0.0);
	wrapToPi(refJoint, wrappedRefJoint);

	// 获取目标位姿
	Frame T = targetPose * m_toolFrame.Inverse();
	EcReal nx = T(0, 0);
	EcReal ny = T(1, 0);
	EcReal nz = T(2, 0);
	EcReal ox = T(0, 1);
	EcReal oy = T(1, 1);
	EcReal oz = T(2, 1);
	EcReal ax = T(0, 2);
	EcReal ay = T(1, 2);
	EcReal az = T(2, 2);
	EcReal px = T(0, 3);
	EcReal py = T(1, 3);
	EcReal pz = T(2, 3);

	// 获取参考臂型
	int globalConf[3];
	configuration(wrappedRefJoint,globalConf);

	// 计算IK
	EcReal wx = px - ax * m_d6;
	EcReal wy = py - ay * m_d6;
	EcReal wz = pz - az * m_d6;
	// q1
	EcReal q1;
	if ((wx * wx + wy * wy) < EPSILON2)  // 肩部奇异
	{
		std::cout << "Shoulder Singularity." << std::endl;
		q1 = wrappedRefJoint[0];
	}
	else
	{
		q1 = std::atan2(wy, wx) + (1 - globalConf[0]) / 2 * PI;
	}
	// q2/q3
	EcReal q2;
	EcReal q3;
	EcReal r = KDL::sqrt(wx * wx + wy * wy + (wz - m_d1) * (wz - m_d1));
	if (r > m_d4 + m_a2 || r <= fabs(m_d4 - m_a2)) // 超出工作空间
	{
		std::cout << "this pose is ouside of workspace." << std::endl;
		return ikState_noSolution;
	}
	EcReal beta = std::atan2(KDL::sqrt(wx * wx + wy * wy), wz - m_d1);
	EcReal cTriangle = (m_a2 * m_a2 + m_d4 * m_d4 - r * r) / (2 * m_a2 * m_d4); // 三角形内角余弦
	if (fabs(cTriangle - 1) < EPSILON2) // 肘部平角奇异(完备性待确认)
	{
		std::cout << "Elbow Singularity." << std::endl;
		q2 = globalConf[0] * beta;
		q3 = PI;
	}
	else if (fabs(cTriangle + 1) < EPSILON2) // 肘部零角奇异
	{
		std::cout << "Elbow Singularity." << std::endl;
		q2 = globalConf[0] * beta;
		q3 = 0;
	}
	else
	{
		EcReal gama = std::acos((m_a2 * m_a2 + r * r - m_d4 * m_d4) / (2 * m_a2 * r));
		q2 = globalConf[0] * beta - globalConf[1] * gama;
		q3 = globalConf[1] * (PI - std::acos(cTriangle));
	}
	// q5
	EcReal c5 = (ax * std::cos(q1) + ay * std::sin(q1)) * std::sin(q2 + q3) + az * std::cos(q2 + q3);
	if (fabs(c5 - 1) < EPSILON2)
	{
		c5 = 1;
	}
	else if (fabs(c5 + 1) < EPSILON2)
	{
		c5 = -1;
	}
	EcReal q5 = globalConf[2] * std::acos(c5);
	// q4/q6
	EcReal q4;
	EcReal q6;
	if (fabs(std::sin(q5)) < EPSILON1) // 腕部奇异时
	{
		std::cout << "Wrist Singularity." << std::endl;
		EcReal s46 = ny * std::cos(q1) - nx * std::sin(q1);
		EcReal c46 = oy * std::cos(q1) - ox * std::sin(q1);
		q4 = wrappedRefJoint[3];
		q6 = std::atan2(s46, c46) - std::cos(q5)*q4;
	}
	else
	{
		EcReal k1 = ay * std::cos(q1) - ax * std::sin(q1);
		EcReal k2 = (ax * std::cos(q1) + ay * std::sin(q1)) * std::cos(q2 + q3) - az * std::sin(q2 + q3);
		q4 = std::atan2(k1 * std::sin(q5), k2 * std::sin(q5));
		EcReal k3 = - ((ox * std::cos(q1) + oy * std::sin(q1)) * std::sin(q2 + q3) + oz * std::cos(q2 + q3));
		EcReal k4 = - ((nx * std::cos(q1) + ny * std::sin(q1)) * std::sin(q2 + q3) + nz * std::cos(q2 + q3));
		q6 = std::atan2(-k3 * std::sin(q5), k4 * std::sin(q5));
	}

	// 预处理关节角
	EcRealVector qIK = {q1, q2, q3, q4, q5, q6 };  // 同时考虑逆解表达式的约束及奇异时保持关节角不变的情形
	EcRealVector wrappedIKJoint(6, 0.0);
	wrapToPi(qIK, wrappedIKJoint);
	normalize4ConfigHolding(refJoint, wrappedIKJoint, targetJoint);

	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		if ((targetJoint[i] < m_lowerJointLimit[i]) || (targetJoint[i] > m_upperJointLimit[i]))
		{
			std::cout << "Out of Limit." << std::endl;
			return ikState_outofLimit; // 后续可替换为用inverseKinematics重新逆解
		}
	}

	// 逆解连续性判据(待商榷)
	EcReal maxDisplacement = PI / 6;  // 暂无理论根据
	EcReal sumDisplacement = 0;
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		sumDisplacement += fabs(targetJoint[i] - refJoint[i]);
	}
	if (sumDisplacement > maxDisplacement)
	{
		return ikState_notContinuous;
	}

	return ikState_normal;
}

void DSKinematics::getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian)
{
	double calcAcs[6];
	Jacobian.resize(NUMOFJOINTS6, NUMOFJOINTS6);
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPosition(i) + m_mk2DH.theta[i];
	}
	Frame T = Frame::Identity();
	for (EcSizeT i = 0; i < NUMOFJOINTS6; i++)
	{
		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;
		T = T * Frame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], calcAcs[i]);
	}
	T = T * m_toolFrame;

	for (EcSizeT i = 0; i < NUMOFJOINTS6; i++)
	{
		m_velAxis[i] = T.p - m_pAxis[i];

		Vector z = vectorCrossOperator(m_zAxis[i], m_velAxis[i]);
		Jacobian(0, i) = z[0];
		Jacobian(1, i) = z[1];
		Jacobian(2, i) = z[2];

		Jacobian(3, i) = m_zAxis[i][0];
		Jacobian(4, i) = m_zAxis[i][1];
		Jacobian(5, i) = m_zAxis[i][2];
	}
}

void DSKinematics::getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot)
{
	double calcAcs[6];
	JacobianDot.resize(NUMOFJOINTS6, NUMOFJOINTS6);
	for (int i = 0; i < 6; i++)
	{
		calcAcs[i] = jointPosition(i) + m_mk2DH.theta[i];
	}

	Frame T = Frame::Identity();
	m_zAxis[0] = T.M.UnitZ();
	m_pAxis[0] = T.p;
	for (EcSizeT i = 1; i < NUMOFJOINTS6 + 1; i++)
	{
		T = T * Frame::DH(m_mk2DH.a[i - 1], m_mk2DH.alpha[i - 1], m_mk2DH.d[i - 1], calcAcs[i - 1]);
		if (i == NUMOFJOINTS6)
			T = T * m_toolFrame;			// 叠加工具坐标系的偏置

		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;

		m_omega[i] = jointVel(i - 1) * m_zAxis[i - 1] + m_omega[i - 1];
		m_pdotAxis[i] = m_pdotAxis[i - 1] + vectorCrossOperator(m_omega[i], m_pAxis[i] - m_pAxis[i - 1]);
	}


	for (EcSizeT i = 0; i < NUMOFJOINTS6; i++)
	{
		//JvDot(:,i) = [cross(cross(m_omega(:,i),m_zAxis(:,i)),(m_pAxis(:,7)-m_pAxis(:,i)))+cross(m_zAxis(:,i),m_pdotAxis(:,7)-m_pdotAxis(:,i)); 
		// cross(m_omega(:, i), m_zAxis(:, i))];
		m_velAxis[i] = T.p - m_pAxis[i];
		Vector z = vectorCrossOperator(vectorCrossOperator(m_omega[i], m_zAxis[i]), (m_pAxis[NUMOFJOINTS6] - m_pAxis[i])) + vectorCrossOperator(m_zAxis[i], m_pdotAxis[NUMOFJOINTS6] - m_pdotAxis[i]);
		Vector w = vectorCrossOperator(m_omega[i], m_zAxis[i]);

		JacobianDot(0, i) = z[0];
		JacobianDot(1, i) = z[1];
		JacobianDot(2, i) = z[2];

		JacobianDot(3, i) = w[0];
		JacobianDot(4, i) = w[1];
		JacobianDot(5, i) = w[2];
	}
}

EcReal DSKinematics::getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian)
{
	VectorXd joint(6);
	joint << jointPosition[0], jointPosition[1], jointPosition[2], jointPosition[3], jointPosition[4], jointPosition[5];
	getJacobianWithToolPointMatrix(joint, Jacobian);
	return Jacobian.determinant();
}

Eigen::MatrixXd DSKinematics::getJacobianMatrixOfArm(const EcRealVector& jointPosition)
{
	Eigen::MatrixXd PositionJacobian(DIMOFTASKSPACE3, NUMOFJOINTS3);
	EcReal s1 = KDL::sin(jointPosition[0]), s2 = KDL::sin(jointPosition[1]), s3 = KDL::sin(jointPosition[2]), s4 = KDL::sin(jointPosition[3]), s5 = KDL::sin(jointPosition[4]), s6 = KDL::sin(jointPosition[5]);
	EcReal c1 = KDL::cos(jointPosition[0]), c2 = KDL::cos(jointPosition[1]), c3 = KDL::cos(jointPosition[2]), c4 = KDL::cos(jointPosition[3]), c5 = KDL::cos(jointPosition[4]), c6 = KDL::cos(jointPosition[5]);
	EcReal s23 = KDL::sin(jointPosition[1] - jointPosition[2]), c23 = KDL::cos(jointPosition[1] - jointPosition[2]);

	PositionJacobian << s1 * (m_a2 * s2 + m_d4 * s23), -c1 * (m_a2 * c2 + m_d4 * c23), m_d4* c23* c1,
		-c1 * (m_a2 * s2 + m_d4 * s23), -s1 * (m_a2 * c2 + m_d4 * c23), m_d4* c23* s1,
		0, -m_a2 * s2 - m_d4 * s23, m_d4* s23;

	return PositionJacobian;
}

Eigen::MatrixXd DSKinematics::getJacobianMatrixOfWrist(const EcRealVector& jointPosition)
{
	Eigen::MatrixXd PositionJacobian(DIMOFTASKSPACE3, NUMOFJOINTS3);
	EcReal s1 = KDL::sin(jointPosition[0]), s2 = KDL::sin(jointPosition[1]), s3 = KDL::sin(jointPosition[2]), s4 = KDL::sin(jointPosition[3]), s5 = KDL::sin(jointPosition[4]), s6 = KDL::sin(jointPosition[5]);
	EcReal c1 = KDL::cos(jointPosition[0]), c2 = KDL::cos(jointPosition[1]), c3 = KDL::cos(jointPosition[2]), c4 = KDL::cos(jointPosition[3]), c5 = KDL::cos(jointPosition[4]), c6 = KDL::cos(jointPosition[5]);
	EcReal s23 = KDL::sin(jointPosition[1] - jointPosition[2]), c23 = KDL::cos(jointPosition[1] - jointPosition[2]);

	PositionJacobian << -s23 * c1, -c4 * s1 - c23 * c1 * s4, -s23 * c1 * c5 - s1 * s4 * s5 + c23 * c1 * c4 * s5,
	-s23 * s1, c1 * c4 - c23 * s1 * s4, -s23 * c5 * s1 + c1 * s4 * s5 + c23 * c4 * s1 * s5,
	c23, -s23 * s4, c23 * c5 + s23 * c4 * s5;

	return PositionJacobian;
}

void DSKinematics::configuration(const EcRealVector& wrappedJoint, int globalConf[3])
{
	// 确定肩部(Hinge)臂型
	EcReal wx1 = m_a2 * std::sin(wrappedJoint[1]) + m_d4 * std::sin(wrappedJoint[1] + wrappedJoint[2]);
	globalConf[0] = (wx1 < 0) ? -1 : 1;

	// 确定肘部/腕部(Pivot)臂型
	globalConf[1] = (wrappedJoint[2] < 0) ? -1 : 1;
	globalConf[2] = (wrappedJoint[4] < 0) ? -1 : 1;
}

void DSKinematics::normalize4ConfigHolding(const EcRealVector& refJoint, const EcRealVector& wrappedJoint, EcRealVector& normalizedJoint)
{
	EcRealVector standardSolution = wrappedJoint;  // wrappedJoint一定在[-PI, PI]之内
	EcRealVector expandedSolution = { 0,0,0,0,0,0 };
	EcRealVector currentDatumPoint = { 0,0,0,0,0,0 };
	// 获取refJoint对应的基准点
	for (int i = 0; i < 6; i++)
	{
		int temp = fabs(refJoint[i]) / PI;
		currentDatumPoint[i] = sign(refJoint[i]) * (temp + temp % 2) * PI; // 获取距离refJoint最近的2kPI作为[-3PI,3PI]平移的参考基准点
	}
	// 先得到refJoint附近的标准解
	for (int i = 0; i < 6; i++)
	{
		standardSolution[i] += currentDatumPoint[i]; // 标准解平移至refJoint附近
	}
	// 再得到refJoint附近的最近拓展解（单侧）;
	for (int i = 0; i < 6; i++)
	{
		// 由于refJoint一定满足关节极限，故仅需考虑refJoint左/右最邻近的两个周期解，其中一个一定为标准解,另一个为另一侧的拓展解
		if (standardSolution[i] > refJoint[i])
		{
			expandedSolution[i] = standardSolution[i] - 2.0 * PI;
		}
		else
		{
			expandedSolution[i] = standardSolution[i] + 2.0 * PI;
		}
		// normalize过程: 优先考虑解的可行性
		if ((isInJointLimit(standardSolution[i], i)) && (!isInJointLimit(expandedSolution[i], i))) // 只有标准解满足关节极限时
		{
			normalizedJoint[i] = standardSolution[i];
		}
		else if ((!isInJointLimit(standardSolution[i], i)) && (isInJointLimit(expandedSolution[i], i))) // 只有拓展解满足关节极限时
		{
			normalizedJoint[i] = expandedSolution[i];
		}
		else // 如果标准解和拓展解，两者同时满足/同时不满足关节极限，则选择距离refJoint较近的解作为normalize的输出
		{
			normalizedJoint[i] = (fabs(standardSolution[i] - refJoint[i]) < fabs(expandedSolution[i] - refJoint[i])) ? standardSolution[i] : expandedSolution[i];
		}
	}
}

void DSKinematics::wrapToPi(const EcRealVector& refJoint, EcRealVector& wrappedJoint)
{
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		// 平移至[-2π, 2π)范围
		double angle = fmod(refJoint[i], 2.0 * PI);

		// 约束至[-π, π]范围
		if (angle > PI) {
			angle -= 2.0 * PI;
		}
		else if (angle < -PI) {
			angle += 2.0 * PI;
		}

		wrappedJoint[i] = angle;
	}
}