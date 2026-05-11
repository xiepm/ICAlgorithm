#include "hic_controller/kinematics/elfinKinematics.h"
#include <iostream>


elfinKinematics::elfinKinematics()
{
	m_d1 = 0.220;
	m_d4 = 0.420;
	m_d6 = 0.155;
	m_a2 = 0.380;
	elfinDHParameters();
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

elfinKinematics::~elfinKinematics()
{

}

void elfinKinematics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

	elfinDHParameters();
}

void elfinKinematics::setJointMotionLimit
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

void elfinKinematics::elfinDHParameters()
{
	//EN_RobotDH dh;

	m_mk2DH.theta[0] = 0;
	m_mk2DH.theta[1] = PI / 2;
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

	m_mk2DH.alpha[0] = PI / 2;
	m_mk2DH.alpha[1] = PI;
	m_mk2DH.alpha[2] = PI / 2;
	m_mk2DH.alpha[3] = -PI / 2;
	m_mk2DH.alpha[4] = PI / 2;
	m_mk2DH.alpha[5] = 0;

	m_compTheta2 = PI / 2;
	m_compTheta3 = PI / 2;
}

double elfinKinematics::vectorMultiplyOperator(const Vector& v1, const Vector& v2)
{
	return (v1(0) * v2(0) + v1(1) * v2(1) + v1(2) * v2(2));
}

Vector elfinKinematics::vectorCrossOperator(const Vector& v1, const Vector& v2)
{
	return{ v1(1) * v2(2) - v2(1) * v1(2), v1(2) * v2(0) - v1(0) * v2(2), v1(0) * v2(1) - v2(0) * v1(1) };
}

void elfinKinematics::setToolCoordinateSystem
(
	const EcFrame& tool
)
{
	m_toolFrame = tool;
}

Frame elfinKinematics::forwardKinematics(const EcRealVector& jointPositions)
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

void elfinKinematics::forwardKinematics2(const EcRealVector& q, EcFrame& fkFrame)
{
	EcReal sq0, sq1, sq2, sq3, sq4, sq5, cq0, cq1, cq2, cq3, cq4, cq5;
	sq0 = KDL::sin(q[0]); cq0 = KDL::cos(q[0]);
	sq1 = KDL::sin(q[1]); cq1 = KDL::cos(q[1]);
	sq2 = KDL::sin(q[2]); cq2 = KDL::cos(q[2]);
	sq3 = KDL::sin(q[3]); cq3 = KDL::cos(q[3]);
	sq4 = KDL::sin(q[4]); cq4 = KDL::cos(q[4]);
	sq5 = KDL::sin(q[5]); cq5 = KDL::cos(q[5]);

	fkFrame.M = Rotation((((sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 - sq0 * sq3) * cq4 + (sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq4) * cq5 + (-(sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 - sq0 * cq3) * sq5, -(((sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 - sq0 * sq3) * cq4 + (sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq4) * sq5 + (-(sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 - sq0 * cq3) * cq5, ((sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 - sq0 * sq3) * sq4 - (sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq4,
		(((sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + sq3 * cq0) * cq4 + (sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq4) * cq5 + (-(sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + cq0 * cq3) * sq5, -(((sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + sq3 * cq0) * cq4 + (sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq4) * sq5 + (-(sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + cq0 * cq3) * cq5, ((sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + sq3 * cq0) * sq4 - (sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq4,
		((-sq1 * sq2 - cq1 * cq2) * sq4 + (sq1 * cq2 - sq2 * cq1) * cq3 * cq4) * cq5 - (sq1 * cq2 - sq2 * cq1) * sq3 * sq5, -((-sq1 * sq2 - cq1 * cq2) * sq4 + (sq1 * cq2 - sq2 * cq1) * cq3 * cq4) * sq5 - (sq1 * cq2 - sq2 * cq1) * sq3 * cq5, -(-sq1 * sq2 - cq1 * cq2) * cq4 + (sq1 * cq2 - sq2 * cq1) * sq4 * cq3
	);
	fkFrame.p = { m_d6 * ((sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 - sq0 * sq3) * sq4 - m_d6 * (sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq4 - m_d4 * sq1 * cq0 * cq2 - m_a2 * sq1 * cq0 + m_d4 * sq2 * cq0 * cq1,
		m_d6 * ((sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + sq3 * cq0) * sq4 - m_d6 * (sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq4 - m_d4 * sq0 * sq1 * cq2 - m_a2 * sq0 * sq1 + m_d4 * sq0 * sq2 * cq1,
		-m_d6 * (-sq1 * sq2 - cq1 * cq2) * cq4 + m_d6 * (sq1 * cq2 - sq2 * cq1) * sq4 * cq3 + m_d4 * sq1 * sq2 + m_d4 * cq1 * cq2 + m_a2 * cq1 + m_d1 };
	fkFrame = fkFrame * m_toolFrame;
}

ENInverseKineState elfinKinematics::inverseKinematics(const Frame& target, const EcRealVector& current, EcRealVector& acs)
{
	acs.assign(6, 0.0);
	EcBoolean ret;
	int ret1, ret2;
	ret = true;
	Frame T;
	T = target * m_toolFrame.Inverse();
	double L6 = m_d6;
	Vector end_p_vector = T.p;
	Vector end_z_vector = { T(0, 2), T(1, 2), T(2, 2) };
	Vector wrist_p_vector = end_p_vector - L6 * end_z_vector;

	double q1[2];
	q1[0] = KDL::atan2(wrist_p_vector(1), wrist_p_vector(0));
	q1[1] = q1[0] + PI;
	double q2_1[2], q2_2[2], q3_1[2], q3_2[2];

	ret1 = mk2Solve4Theta23(q1[0], wrist_p_vector, q2_1, q3_1);
	ret2 = mk2Solve4Theta23(q1[1], wrist_p_vector, q2_2, q3_2);
	if (ret1 + ret2 > 1)
	{
		acs = current;
		//std::cout << "ik no solution" << std::endl;
		return ikState_noSolution;
	}

	// solve joint 1，2，3；
	double q[][6] = {
	{ q1[0], q2_1[0], q3_1[0], 0, 0, 0 },
	{ q1[0], q2_1[0], q3_1[0], 0, 0, 0 },
	{ q1[0], q2_1[1], q3_1[1], 0, 0, 0 },
	{ q1[0], q2_1[1], q3_1[1], 0, 0, 0 },
	{ q1[1], q2_2[0], q3_2[0], 0, 0, 0 },
	{ q1[1], q2_2[0], q3_2[0], 0, 0, 0 },
	{ q1[1], q2_2[1], q3_2[1], 0, 0, 0 },
	{ q1[1], q2_2[1], q3_2[1], 0, 0, 0 } };

	for (int i = 0; i < 8; i = i + 2)
	{
		mk2Solve4wrist(q[i], T, 1);
		mk2Solve4wrist(q[i + 1], T, -1);
	}


	EcRealVector tempVector(6);
	EcRealVectorVector jointPositions;
	jointPositions.assign(8, tempVector);

	// 需要1、4、6为多圈的情况； 经解析解得到的关节角度范围为±180°，在设定的关节边界内，遍历比较±2*n*pi，从中寻找一个最小值；
	// 不需要遍历，定义最优解：与参考角度相差不大于2PI，且在边界范围内？
	normalize(current, q, jointPositions);			// regular into ±2pi range


	double faux0, faux1, faux2;
	EcRealVector sum1(NUMOFSOLVED8), sum2(NUMOFSOLVED8), sum3(NUMOFSOLVED8), sum6(NUMOFSOLVED8);
	for (int i = 0; i < NUMOFSOLVED8; i++)
	{
		faux0 = jointPositions[i][0] - current[0];
		sum1[i] = faux0 * faux0;

		faux1 = jointPositions[i][1] - current[1];
		sum2[i] = sum1[i] + faux1 * faux1;

		faux2 = jointPositions[i][2] - current[2];
		sum3[i] = sum2[i] + faux2 * faux2;

		faux0 = jointPositions[i][3] - current[3];
		faux1 = jointPositions[i][4] - current[4];
		faux2 = jointPositions[i][5] - current[5];
		sum6[i] = sum3[i] + faux0 * faux0 + faux1 * faux1 + 0.1 * faux2 * faux2;
	}

	double SumMin = sum6[0];
	int out_index = 0;
	for (int i = 1; i < NUMOFSOLVED8; i++)
	{
		if (fabs(SumMin - sum6[i]) < 1e-6)
		{
			if (fabs(sum3[out_index] - sum3[i]) < 1e-6)
			{
				if (fabs(sum2[out_index] - sum2[i]) < 1e-6)
				{
					if (sum1[out_index] > sum1[i])
					{
						SumMin = sum6[i];
						out_index = i;
					}
				}
				else if (sum2[out_index] > sum2[i])
				{
					SumMin = sum6[i];
					out_index = i;
				}
			}
			else if (sum3[out_index] > sum3[i])
			{
				SumMin = sum6[i];
				out_index = i;
			}
		}
		else if (SumMin > sum6[i])
		{
			SumMin = sum6[i];
			out_index = i;
		}
	}
	for (int i = 0; i < 6; i++)
	{
		acs[i] = jointPositions[out_index][i];
	}

	for (int i = 0; i < 6; i++)
	{
		if ((acs[i] < m_lowerJointLimit[i]) || (acs[i] > m_upperJointLimit[i]))
		{
			return ikState_outofLimit;
		}
	}

	// 逆解连续性判据(待商榷)
	EcReal maxDisplacement = PI / 6;  // 暂无理论根据
	EcReal sumDisplacement = 0;
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		sumDisplacement += fabs(acs[i] - current[i]);
	}
	if (sumDisplacement > maxDisplacement)
	{
		return ikState_notContinuous;
	}

	return ikState_normal;
}

ENInverseKineState elfinKinematics::inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint,EcRealVector& targetJoint)
{
	targetJoint = refJoint;
	Frame T = targetPose * m_toolFrame.Inverse();


	// 获取目标位姿
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
	configuration(refJoint,globalConf);

	// 计算IK
	EcReal wx = px - ax * m_d6;
	EcReal wy = py - ay * m_d6;
	EcReal wz = pz - az * m_d6;
	// q1
	EcReal q1;
	if ((wx * wx + wy * wy) < EPSILON2)  // 肩部奇异
	{
		std::cout << "Shoulder Singularity." << std::endl;
		q1 = refJoint[0];
	}
	else
	{
		q1 = std::atan2(wy, wx) + (1 + globalConf[0]) / 2 * PI;
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
	if (fabs(cTriangle + 1) < EPSILON2) // 肘部奇异
	{
		std::cout << "Elbow Singularity." << std::endl;
		q2 = globalConf[0] * beta;
		q3 = 0;
	}
	else
	{
		EcReal gama = std::acos((m_a2 * m_a2 + r * r - m_d4 * m_d4) / (2 * m_a2 * r));
		q2 = globalConf[0] * beta + globalConf[1] * gama;
		q3 = globalConf[1] * (PI - std::acos(cTriangle));
	}
	// q5
	EcReal c5 = az * std::cos(q2 - q3) - (ax * std::cos(q1) + ay * std::sin(q1)) * std::sin(q2 - q3);
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
		q4 = refJoint[3];
		q6 = std::atan2(s46, c46) - q4;
	}
	else
	{
		EcReal k1 = ay * std::cos(q1) - ax * std::sin(q1);
		EcReal k2 = az * std::sin(q2 - q3) + (ax * std::cos(q1) + ay * std::sin(q1)) * std::cos(q2 - q3);
		q4 = std::atan2(k1 * std::sin(q5), k2 * std::sin(q5));
		EcReal k3 = oz * std::cos(q2 - q3) - (ox * std::cos(q1) + oy * std::sin(q1)) * std::sin(q2 - q3);
		EcReal k4 = nz * std::cos(q2 - q3) - (nx * std::cos(q1) + ny * std::sin(q1)) * std::sin(q2 - q3);
		q6 = std::atan2(k3 * std::sin(q5), -k4 * std::sin(q5));
	}
	EcReal qIK[6] = {q1, q2, q3, q4, q5, q6 };
	normalize4ConfigHolding(refJoint, qIK, targetJoint);

	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		if ((targetJoint[i] < m_lowerJointLimit[i]) || (targetJoint[i] > m_upperJointLimit[i]))
		{
			std::cout << "Out of Limit." << std::endl;
			return ikState_outofLimit; // 后续可替换为用inverseKinematics重新逆解
		}
	}
	return ikState_normal;
}

int elfinKinematics::mk2Solve4Theta23(const double& q1, Vector& pw_vector, double q2[2], double q3[2])
{
	int ret = 0;
	double L2 = m_a2;
	double L3 = m_d4;
	Frame T01;
	T01.M = Rotation(KDL::cos(q1), 0, KDL::sin(q1),
		KDL::sin(q1), 0, -KDL::cos(q1),
		0, 1, 0);
	T01.p = { 0, 0, m_d1 };
	Vector p1_vector;
	p1_vector = T01.Inverse() * pw_vector;

	double r = KDL::sqrt(p1_vector(0) * p1_vector(0) + p1_vector(1) * p1_vector(1));
	double beta = KDL::atan2(p1_vector(1), p1_vector(0));
	double elem1 = (L2 * L2 + r * r - L3 * L3) / (2 * r * L2);
	if ((elem1 < -1) || (elem1 > 1))
	{
		ret = 1;
		return ret;
	}

	double gama = KDL::acos(elem1);

	q2[0] = beta + gama - PI / 2;
	q2[1] = beta - gama - PI / 2;

	double elem2 = (L2 * L2 + L3 * L3 - r * r) / (2 * L2 * L3);
	if ((elem2 < -1) || (elem2 > 1))
	{
		ret = 1;
		return ret;
	}
	double omega = KDL::acos(elem2);
	q3[0] = -omega + PI;
	q3[1] = omega - PI;

	return ret;
}

int elfinKinematics::normalize(const EcRealVector& current, double q[][6], EcRealVectorVector& qVector)
{
	EcRealVector standardSolution = { 0,0,0,0,0,0 };
	EcRealVector expandedSolution = { 0,0,0,0,0,0 };
	EcRealVector currentDatumPoint = { 0,0,0,0,0,0 };
	// [-pi,pi]内的standardSolution和[-pi,pi]外的expandedSolution，以及其他平移2PI后的解，统称为periodicSolution
	for (int i = 0; i < 8; i++)
	{
		// 获取current对应的基准点
		for (int j = 0; j < 6; j++)
		{
			int temp = fabs(current[j]) / PI;
			currentDatumPoint[j] = sign(current[j]) * (temp + temp % 2) * PI; // 获取距离current最近的2kPI作为[-3PI,3PI]平移的参考基准点
		}
		// 先得到current附近的标准解;
		for (int j = 0; j < 6; j++)
		{
			// 获取[-pi,pi]内的标准解
			if (q[i][j] > PI)
			{
				standardSolution[j] = q[i][j] - 2.0 * PI;
			}
			else if (q[i][j] <= -PI)
			{
				standardSolution[j] = q[i][j] + 2.0 * PI;
			}
			else {
				standardSolution[j] = q[i][j];
			}
			standardSolution[j] += currentDatumPoint[j]; // 标准解平移至current附近
		}
		// 再得到current附近的最近拓展解（单侧）;
		for (int j = 0; j < 6; j++)
		{
			// 由于current一定满足关节极限，故仅需考虑current左/右最邻近的两个周期解，其中一个一定为标准解,另一个为另一侧的拓展解
			if (standardSolution[j] > current[j])
			{
				expandedSolution[j] = standardSolution[j] - 2.0 * PI;
			}
			else
			{
				expandedSolution[j] = standardSolution[j] + 2.0 * PI;
			}
			// normalize过程: 优先考虑解的可行性
			if ((isInJointLimit(standardSolution[j], j)) && (!isInJointLimit(expandedSolution[j], j))) // 只有标准解满足关节极限时
			{
				qVector[i][j] = standardSolution[j];
			}
			else if ((!isInJointLimit(standardSolution[j], j)) && (isInJointLimit(expandedSolution[j], j))) // 只有拓展解满足关节极限时
			{
				qVector[i][j] = expandedSolution[j];
			}
			else // 如果标准解和拓展解，两者同时满足/同时不满足关节极限，则选择距离current较近的解作为normalize的输出
			{
				qVector[i][j] = (fabs(standardSolution[j] - current[j]) < fabs(expandedSolution[j] - current[j])) ? standardSolution[j] : expandedSolution[j];
			}
		}
	}
	/*
	int index[3] = { 0, 3, 5 };
	// 2. 将关节1、4、6的求解范围扩展到currentJoint 所在的圈内；;
	for (int jj = 0; jj < 3; jj++)
	{
		int cycleCount = current[index[jj]] / (2 * PI);
		m_joint2PIExpand[jj] = 2 * PI * cycleCount;
		EcReal qsign = m_joint2PIExpand[jj];
		qVector[i][index[jj]] += qsign;

		// 3. 圈内±2PI，寻找其中差值最小的解；
		EcReal	delta = fabs(qVector[i][index[jj]] - current[index[jj]]);
		EcReal deltaPlus2PI = fabs(qVector[i][index[jj]] + 2 * PI - current[index[jj]]);
		EcReal deltaMinus2PI = fabs(qVector[i][index[jj]] - 2 * PI - current[index[jj]]);

		if (fabs(deltaPlus2PI) < deltaMinus2PI)
			qVector[i][index[jj]] = (delta < deltaPlus2PI) ? qVector[i][index[jj]] : qVector[i][index[jj]] + 2 * PI;
		else
			qVector[i][index[jj]] = (delta < deltaMinus2PI) ? qVector[i][index[jj]] : qVector[i][index[jj]] - 2 * PI;


		// 如果搜寻得到的解超出边界，那么需要重新将解丢回边界中；
		if (fabs(qVector[i][index[jj]]) > m_upperJointLimit[index[jj]])			// 确保扩展后的角度不超出设定边界；
			qVector[i][index[jj]] -= sign(qVector[i][index[jj]]) * 2 * PI;

	}
	*/
	return 0;
}

int elfinKinematics::mk2Solve4wrist(double q[6], Frame& T, const int& wrist)
{
	int ret = 0;
	Frame T03;
	q[1] = q[1] + m_compTheta2;
	q[2] = q[2] + m_compTheta3;
	/*
	T01 = Frame::DH(m_mk2DH.a[0], m_mk2DH.alpha[0], m_mk2DH.d[0], q[0]);
	T12 = Frame::DH(m_mk2DH.a[1], m_mk2DH.alpha[1], m_mk2DH.d[1], q[1]);
	T23 = Frame::DH(m_mk2DH.a[2], m_mk2DH.alpha[2], m_mk2DH.d[2], q[2]);
	T03 = T01*T12*T23;
	*/

	EcReal sq0, cq0, sq1, cq1, sq2, cq2, sq3, cq3, sq4, cq4;
	sq0 = KDL::sin(q[0]);	cq0 = KDL::cos(q[0]);
	sq1 = KDL::sin(q[1]);	cq1 = KDL::cos(q[1]);
	sq2 = KDL::sin(q[2]);	cq2 = KDL::cos(q[2]);

	T03.M = Rotation(sq1 * sq2 * cq0 + cq0 * cq1 * cq2, -sq0, -sq1 * cq0 * cq2 + sq2 * cq0 * cq1,
		sq0 * sq1 * sq2 + sq0 * cq1 * cq2, cq0, -sq0 * sq1 * cq2 + sq0 * sq2 * cq1,
		sq1 * cq2 - sq2 * cq1, 0, sq1 * sq2 + cq1 * cq2);


	Vector x3_vector = { T03(0, 0), T03(1, 0), T03(2, 0) };
	Vector y3_vector = { T03(0, 1), T03(1, 1), T03(2, 1) };
	Vector z3_vector = { T03(0, 2), T03(1, 2), T03(2, 2) };

	Vector z5_vector = { T(0, 2), T(1, 2), T(2, 2) };
	Vector z4_vector = vectorCrossOperator(z3_vector, z5_vector);

	if (z4_vector.Norm() < 0.00000001)
	{
		if (wrist == 1)
			q[3] = 0;
		else
			q[3] = -PI;
	}
	else
	{
		double cq4 = wrist * vectorMultiplyOperator(y3_vector, z4_vector);
		double sq4 = -wrist * vectorMultiplyOperator(x3_vector, z4_vector);
		q[3] = KDL::atan2(sq4, cq4);
	}


	sq3 = KDL::sin(q[3]);	cq3 = KDL::cos(q[3]);
	Vector x4_vector = { (sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 - sq0 * sq3,
		(sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + sq3 * cq0,
		(sq1 * cq2 - sq2 * cq1) * cq3 };
	Vector y4_vector = { sq1 * cq0 * cq2 - sq2 * cq0 * cq1,
		sq0 * sq1 * cq2 - sq0 * sq2 * cq1,
		-sq1 * sq2 - cq1 * cq2 };



	double cq5 = -vectorMultiplyOperator(y4_vector, z5_vector);
	double sq5 = vectorMultiplyOperator(x4_vector, z5_vector);
	q[4] = KDL::atan2(sq5, cq5);

	Vector x6_vector = { T(0, 0), T(1, 0), T(2, 0) };

	sq4 = KDL::sin(q[4]);	cq4 = KDL::cos(q[4]);
	Vector x5_vector = { ((sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 - sq0 * sq3) * cq4 + (sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq4,
		((sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + sq3 * cq0) * cq4 + (sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq4,
		(-sq1 * sq2 - cq1 * cq2) * sq4 + (sq1 * cq2 - sq2 * cq1) * cq3 * cq4 };

	Vector y5_vector = { -(sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 - sq0 * cq3,
		-(sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + cq0 * cq3,
		-(sq1 * cq2 - sq2 * cq1) * sq3 };

	double cq6 = vectorMultiplyOperator(x6_vector, x5_vector);
	double sq6 = vectorMultiplyOperator(x6_vector, y5_vector);
	q[5] = KDL::atan2(sq6, cq6);

	q[1] = q[1] - m_mk2DH.theta[1];
	q[2] = q[2] - m_mk2DH.theta[2];

	return ret;
}

void elfinKinematics::getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian)
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

void elfinKinematics::getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot)
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

EcReal elfinKinematics::getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian)
{
	Jacobian.resize(NUMOFJOINTS6, NUMOFJOINTS6);
	EcReal s1 = KDL::sin(jointPosition[0]), s2 = KDL::sin(jointPosition[1]), s3 = KDL::sin(jointPosition[2]), s4 = KDL::sin(jointPosition[3]), s5 = KDL::sin(jointPosition[4]), s6 = KDL::sin(jointPosition[5]);
	EcReal c1 = KDL::cos(jointPosition[0]), c2 = KDL::cos(jointPosition[1]), c3 = KDL::cos(jointPosition[2]), c4 = KDL::cos(jointPosition[3]), c5 = KDL::cos(jointPosition[4]), c6 = KDL::cos(jointPosition[5]);

	Jacobian << -m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 + m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5 + m_d4 * s1 * s2 * c3 + m_a2 * s1 * s2 - m_d4 * s1 * s3 * c2, -(-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4 + m_d4 * s2 * s3 + m_d4 * c2 * c3 + m_a2 * c2) * c1, (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4 + m_d4 * s2 * s3 + m_d4 * c2 * c3)* c1, -(s2 * s3 + c2 * c3) * (m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5 - m_d4 * s1 * s2 * c3 + m_d4 * s1 * s3 * c2) + (-s1 * s2 * c3 + s1 * s3 * c2) * (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4 + m_d4 * s2 * s3 + m_d4 * c2 * c3), (m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5)* (s2 * c3 - s3 * c2)* s4 + (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4) * (-(s1 * s2 * s3 + s1 * c2 * c3) * s4 + c1 * c4), -(m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5) * (-(-s2 * s3 - c2 * c3) * c5 + (s2 * c3 - s3 * c2) * s5 * c4) + (((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - (s1 * s2 * c3 - s1 * s3 * c2) * c5) * (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4),
		m_d6* ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4)* s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5 - m_d4 * s2 * c1 * c3 - m_a2 * s2 * c1 + m_d4 * s3 * c1 * c2, -(-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4 + m_d4 * s2 * s3 + m_d4 * c2 * c3 + m_a2 * c2) * s1, (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4 + m_d4 * s2 * s3 + m_d4 * c2 * c3)* s1, (s2 * s3 + c2 * c3)* (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5 - m_d4 * s2 * c1 * c3 + m_d4 * s3 * c1 * c2) - (-s2 * c1 * c3 + s3 * c1 * c2) * (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4 + m_d4 * s2 * s3 + m_d4 * c2 * c3), -(m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5) * (s2 * c3 - s3 * c2) * s4 - (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4) * (-(s2 * s3 * c1 + c1 * c2 * c3) * s4 - s1 * c4), (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5)* (-(-s2 * s3 - c2 * c3) * c5 + (s2 * c3 - s3 * c2) * s5 * c4) - (((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - (s2 * c1 * c3 - s3 * c1 * c2) * c5) * (-m_d6 * (-s2 * s3 - c2 * c3) * c5 + m_d6 * (s2 * c3 - s3 * c2) * s5 * c4),
		0, (m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5 - m_d4 * s1 * s2 * c3 - m_a2 * s1 * s2 + m_d4 * s1 * s3 * c2)* s1 + (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5 - m_d4 * s2 * c1 * c3 - m_a2 * s2 * c1 + m_d4 * s3 * c1 * c2) * c1, -(m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5 - m_d4 * s1 * s2 * c3 + m_d4 * s1 * s3 * c2) * s1 - (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5 - m_d4 * s2 * c1 * c3 + m_d4 * s3 * c1 * c2) * c1, -(-s1 * s2 * c3 + s1 * s3 * c2) * (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5 - m_d4 * s2 * c1 * c3 + m_d4 * s3 * c1 * c2) + (-s2 * c1 * c3 + s3 * c1 * c2) * (m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5 - m_d4 * s1 * s2 * c3 + m_d4 * s1 * s3 * c2), (m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5)* (-(s2 * s3 * c1 + c1 * c2 * c3) * s4 - s1 * c4) - (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5) * (-(s1 * s2 * s3 + s1 * c2 * c3) * s4 + c1 * c4), (m_d6 * ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - m_d6 * (s1 * s2 * c3 - s1 * s3 * c2) * c5)* (((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - (s2 * c1 * c3 - s3 * c1 * c2) * c5) - (((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1) * s5 - (s1 * s2 * c3 - s1 * s3 * c2) * c5) * (m_d6 * ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4) * s5 - m_d6 * (s2 * c1 * c3 - s3 * c1 * c2) * c5),
		0, s1, -s1, -s2 * c1 * c3 + s3 * c1 * c2, -(s2 * s3 * c1 + c1 * c2 * c3) * s4 - s1 * c4, ((s2 * s3 * c1 + c1 * c2 * c3) * c4 - s1 * s4)* s5 - (s2 * c1 * c3 - s3 * c1 * c2) * c5,
		0, -c1, c1, -s1 * s2 * c3 + s1 * s3 * c2, -(s1 * s2 * s3 + s1 * c2 * c3) * s4 + c1 * c4, ((s1 * s2 * s3 + s1 * c2 * c3) * c4 + s4 * c1)* s5 - (s1 * s2 * c3 - s1 * s3 * c2) * c5,
		1, 0, 0, s2* s3 + c2 * c3, -(s2 * c3 - s3 * c2) * s4, -(-s2 * s3 - c2 * c3) * c5 + (s2 * c3 - s3 * c2) * s5 * c4;
	return Jacobian.determinant();
}

void elfinKinematics::configuration(const EcRealVector& refJoint, int globalConf[3])
{
	// 确定肩部(Hinge)臂型
	EcReal wx1 = m_a2 * std::sin(refJoint[1]) + m_d4 * std::sin(refJoint[1] - refJoint[2]);
	globalConf[0] = (wx1 < 0) ? -1 : 1;

	// 确定肘部/腕部(Pivot)臂型
	globalConf[1] = (refJoint[2] < 0) ? -1 : 1;
	globalConf[2] = (refJoint[4] < 0) ? -1 : 1;
}

int elfinKinematics::normalizePI(double q[NUMOFJOINTS6])
{
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		if (q[i] > PI)
		{
			q[i] -= 2.0 * PI;
		}
		else if (q[i] <= -PI)
		{
			q[i] += 2.0 * PI;
		}
	}
	return 0;
}

int elfinKinematics::normalize4ConfigHolding(const EcRealVector& current, double q[NUMOFJOINTS6], EcRealVector& qVector)
{
	EcRealVector mirrorSolution = { 0,0,0,0,0,0 };
	// 获取[-2PI,2PI]内的镜像解
	for (int i = 0; i < 6; i++)
	{
		if (q[i] > 0)
		{
			mirrorSolution[i] = q[i] - 2.0 * PI;
		}
		else //由于逆解表达式的约束，入参q一定在[-2PI, 2PI]之内
		{
			mirrorSolution[i] = q[i] + 2.0 * PI;
		}
	}
	// 在双解中选择确定最终逆解
	for (int i = 0; i < 6; i++)
	{
		// 优先考虑解的可行性
		if ((isInJointLimit(q[i], i)) && (!isInJointLimit(mirrorSolution[i], i)))
		{
			qVector[i] = q[i];
		}
		else if ((!isInJointLimit(q[i], i)) && (isInJointLimit(mirrorSolution[i], i)))
		{
			qVector[i] = mirrorSolution[i];
		}
		else // 如果两者同时满足/同时不满足关节极限，则选择距离current较近的解作为normalize的输出
		{
			qVector[i] = (fabs(q[i] - current[i]) < fabs(mirrorSolution[i] - current[i])) ? q[i] : mirrorSolution[i];
		}
	}
	return 0;
}