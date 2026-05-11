#include "hic_controller/kinematics/URKinematics.h"
#include <iostream>


URKinematics::URKinematics()
{
	m_d1 = 0.1710; // 修改后的DH Parameter
	m_d4 = 0.1830;
	m_d5 = 0.1355;
	m_d6 = 0.1132;
	m_a2 = -0.8500;
	m_a3 = -0.7145;

	URDHParameters();

	m_upperJointLimit.assign(6, PI);
	m_lowerJointLimit.assign(6, -PI);
	m_toolFrame = Frame::Identity();

	KDL::Vector v;
	m_zAxis.assign(NUMOFJOINTS6 + 1, v);
	m_pAxis.assign(NUMOFJOINTS6 + 1, v);
	m_velAxis.assign(NUMOFJOINTS6 + 1, v);

	m_pdotAxis.assign(NUMOFJOINTS6 + 1, v);
	m_omega.assign(NUMOFJOINTS6 + 1, v);
}


URKinematics::~URKinematics()
{

}

// 入参kinematcisParam: [d1 d2 d3 d4 d5 d6 a2 a3]
void URKinematics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1] - kinematcisParam[2] + kinematcisParam[3];
	m_d5 = kinematcisParam[4];
	m_d6 = kinematcisParam[5];
	m_a2 = -kinematcisParam[6];
	m_a3 = -kinematcisParam[7];

	URDHParameters();
}


void URKinematics::URDHParameters()
{
	//EN_RobotDH dh;

	m_mk2DH.theta[0] = 0;
	m_mk2DH.theta[1] = 0;
	m_mk2DH.theta[2] = 0;
	m_mk2DH.theta[3] = 0;
	m_mk2DH.theta[4] = 0;
	m_mk2DH.theta[5] = 0;

	m_mk2DH.d[0] = m_d1;
	m_mk2DH.d[1] = 0;
	m_mk2DH.d[2] = 0;
	m_mk2DH.d[3] = m_d4;
	m_mk2DH.d[4] = m_d5;
	m_mk2DH.d[5] = m_d6;

	m_mk2DH.a[0] = 0;
	m_mk2DH.a[1] = m_a2;
	m_mk2DH.a[2] = m_a3;
	m_mk2DH.a[3] = 0;
	m_mk2DH.a[4] = 0;
	m_mk2DH.a[5] = 0;

	m_mk2DH.alpha[0] = PI / 2;
	m_mk2DH.alpha[1] = 0;
	m_mk2DH.alpha[2] = 0;
	m_mk2DH.alpha[3] = PI / 2;
	m_mk2DH.alpha[4] = -PI / 2;
	m_mk2DH.alpha[5] = 0;

}

void URKinematics::setJointMotionLimit
(
	const EcRealVector& upperJointLimit,
	const EcRealVector& lowerJointLimit
)
{   
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		m_upperJointLimit[i] = upperJointLimit[i] + epsilon; // 避免由于数值传参损失精度导致边界解丢失
		m_lowerJointLimit[i] = lowerJointLimit[i] - epsilon;
	}
}

void URKinematics::setToolCoordinateSystem
(
	const EcFrame& tool
)
{

	m_toolFrame = tool;

}

Frame URKinematics::forwardKinematics(const EcRealVector& jointPositions)
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

void URKinematics::forwardKinematics2
(
	const EcRealVector& q,
	EcFrame& fkFrame
)
{
	/**/
	EcReal sq0, sq1, sq2, sq3, sq4, sq5, cq0, cq1, cq2, cq3, cq4, cq5;
	sq0 = KDL::sin(q[0]); cq0 = KDL::cos(q[0]);
	sq1 = KDL::sin(q[1]); cq1 = KDL::cos(q[1]);
	sq2 = KDL::sin(q[2]); cq2 = KDL::cos(q[2]);
	sq3 = KDL::sin(q[3]); cq3 = KDL::cos(q[3]);
	sq4 = KDL::sin(q[4]); cq4 = KDL::cos(q[4]);
	sq5 = KDL::sin(q[5]); cq5 = KDL::cos(q[5]);


	fkFrame.M = Rotation(
		(((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * cq4 + sq0 * sq4) * cq5 + (-(-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq3) * sq5, -(((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * cq4 + sq0 * sq4) * sq5 + (-(-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq3) * cq5, -((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * sq4 + sq0 * cq4,
		(((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * cq4 - sq4 * cq0) * cq5 + (-(-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq3) * sq5, -(((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * cq4 - sq4 * cq0) * sq5 + (-(-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq3) * cq5, -((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * sq4 - cq0 * cq4,
		((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * cq4 * cq5 + ((-sq1 * sq2 + cq1 * cq2) * cq3 - (sq1 * cq2 + sq2 * cq1) * sq3) * sq5, -((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * sq5 * cq4 + ((-sq1 * sq2 + cq1 * cq2) * cq3 - (sq1 * cq2 + sq2 * cq1) * sq3) * cq5, -((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * sq4
	);
	fkFrame.p = { -m_d6 * ((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * sq4 + m_d5 * (-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 - m_d5 * (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq3 + m_d6 * sq0 * cq4 + m_d4 * sq0 - m_a3 * sq1 * sq2 * cq0 + m_a3 * cq0 * cq1 * cq2 + m_a2 * cq0 * cq1,
		 -m_d6 * ((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * sq4 + m_d5 * (-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 - m_d5 * (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq3 - m_a3 * sq0 * sq1 * sq2 + m_a3 * sq0 * cq1 * cq2 + m_a2 * sq0 * cq1 - m_d6 * cq0 * cq4 - m_d4 * cq0,
		-m_d6 * ((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * sq4 - m_d5 * (-sq1 * sq2 + cq1 * cq2) * cq3 + m_d5 * (sq1 * cq2 + sq2 * cq1) * sq3 + m_a3 * sq1 * cq2 + m_a2 * sq1 + m_a3 * sq2 * cq1 + m_d1
	};

	/*
	fkFrame.M = Rotation(
		(((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * cq4 + sq0 * sq4) * cq5 + (-(-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq3) * sq5, -(((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * cq4 + sq0 * sq4) * sq5 + (-(-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq3) * cq5, -((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * sq4 + sq0 * cq4,
		(((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * cq4 - sq4 * cq0) * cq5 + (-(-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq3) * sq5, -(((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * cq4 - sq4 * cq0) * sq5 + (-(-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq3) * cq5, -((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * sq4 - cq0 * cq4,
		((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * cq4 * cq5 + ((-sq1 * sq2 + cq1 * cq2) * cq3 - (sq1 * cq2 + sq2 * cq1) * sq3) * sq5, -((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * sq5 * cq4 + ((-sq1 * sq2 + cq1 * cq2) * cq3 - (sq1 * cq2 + sq2 * cq1) * sq3) * cq5, -((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * sq4
	);

	fkFrame.p = { -m_d6 * ((-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * cq3 + (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * sq3) * sq4 + m_d5 * (-sq1 * sq2 * cq0 + cq0 * cq1 * cq2) * sq3 - m_d5 * (-sq1 * cq0 * cq2 - sq2 * cq0 * cq1) * cq3 + m_d6 * sq0 * cq4 + (m_d2 + m_d3 + m_d4) * sq0 - m_a3 * sq1 * sq2 * cq0 + m_a3 * cq0 * cq1 * cq2 + m_a2 * cq0 * cq1,
					-m_d6 * ((-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * cq3 + (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * sq3) * sq4 + m_d5 * (-sq0 * sq1 * sq2 + sq0 * cq1 * cq2) * sq3 - m_d5 * (-sq0 * sq1 * cq2 - sq0 * sq2 * cq1) * cq3 - m_a3 * sq0 * sq1 * sq2 + m_a3 * sq0 * cq1 * cq2 + m_a2 * sq0 * cq1 - m_d6 * cq0 * cq4 - (m_d2 + m_d3 + m_d4) * cq0,
					-m_d6 * ((-sq1 * sq2 + cq1 * cq2) * sq3 + (sq1 * cq2 + sq2 * cq1) * cq3) * sq4 - m_d5 * (-sq1 * sq2 + cq1 * cq2) * cq3 + m_d5 * (sq1 * cq2 + sq2 * cq1) * sq3 + m_a3 * sq1 * cq2 + m_a2 * sq1 + m_a3 * sq2 * cq1 + m_d1 };
	*/
	fkFrame = fkFrame * m_toolFrame;

}

ENInverseKineState URKinematics::inverseKinematics(const Frame& target, const EcRealVector& current, EcRealVector& acs)
{
	acs = current; // 默认返回参考角
	// 获取目标位姿
	Frame T = target * m_toolFrame.Inverse();
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

	// 初始化全逆解的二维数组
	double q[NUMOFSOLVED8][NUMOFJOINTS6] = { 0 };
	for (int i = 0; i < NUMOFSOLVED8; i++)
	{
		q[i][3] = INFINITE;  // 将q4置为是否可解的标志位
	}

	EcReal s1, c1, s2, c2, s3, c3, s5, c5, s6, c6, s234, c234;
	EcReal k1, k2, k3, k4, k5, k6;
	// q1
	k1 = px - m_d6 * ax;
	k2 = -(py - m_d6 * ay);
	EcReal dSquare = k1 * k1 + k2 * k2 - m_d4 * m_d4;
	if (dSquare < EPSILON2)
	{
		// std::cout << "Shoulder Singularity，or Ouside of Workspace!" << std::endl; // 目标点落入黑柱内
		acs = current;
		return ikState_noSolution;
	}
	EcReal d = KDL::sqrt(dSquare);
	EcReal alpha = KDL::atan2(k1, k2); // [-PI, PI]
	EcReal beta = KDL::atan2(d, m_d4); // [0,PI/2)
	for (int i = 0; i < 4; i++)
	{
		q[i][0] = alpha + beta;
		q[i+4][0] = alpha - beta;
	}

	// q5/q6
	int shoulderRowSet[2] = { 0, 4 }; // 基于肩部臂型区分的全逆解数组的起始行
	for (int &startRow : shoulderRowSet)
	{
		s1 = KDL::sin(q[startRow][0]);  c1 = KDL::cos(q[startRow][0]);
		c5 = ax * s1 - ay * c1;
		k3 = -(ox * s1 - oy * c1);  k4 = nx * s1 - ny * c1;
		if (fabs(c5) >= 1 - EPSILON2) // 腕部奇异时（1-cosX等价无穷小为X^2)
		{
			c5 = KDL::sign(c5);
			q[startRow + 0][4] = KDL::acos(c5);   q[startRow + 0][5] = current[5];
			q[startRow + 1][4] = KDL::acos(c5);   q[startRow + 1][5] = current[5];
			q[startRow + 2][4] = -KDL::acos(c5);  q[startRow + 2][5] = current[5];
			q[startRow + 3][4] = -KDL::acos(c5);  q[startRow + 3][5] = current[5];
		}
		else
		{
			q[startRow + 0][4] = KDL::acos(c5);   q[startRow + 0][5] = KDL::atan2(k3, k4);
			q[startRow + 1][4] = KDL::acos(c5);   q[startRow + 1][5] = KDL::atan2(k3, k4);
			q[startRow + 2][4] = -KDL::acos(c5);   q[startRow + 2][5] = KDL::atan2(-k3, -k4);
			q[startRow + 3][4] = -KDL::acos(c5);   q[startRow + 3][5] = KDL::atan2(-k3, -k4);
		}
	}

	// q2/q3/q4
	int wristRowSet[4] = { 0, 2, 4, 6 }; // 基于肩部及腕部臂型区分的全逆解数组的起始行
	int GCE; // 肘部臂型
	EcReal r, cTriangle;
	EcReal q2, q3, q4;
	EcReal temp_a, temp_b, temp_c, temp_d;
	for (int& startRow : wristRowSet)
	{
		s1 = KDL::sin(q[startRow][0]); c1 = KDL::cos(q[startRow][0]);
		s6 = KDL::sin(q[startRow][5]); c6 = KDL::cos(q[startRow][5]);
		k5 = m_d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - m_d6 * (ax * c1 + ay * s1) + px * c1 + py * s1;
		k6 = pz - m_d1 - az * m_d6 + m_d5 * (oz * c6 + nz * s6);
		r = KDL::sqrt(k5 * k5 + k6 * k6);
		if (r > fabs(m_a2 + m_a3) || r < fabs(m_a2 - m_a3)) // 超出工作空间
		{
			// std::cout << "startRow:" << startRow << " , Ouside of Workspace!" << std::endl;
			q[startRow + 0][1] = INFINITE; q[startRow + 0][2] = INFINITE; q[startRow + 0][3] = INFINITE;
			q[startRow + 1][1] = INFINITE; q[startRow + 1][2] = INFINITE; q[startRow + 1][3] = INFINITE;
		}
		else
		{
			cTriangle = (m_a2 * m_a2 + m_a3 * m_a3 - r * r) / (2 * m_a2 * m_a3);
			if (fabs(cTriangle) >= 1 - EPSILON2) // 肘部奇异附近
			{
				// std::cout << "startRow:" << startRow << " , Elbow Singularity!" << std::endl;
				cTriangle = KDL::sign(cTriangle);
			}
			for (int i = 0; i < 2; i++)
			{
				GCE = 1 - 2 * i;
				q3 = GCE * (PI - KDL::acos(cTriangle)); // q3

				s3 = KDL::sin(q3);	c3 = KDL::cos(q3);
				temp_a = m_a3 * c3 + m_a2;
				temp_b = m_a3 * s3;
				temp_c = k5;
				temp_d = k6;
				q2 = KDL::atan2(temp_a*temp_d-temp_b*temp_c, temp_a*temp_c+temp_b*temp_d); // q2

				s234 = -s6 * (nx * c1 + ny * s1) - c6 * (ox * c1 + oy * s1);
				c234 = oz * c6 + nz * s6;
				q4 = KDL::atan2(s234, c234) - q2 - q3; // q4

				q[startRow + i][1] = q2;
				q[startRow + i][2] = q3;
				q[startRow + i][3] = q4;
			}
		}
	}

	// 检查是否有解
	int count = 0;
	for (int i = 0; i < NUMOFSOLVED8; i++) // 将q4置为是否可解的标志位
	{
		if (INFINITE == q[i][3])
		{
			count = count + 1;
		}
	}
	if (NUMOFSOLVED8 == count)
	{
		// std::cout << "Ouside of Workspace!" << std::endl; // 超出工作空间
		acs = current;
		return ikState_noSolution;
	}

	// 后续根据行程最短原则，从八组逆解中选解
	EcRealVector tempVector(6);
	EcRealVectorVector jointPositions;
	jointPositions.assign(8, tempVector);

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
		sum6[i] = sum3[i] + faux0 * faux0 + faux1 * faux1 + faux2 * faux2;
	}
	// 依次比较总行程/前三个关节行程/前两个关节行程/关节1的行程
	double SumMin = sum6[0];
	int out_index = 0;
	for (int i = 1; i < NUMOFSOLVED8; i++)
	{
		if (fabs(SumMin - sum6[i]) < EPSILON2)
		{
			if (fabs(sum3[out_index] - sum3[i]) < EPSILON2)
			{
				if (fabs(sum2[out_index] - sum2[i]) < EPSILON2)
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
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		acs[i] = jointPositions[out_index][i];
	}

	// 检查关节超限
	for (int i = 0; i < NUMOFJOINTS6; i++)
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

ENInverseKineState URKinematics::inverseKinematicsWithConfigHolding(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint)
{
	targetJoint = refJoint;

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
	configuration(refJoint, globalConf);

	// q1
	EcReal k1 = px - m_d6 * ax;
	EcReal k2 = -(py - m_d6 * ay);
	EcReal dSquare = k1 * k1 + k2 * k2 - m_d4 * m_d4;
	if (dSquare < EPSILON2)
	{
		// std::cout << "Shoulder Singularity，or Ouside of Workspace!" << std::endl; // 目标点落入黑柱内
		targetJoint = refJoint;
		return ikState_noSolution;
	}
	EcReal d = KDL::sqrt(dSquare);
	EcReal alpha1 = KDL::atan2(k1, k2); // [-PI, PI]
	EcReal alpha2 = KDL::atan2(d, m_d4); // [0,PI/2)
	EcReal q1 = alpha1 + globalConf[0] * alpha2; // q1

	// q5/q6
	EcReal c5 = ax * KDL::sin(q1) - ay * KDL::cos(q1);
	if (fabs(c5) >= 1 - EPSILON2)
	{
		c5 = KDL::sign(c5);
	}
	EcReal q5 = globalConf[2] * KDL::acos(c5); // q5
	EcReal q6;
	if (fabs(std::sin(q5)) < EPSILON1) // 腕部奇异时
	{
		// std::cout << "Wrist Singularity!" << std::endl;
		q6 = refJoint[5];
	}
	else
	{
		EcReal k3 = -(ox * KDL::sin(q1) - oy * KDL::cos(q1));
		EcReal k4 = nx * KDL::sin(q1) - ny * KDL::cos(q1);
		q6 = KDL::atan2((globalConf[2] * k3), (globalConf[2] * k4)); // q6
	}

	// q2/q3/q4
	EcReal s1, c1, s6, c6; // 分别对应关节1/6的正余弦
	s1 = KDL::sin(q1);	c1 = KDL::cos(q1);
	s6 = KDL::sin(q6);	c6 = KDL::cos(q6);

	EcReal k5 = m_d5 * (s6 * (nx * c1 + ny * s1) + c6 * (ox * c1 + oy * s1)) - m_d6 * (ax * c1 + ay * s1) + px * c1 + py * s1;
	EcReal k6 = pz - m_d1 - az * m_d6 + m_d5 * (oz * c6 + nz * s6);
	EcReal r = KDL::sqrt(k5 * k5 + k6 * k6);
	if (r > fabs(m_a2 + m_a3) || r < fabs(m_a2 - m_a3)) // 超出工作空间 //注意a2和a3为负值
	{
		// std::cout << "Ouside of Workspace!" << std::endl;
		// std::cout << "q3ref: " << refJoint[2] << ",  q5ref: " << refJoint[4] << std::endl;
		targetJoint = refJoint;
		return ikState_noSolution;
	}
	EcReal cTriangle = (m_a2 * m_a2 + m_a3 * m_a3 - r * r) / (2 * m_a2 * m_a3);
	if (fabs(cTriangle) >= 1 - EPSILON2)
	{
		// std::cout << "Elbow Singularity!" << std::endl;
		cTriangle = KDL::sign(cTriangle);
	}
	EcReal q3 = globalConf[1] * (PI - KDL::acos(cTriangle)); // q3

	EcReal s3, c3;
	EcReal temp_a, temp_b, temp_c, temp_d;
	s3 = KDL::sin(q3);	c3 = KDL::cos(q3);
	temp_a = m_a3 * c3 + m_a2;
	temp_b = m_a3 * s3;
	temp_c = k5;
	temp_d = k6;
	EcReal q2 = KDL::atan2(temp_a * temp_d - temp_b * temp_c, temp_a * temp_c + temp_b * temp_d); // q2

	EcReal s234 = -s6 * (nx * c1 + ny * s1) - c6 * (ox * c1 + oy * s1);
	EcReal c234 = oz * c6 + nz * s6;
	EcReal q4 = KDL::atan2(s234, c234) - q2 - q3; // q4

	// 将q4的值域由[-3PI,3PI]归并到[-2PI,2PI]
	if (q4 < -2 * PI)
	{
		q4 = q4 + 2 * PI;
	}
	else if (q4 > 2 * PI)
	{
		q4 = q4 - 2 * PI;
	}
	EcReal qIK[6] = { q1, q2, q3, q4, q5, q6 };
	normalize4ConfigHolding(refJoint, qIK, targetJoint);
	for (int i = 0; i < NUMOFJOINTS6; i++)
	{
		if ((targetJoint[i] < m_lowerJointLimit[i]) || (targetJoint[i] > m_upperJointLimit[i]))
		{
			// std::cout << "Out of Limit." << std::endl;
			return ikState_outofLimit; // 后续可替换为用inverseKinematics重新逆解
		}
	}
	return ikState_normal;
}

int URKinematics::normalize(const EcRealVector& current, double q[][6], EcRealVectorVector& qVector)
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
	return 0;
}

void URKinematics::getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian)
{
	/**/
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


void URKinematics::getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot)
{
	/**/
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


EcReal URKinematics::getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian)
{
	Jacobian.resize(NUMOFJOINTS6, NUMOFJOINTS6);
	EcReal s1 = KDL::sin(jointPosition[0]), s2 = KDL::sin(jointPosition[1]), s3 = KDL::sin(jointPosition[2]), s4 = KDL::sin(jointPosition[3]), s5 = KDL::sin(jointPosition[4]), s6 = KDL::sin(jointPosition[5]);
	EcReal c1 = KDL::cos(jointPosition[0]), c2 = KDL::cos(jointPosition[1]), c3 = KDL::cos(jointPosition[2]), c4 = KDL::cos(jointPosition[3]), c5 = KDL::cos(jointPosition[4]), c6 = KDL::cos(jointPosition[5]);

	Jacobian << m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 - m_d5 * (-s1 * s2 * s3 + s1 * c2 * c3) * s4 + m_d5 * (-s1 * s2 * c3 - s1 * s3 * c2) * c4 + m_a3 * s1 * s2 * s3 - m_a3 * s1 * c2 * c3 - m_a2 * s1 * c2 + m_d6 * c1 * c5 + m_d4 * c1, -(-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4 + m_a3 * s2 * c3 + m_a2 * s2 + m_a3 * s3 * c2) * c1, -(-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4 + m_a3 * s2 * c3 + m_a3 * s3 * c2) * c1, -(-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4) * c1, -(-(-s2 * s3 + c2 * c3) * c4 + (s2 * c3 + s3 * c2) * s4) * (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 + m_d5 * (-s1 * s2 * s3 + s1 * c2 * c3) * s4 - m_d5 * (-s1 * s2 * c3 - s1 * s3 * c2) * c4 - m_d6 * c1 * c5) + ((-s1 * s2 * s3 + s1 * c2 * c3) * s4 - (-s1 * s2 * c3 - s1 * s3 * c2) * c4) * (-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4), -m_d6 * (-((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 - c1 * c5) * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 + (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 - m_d6 * c1 * c5) * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5,
		-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d5 * (-s2 * s3 * c1 + c1 * c2 * c3) * s4 - m_d5 * (-s2 * c1 * c3 - s3 * c1 * c2) * c4 + m_d6 * s1 * c5 + m_d4 * s1 - m_a3 * s2 * s3 * c1 + m_a3 * c1 * c2 * c3 + m_a2 * c1 * c2, -(-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4 + m_a3 * s2 * c3 + m_a2 * s2 + m_a3 * s3 * c2) * s1, -(-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4 + m_a3 * s2 * c3 + m_a3 * s3 * c2) * s1, -(-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4) * s1, (-(-s2 * s3 + c2 * c3) * c4 + (s2 * c3 + s3 * c2) * s4)* (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d5 * (-s2 * s3 * c1 + c1 * c2 * c3) * s4 - m_d5 * (-s2 * c1 * c3 - s3 * c1 * c2) * c4 + m_d6 * s1 * c5) - ((-s2 * s3 * c1 + c1 * c2 * c3) * s4 - (-s2 * c1 * c3 - s3 * c1 * c2) * c4) * (-m_d6 * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5 - m_d5 * (-s2 * s3 + c2 * c3) * c4 + m_d5 * (s2 * c3 + s3 * c2) * s4), m_d6* (-((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + s1 * c5)* ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4)* s5 - (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d6 * s1 * c5) * ((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5,
		0, (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 + m_d5 * (-s1 * s2 * s3 + s1 * c2 * c3) * s4 - m_d5 * (-s1 * s2 * c3 - s1 * s3 * c2) * c4 - m_a3 * s1 * s2 * s3 + m_a3 * s1 * c2 * c3 + m_a2 * s1 * c2 - m_d6 * c1 * c5 - m_d4 * c1)* s1 + (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d5 * (-s2 * s3 * c1 + c1 * c2 * c3) * s4 - m_d5 * (-s2 * c1 * c3 - s3 * c1 * c2) * c4 + m_d6 * s1 * c5 + m_d4 * s1 - m_a3 * s2 * s3 * c1 + m_a3 * c1 * c2 * c3 + m_a2 * c1 * c2) * c1, (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 + m_d5 * (-s1 * s2 * s3 + s1 * c2 * c3) * s4 - m_d5 * (-s1 * s2 * c3 - s1 * s3 * c2) * c4 - m_a3 * s1 * s2 * s3 + m_a3 * s1 * c2 * c3 - m_d6 * c1 * c5 - m_d4 * c1)* s1 + (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d5 * (-s2 * s3 * c1 + c1 * c2 * c3) * s4 - m_d5 * (-s2 * c1 * c3 - s3 * c1 * c2) * c4 + m_d6 * s1 * c5 + m_d4 * s1 - m_a3 * s2 * s3 * c1 + m_a3 * c1 * c2 * c3) * c1, (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 + m_d5 * (-s1 * s2 * s3 + s1 * c2 * c3) * s4 - m_d5 * (-s1 * s2 * c3 - s1 * s3 * c2) * c4 - m_d6 * c1 * c5 - m_d4 * c1)* s1 + (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d5 * (-s2 * s3 * c1 + c1 * c2 * c3) * s4 - m_d5 * (-s2 * c1 * c3 - s3 * c1 * c2) * c4 + m_d6 * s1 * c5 + m_d4 * s1) * c1, -((-s1 * s2 * s3 + s1 * c2 * c3) * s4 - (-s1 * s2 * c3 - s1 * s3 * c2) * c4) * (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d5 * (-s2 * s3 * c1 + c1 * c2 * c3) * s4 - m_d5 * (-s2 * c1 * c3 - s3 * c1 * c2) * c4 + m_d6 * s1 * c5) + ((-s2 * s3 * c1 + c1 * c2 * c3) * s4 - (-s2 * c1 * c3 - s3 * c1 * c2) * c4) * (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 + m_d5 * (-s1 * s2 * s3 + s1 * c2 * c3) * s4 - m_d5 * (-s1 * s2 * c3 - s1 * s3 * c2) * c4 - m_d6 * c1 * c5), -(-((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 - c1 * c5) * (-m_d6 * ((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + m_d6 * s1 * c5) + (-m_d6 * ((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 - m_d6 * c1 * c5) * (-((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + s1 * c5),
		0, s1, s1, s1, (-s2 * s3 * c1 + c1 * c2 * c3)* s4 - (-s2 * c1 * c3 - s3 * c1 * c2) * c4, -((-s2 * s3 * c1 + c1 * c2 * c3) * c4 + (-s2 * c1 * c3 - s3 * c1 * c2) * s4) * s5 + s1 * c5,
		0, -c1, -c1, -c1, (-s1 * s2 * s3 + s1 * c2 * c3)* s4 - (-s1 * s2 * c3 - s1 * s3 * c2) * c4, -((-s1 * s2 * s3 + s1 * c2 * c3) * c4 + (-s1 * s2 * c3 - s1 * s3 * c2) * s4) * s5 - c1 * c5,
		1, 0, 0, 0, -(-s2 * s3 + c2 * c3) * c4 + (s2 * c3 + s3 * c2) * s4, -((-s2 * s3 + c2 * c3) * s4 + (s2 * c3 + s3 * c2) * c4) * s5;
	return Jacobian.determinant();
}

Vector URKinematics::vectorCrossOperator(const Vector& v1, const Vector& v2)
{
	return{ v1(1) * v2(2) - v2(1) * v1(2), v1(2) * v2(0) - v1(0) * v2(2), v1(0) * v2(1) - v2(0) * v1(1) };
}

void URKinematics::configuration(const EcRealVector& refJoint, int globalConf[3]) // 目前要求q1/q3/q5∈[-2PI,2PI]
{
	// 确定肩部(Hinge)臂型
	EcReal wx1 = m_a2 * KDL::cos(refJoint[1]) + m_a3 * KDL::cos(refJoint[1] + refJoint[2]) + m_d5 * KDL::sin(refJoint[1] + refJoint[2] + refJoint[3]);
	globalConf[0] = (wx1 < 0) ? 1 : -1; // 定义零位时的臂型为+1；根本原因是alpha2的符号恒为正，而不是恒为负；

	// 确定肘部/腕部(Pivot)臂型, 由于UR关节3和关节5的关节极限为[-2PI,2PI]，需要先归并到[-PI,PI]
	EcReal q3 = refJoint[2];
	EcReal q5 = refJoint[4];
	if ( q3 < -PI)
	{
		q3 = q3 + 2 * PI;
	}
	else if (q3 > PI)
	{
		q3 = q3 - 2 * PI;
	}
	if (q5 < -PI)
	{
		q5 = q5 + 2 * PI;
	}
	else if (q5 > PI)
	{
		q5 = q5 - 2 * PI;
	}
	globalConf[1] = (q3 < 0) ? -1 : 1;
	globalConf[2] = (q5 < 0) ? -1 : 1;
}

int URKinematics::normalize4ConfigHolding(const EcRealVector& current, double q[NUMOFJOINTS6], EcRealVector& qVector)
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