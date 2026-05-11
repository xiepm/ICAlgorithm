#include "hic_controller/kinematics/palletKinematics.h"
#include <iostream>


palletKinematics::palletKinematics()
{
	m_d1 = 0.089159;
	m_d4 = 0.10915;
	m_d5 = 0.09465;
	m_a2 = -0.425;
	m_a3 = -0.39225;

	palletDHParameters();

	m_upperJointLimit.assign(NUMOFJOINTS5, PI);
	m_lowerJointLimit.assign(NUMOFJOINTS5, -PI);
	m_toolFrame = Frame::Identity();

	KDL::Vector v;
	m_zAxis.assign(NUMOFJOINTS5 + 1, v);
	m_pAxis.assign(NUMOFJOINTS5 + 1, v);
	m_velAxis.assign(NUMOFJOINTS5 + 1, v);

	m_pdotAxis.assign(NUMOFJOINTS5 + 1, v);
	m_omega.assign(NUMOFJOINTS5 + 1, v);
}


palletKinematics::~palletKinematics()
{

}

// 入参kinematcisParam: [d1 d2 d3 d4 d5 d6 a2 a3]
void palletKinematics::setRobotDHParameters
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

	palletDHParameters();
}


void palletKinematics::palletDHParameters()
{
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
	m_mk2DH.d[5] = 0;

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
	m_mk2DH.alpha[4] = 0;
	m_mk2DH.alpha[5] = 0;
}

void palletKinematics::setJointMotionLimit
(
	const EcRealVector& upperJointLimit,
	const EcRealVector& lowerJointLimit
)
{   
	for (int i = 0; i < NUMOFJOINTS5; i++)
	{
		m_upperJointLimit[i] = upperJointLimit[i] + epsilon; // 避免由于数值传参损失精度导致边界解丢失
		m_lowerJointLimit[i] = lowerJointLimit[i] - epsilon;
	}
}

void palletKinematics::setToolCoordinateSystem
(
	const EcFrame& tool
)
{
	m_toolFrame = tool;
}

Frame palletKinematics::forwardKinematics(const EcRealVector& jointPositions)
{
	Frame T;
	EcReal theta[NUMOFJOINTS5];
	for (int i = 0; i < NUMOFJOINTS5; i++)
	{
		theta[i] = jointPositions[i] + m_mk2DH.theta[i];
	}

	for (int i = 0; i < NUMOFJOINTS5; i++)
	{
		T = T * Frame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], theta[i]);
	}

	T = T * m_toolFrame;
	return T;
}

void palletKinematics::forwardKinematics2
(
	const EcRealVector& q,
	EcFrame& fkFrame
)
{
	EcReal sq0, sq1, sq2, sq3, sq4, cq0, cq1, cq2, cq3, cq4;
	sq0 = KDL::sin(q[0]); cq0 = KDL::cos(q[0]);
	sq1 = KDL::sin(q[1]); cq1 = KDL::cos(q[1]);
	sq2 = KDL::sin(q[2]); cq2 = KDL::cos(q[2]);
	sq3 = KDL::sin(q[3]); cq3 = KDL::cos(q[3]);
	sq4 = KDL::sin(q[4]); cq4 = KDL::cos(q[4]);

	fkFrame.M = Rotation(
	 sq0 * sq4 + cq4 * (cq3 * (cq0 * cq1 * cq2 - cq0 * sq1 * sq2) - sq3 * (cq0 * cq1 * sq2 + cq0 * cq2 * sq1)), cq4 * sq0 - sq4 * (cq3 * (cq0 * cq1 * cq2 - cq0 * sq1 * sq2) - sq3 * (cq0 * cq1 * sq2 + cq0 * cq2 * sq1)), cq3 * (cq0 * cq1 * sq2 + cq0 * cq2 * sq1) + sq3 * (cq0 * cq1 * cq2 - cq0 * sq1 * sq2),
	-cq0 * sq4 - cq4 * (cq3 * (sq0 * sq1 * sq2 - cq1 * cq2 * sq0) + sq3 * (cq1 * sq0 * sq2 + cq2 * sq0 * sq1)), sq4 * (cq3 * (sq0 * sq1 * sq2 - cq1 * cq2 * sq0) + sq3 * (cq1 * sq0 * sq2 + cq2 * sq0 * sq1)) - cq0 * cq4, cq3 * (cq1 * sq0 * sq2 + cq2 * sq0 * sq1) - sq3 * (sq0 * sq1 * sq2 - cq1 * cq2 * sq0),
	cq4 * (cq3 * (cq1 * sq2 + cq2 * sq1) + sq3 * (cq1 * cq2 - sq1 * sq2)), -sq4 * (cq3 * (cq1 * sq2 + cq2 * sq1) + sq3 * (cq1 * cq2 - sq1 * sq2)), sq3 * (cq1 * sq2 + cq2 * sq1) - cq3 * (cq1 * cq2 - sq1 * sq2)
	);
	fkFrame.p = { m_d5 * (cq3 * (cq0 * cq1 * sq2 + cq0 * cq2 * sq1) + sq3 * (cq0 * cq1 * cq2 - cq0 * sq1 * sq2)) + m_d4 * sq0 + m_a2 * cq0 * cq1 + m_a3 * cq0 * cq1 * cq2 - m_a3 * cq0 * sq1 * sq2,
				  m_d5 * (cq3 * (cq1 * sq0 * sq2 + cq2 * sq0 * sq1) - sq3 * (sq0 * sq1 * sq2 - cq1 * cq2 * sq0)) - m_d4 * cq0 + m_a2 * cq1 * sq0 + m_a3 * cq1 * cq2 * sq0 - m_a3 * sq0 * sq1 * sq2,
				  m_d1 + m_a2 * sq1 - m_d5 * (cq3 * (cq1 * cq2 - sq1 * sq2) - sq3 * (cq1 * sq2 + cq2 * sq1)) + m_a3 * cq1 * sq2 + m_a3 * cq2 * sq1
	};

	fkFrame = fkFrame * m_toolFrame;
}

ENInverseKineState palletKinematics::inverseKinematics(const Frame& targetPose, const EcRealVector& refJoint, EcRealVector& targetJoint)
{
	targetJoint = refJoint; // 默认返回参考角
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

	// 初始化全逆解的二维数组
	EcReal q[NUMOFSOLVED4][NUMOFJOINTS5] = { 0 };
	for (int i = 0; i < NUMOFSOLVED4; i++)
	{
		q[i][3] = INFINITE;  // 将q4置为是否可解的标志位
	}
	EcBoolean palletState = false;  // 末端位姿是否处于码垛状态的标志

	EcReal s1, c1, s2, c2, s3, c3, s5, c5, s6, c6, s234, c234;
	EcReal ka, kb, kc, kd;
	EcU32 halfNumOfSolutions = NUMOFSOLVED4 / 2;

	// q1
	EcReal consistentCondition = fabs(fabs(px * ay - py * ax) - m_d4 * KDL::sqrt(ax * ax + ay * ay));  // 位姿相容性条件
	if (consistentCondition < 0.00001)
	{
		if ((ax * ax + ay * ay) < EPSILON2)  // 码垛状态基于位置约束处理多解情形
		{
			EcReal dSquare = px * px + py * py - m_d4 * m_d4;
			if (dSquare < EPSILON2)
			{
				// std::cout << "Shoulder Singularity，or Ouside of Workspace!" << std::endl; // 目标点落入黑柱内
				targetJoint = refJoint;
				return ikState_noSolution;
			}
			EcReal d = KDL::sqrt(dSquare);
			EcReal alpha = KDL::atan2(px, -py);  // [-PI, PI]
			EcReal beta = KDL::atan2(d, m_d4);  // [0,PI/2)

			for (int i = 0; i < halfNumOfSolutions; i++)
			{
				q[i][0] = alpha + beta;
				q[i + halfNumOfSolutions][0] = alpha - beta;
			}

			palletState = true;
		}
		else  // 非码垛状态基于姿态约束处理单解情形
		{
			if ((px * ay - py * ax) > 0)
			{
				for (int i = 0; i < halfNumOfSolutions; i++)
				{
					q[i][0] = KDL::atan2(ay, ax);
				}
			}
			else if ((px * ay - py * ax) < 0)
			{
				for (int i = 0; i < halfNumOfSolutions; i++)
				{
					q[i][0] = KDL::atan2(-ay, -ax);
				}
			}
			else
			{
				// 暂不处理(事实上，在m_d4的值大于1e-6时，永远不会进入该条件分支)
			}

			palletState = false;
		}
	}
	else
	{
		// std::cout << "Pose is Inconsistent!" << std::endl; // 位姿不相容
		targetJoint = refJoint;
		return ikState_noSolution;
	}

	EcU32Vector shoulderRowSet = { 0 }; // 基于肩部臂型区分的全逆解数组的起始行
	if (palletState)
	{
		shoulderRowSet.push_back(halfNumOfSolutions);  // 码垛状态有两个起始行
	}

	for (EcU32 startRow : shoulderRowSet)  // 采用值拷贝而非传引用，避免对遍历元素的修改
	{
		s1 = KDL::sin(q[startRow][0]);  c1 = KDL::cos(q[startRow][0]);
		// q5
		s5 = nx * s1 - ny * c1;
		c5 = ox * s1 - oy * c1;
		for (int i = 0; i < halfNumOfSolutions; i++)
		{
			q[startRow + i][4] = KDL::atan2(s5, c5);
		}

		// q2/q3/q4
		int GCE; // 肘部臂型
		EcReal r, cTriangle;
		EcReal q2, q3, q4;
		kc = px * c1 + py * s1 - m_d5 * (ax * c1 + ay * s1);
		kd = pz - m_d1 - az * m_d5;
		r = KDL::sqrt(kc * kc + kd * kd);

		if (r > fabs(m_a2 + m_a3) || r < fabs(m_a2 - m_a3)) // 超出工作空间
		{
			// std::cout << "startRow:" << startRow << " , Ouside of Workspace!" << std::endl;
			for (int i = 0; i < halfNumOfSolutions; i++)
			{
				q[startRow + i][1] = INFINITE; q[startRow + i][2] = INFINITE; q[startRow + i][3] = INFINITE;
			}
		}
		else
		{
			cTriangle = (m_a2 * m_a2 + m_a3 * m_a3 - r * r) / (2 * m_a2 * m_a3);
			if (fabs(cTriangle) >= 1 - EPSILON2) // 肘部奇异附近
			{
				// std::cout << "startRow:" << startRow << " , Elbow Singularity!" << std::endl;
				cTriangle = KDL::sign(cTriangle);
			}
			for (int i = 0; i < halfNumOfSolutions; i++)
			{
				GCE = 1 - 2 * i;
				q3 = GCE * (PI - KDL::acos(cTriangle));  // q3
				s3 = KDL::sin(q3);	c3 = KDL::cos(q3);

				ka = m_a3 * c3 + m_a2;
				kb = m_a3 * s3;
				q2 = KDL::atan2(ka * kd - kb * kc, ka * kc + kb * kd);  // q2

				s234 = ax * c1 + ay * s1;
				c234 = -az;
				q4 = KDL::atan2(s234, c234) - q2 - q3;  // q4

				q[startRow + i][1] = q2; q[startRow + i][2] = q3; q[startRow + i][3] = q4;
			}
		}

	}

	// 检查是否有解
	int count = 0;
	for (int i = 0; i < NUMOFSOLVED4; i++) // 将q4置为是否可解的标志位
	{
		if (INFINITE == q[i][3])
		{
			count = count + 1;
		}
	}
	if (NUMOFSOLVED4 == count)
	{
		targetJoint = refJoint;
		return ikState_noSolution;
	}

	EcRealVectorVector jointPositions(NUMOFSOLVED4, EcRealVector(NUMOFJOINTS5, 0.0));
	normalize(refJoint, q, jointPositions);

	// 根据行程最短原则选解，后续整理为单独的函数
	EcReal faux0, faux1, faux2;
	EcRealVector sum1(NUMOFSOLVED4), sum2(NUMOFSOLVED4), sum3(NUMOFSOLVED4), sum5(NUMOFSOLVED4);
	for (int i = 0; i < NUMOFSOLVED4; i++)
	{
		faux0 = jointPositions[i][0] - refJoint[0];
		sum1[i] = faux0 * faux0;

		faux1 = jointPositions[i][1] - refJoint[1];
		sum2[i] = sum1[i] + faux1 * faux1;

		faux2 = jointPositions[i][2] - refJoint[2];
		sum3[i] = sum2[i] + faux2 * faux2;

		faux0 = jointPositions[i][3] - refJoint[3];
		faux1 = jointPositions[i][4] - refJoint[4];
		sum5[i] = sum3[i] + faux0 * faux0 + faux1 * faux1;
	}
	// 依次比较总行程/前三个关节行程/前两个关节行程/第一个关节的行程
	int out_index = 0;
	for (int i = 1; i < NUMOFSOLVED4; i++)
	{
		if (fabs(sum5[out_index] - sum5[i]) < EPSILON2)
		{
			if (fabs(sum3[out_index] - sum3[i]) < EPSILON2)
			{
				if (fabs(sum2[out_index] - sum2[i]) < EPSILON2)
				{
					if (sum1[out_index] > sum1[i])
					{
						out_index = i;
					}
				}
				else if (sum2[out_index] > sum2[i])
				{
					out_index = i;
				}
			}
			else if (sum3[out_index] > sum3[i])
			{
				out_index = i;
			}
		}
		else if (sum5[out_index] > sum5[i])
		{
			out_index = i;
		}
	}
	for (int i = 0; i < NUMOFJOINTS5; i++)
	{
		targetJoint[i] = jointPositions[out_index][i];
	}
	targetJoint[5] = 0;  // 关节6默认为0

	// 检查关节超限
	for (int i = 0; i < NUMOFJOINTS5; i++)
	{
		if ((targetJoint[i] < m_lowerJointLimit[i]) || (targetJoint[i] > m_upperJointLimit[i]))
		{
			return ikState_outofLimit;
		}
	}

	// 逆解连续性判据(待商榷)
	EcReal maxDisplacement = PI / 6;  // 暂无理论根据
	EcReal sumDisplacement = 0;
	for (int i = 0; i < NUMOFJOINTS5; i++)
	{
		sumDisplacement += fabs(targetJoint[i] - refJoint[i]);
	}
	if (sumDisplacement > maxDisplacement)
	{
		return ikState_notContinuous;
	}

	return ikState_normal;
}

void palletKinematics::normalize(const EcRealVector& refJoint, EcReal qArray[][NUMOFJOINTS5], EcRealVectorVector& qVector)
{
	EcRealVector standardSolution(NUMOFJOINTS5, 0.0);
	EcRealVector expandedSolution(NUMOFJOINTS5, 0.0);
	EcRealVector currentDatumPoint(NUMOFJOINTS5, 0.0);

	// [-pi,pi]内的standardSolution和[-pi,pi]外的expandedSolution，以及其他平移2PI后的解，统称为periodicSolution
	for (int i = 0; i < NUMOFSOLVED4; i++)
	{
		// 获取refJoint对应的基准点
		for (int j = 0; j < NUMOFJOINTS5; j++)
		{
			int temp = fabs(refJoint[j]) / PI;
			currentDatumPoint[j] = sign(refJoint[j]) * (temp + temp % 2) * PI;  // 获取距离refJoint最近的2kPI作为[-3PI,3PI]平移的参考基准点
		}
		// 先得到refJoint附近的标准解;
		for (int j = 0; j < NUMOFJOINTS5; j++)
		{
			// 获取[-pi,pi]内的标准解
			if (qArray[i][j] > PI)
			{
				standardSolution[j] = qArray[i][j] - 2.0 * PI;
			}
			else if (qArray[i][j] <= -PI)
			{
				standardSolution[j] = qArray[i][j] + 2.0 * PI;
			}
			else
			{
				standardSolution[j] = qArray[i][j];
			}
			standardSolution[j] += currentDatumPoint[j]; // 标准解平移至refJoint附近
		}
		// 再得到refJoint附近的最近拓展解（单侧）;
		for (int j = 0; j < NUMOFJOINTS5; j++)
		{
			// 由于refJoint一定满足关节极限，故仅需考虑refJoint左/右最邻近的两个周期解，其中一个一定为标准解,另一个为另一侧的拓展解
			if (standardSolution[j] > refJoint[j]) 
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
			else // 如果标准解和拓展解，两者同时满足/同时不满足关节极限，则选择距离refJoint较近的解作为normalize的输出
			{
				qVector[i][j] = (fabs(standardSolution[j] - refJoint[j]) < fabs(expandedSolution[j] - refJoint[j])) ? standardSolution[j] : expandedSolution[j];
			}
		}
	}
}

void palletKinematics::getJacobianWithToolPointMatrix(const VectorXd& jointPosition, Eigen::MatrixXd& Jacobian)
{
	EcReal calcAcs[NUMOFJOINTS5];
	Jacobian.resize(DIMOFTASKSPACE6, NUMOFJOINTS5);
	for (EcSizeT i = 0; i < NUMOFJOINTS5; i++)
	{
		calcAcs[i] = jointPosition(i) + m_mk2DH.theta[i];
	}
	Frame T = Frame::Identity();
	for (EcSizeT i = 0; i < NUMOFJOINTS5; i++)
	{
		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;
		T = T * Frame::DH(m_mk2DH.a[i], m_mk2DH.alpha[i], m_mk2DH.d[i], calcAcs[i]);
	}
	T = T * m_toolFrame;

	for (EcSizeT i = 0; i < NUMOFJOINTS5; i++)
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

void palletKinematics::getJacobianDotWithToolMatrix(const VectorXd& jointPosition, const VectorXd& jointVel, Eigen::MatrixXd& JacobianDot)
{
	EcReal calcAcs[NUMOFJOINTS5];
	JacobianDot.resize(DIMOFTASKSPACE6, NUMOFJOINTS5);
	for (EcSizeT i = 0; i < NUMOFJOINTS5; i++)
	{
		calcAcs[i] = jointPosition(i) + m_mk2DH.theta[i];
	}

	Frame T = Frame::Identity();
	m_zAxis[0] = T.M.UnitZ();
	m_pAxis[0] = T.p;
	for (EcSizeT i = 1; i < NUMOFJOINTS5 + 1; i++)
	{
		T = T * Frame::DH(m_mk2DH.a[i - 1], m_mk2DH.alpha[i - 1], m_mk2DH.d[i - 1], calcAcs[i - 1]);
		if (i == NUMOFJOINTS5)
			T = T * m_toolFrame;			// 叠加工具坐标系的偏置

		m_zAxis[i] = T.M.UnitZ();
		m_pAxis[i] = T.p;

		m_omega[i] = jointVel(i - 1) * m_zAxis[i - 1] + m_omega[i - 1];
		m_pdotAxis[i] = m_pdotAxis[i - 1] + vectorCrossOperator(m_omega[i], m_pAxis[i] - m_pAxis[i - 1]);
	}


	for (EcSizeT i = 0; i < NUMOFJOINTS5; i++)
	{
		//JvDot(:,i) = [cross(cross(m_omega(:,i),m_zAxis(:,i)),(m_pAxis(:,7)-m_pAxis(:,i)))+cross(m_zAxis(:,i),m_pdotAxis(:,7)-m_pdotAxis(:,i));
		// cross(m_omega(:, i), m_zAxis(:, i))];
		m_velAxis[i] = T.p - m_pAxis[i];
		Vector z = vectorCrossOperator(vectorCrossOperator(m_omega[i], m_zAxis[i]), (m_pAxis[NUMOFJOINTS5] - m_pAxis[i])) + vectorCrossOperator(m_zAxis[i], m_pdotAxis[NUMOFJOINTS5] - m_pdotAxis[i]);
		Vector w = vectorCrossOperator(m_omega[i], m_zAxis[i]);

		JacobianDot(0, i) = z[0];
		JacobianDot(1, i) = z[1];
		JacobianDot(2, i) = z[2];

		JacobianDot(3, i) = w[0];
		JacobianDot(4, i) = w[1];
		JacobianDot(5, i) = w[2];
	}

}

EcReal palletKinematics::getJacobianMatrix(const EcRealVector& jointPosition, Eigen::MatrixXd& Jacobian)
{
	Jacobian.resize(DIMOFTASKSPACE6, NUMOFJOINTS5);
	EcReal s1 = KDL::sin(jointPosition[0]), s2 = KDL::sin(jointPosition[1]), s3 = KDL::sin(jointPosition[2]), s4 = KDL::sin(jointPosition[3]), s5 = KDL::sin(jointPosition[4]);
	EcReal c1 = KDL::cos(jointPosition[0]), c2 = KDL::cos(jointPosition[1]), c3 = KDL::cos(jointPosition[2]), c4 = KDL::cos(jointPosition[3]), c5 = KDL::cos(jointPosition[4]);
	EcReal s23 = KDL::sin(jointPosition[1] + jointPosition[2]), c23 = KDL::cos(jointPosition[1] + jointPosition[2]);
	EcReal s234 = KDL::sin(jointPosition[1] + jointPosition[2] + jointPosition[3]), c234 = KDL::cos(jointPosition[1] + jointPosition[2] + jointPosition[3]);

	Jacobian << m_d4 * c1 - m_d5 * (c4 * (c2 * s1 * s3 + c3 * s1 * s2) - s4 * (s1 * s2 * s3 - c2 * c3 * s1)) - m_a2 * c2 * s1 - m_a3 * c2 * c3 * s1 + m_a3 * s1 * s2 * s3, -c1 * (m_a3 * s23 + m_a2 * s2 - m_d5 * c234), -c1 * (m_a3 * s23 - m_d5 * c234), m_d5 * c234 * c1, 0,
		m_d5 * (c4 * (c1 * c2 * s3 + c1 * c3 * s2) + s4 * (c1 * c2 * c3 - c1 * s2 * s3)) + m_d4 * s1 + m_a2 * c1 * c2 + m_a3 * c1 * c2 * c3 - m_a3 * c1 * s2 * s3, -s1 * (m_a3 * s23 + m_a2 * s2 - m_d5 * c234), -s1 * (m_a3 * s23 - m_d5 * c234), m_d5 * c234 * s1, 0,
		0, m_a3 * c23 + m_a2 * c2 + m_d5 * s234, m_a3 * c23 + m_d5 * s234, m_d5 * s234, 0,
		0, s1, s1, s1, s234 * c1,
		0, -c1, -c1, -c1, s234 * s1,
		1, 0, 0, 0, -c234;

	Eigen::MatrixXd gramianMatrix = Jacobian.transpose() * Jacobian;   // 引入Gramian矩阵描述6×5雅克比的奇异性

	return KDL::sqrt(gramianMatrix.determinant());
}

EcReal palletKinematics::getJacobianConditionNum(const EcRealVector& jointPosition) 
{
	EcRealMatrixX Jacobian(DIMOFTASKSPACE6, NUMOFJOINTS5);
	getJacobianMatrix(jointPosition, Jacobian);
	EcRealMatrixX gramianMatrix = Jacobian.transpose() * Jacobian;   // 引入Gramian矩阵描述6×5雅克比的奇异性

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(gramianMatrix);
	EcReal minSingularValue = svd.singularValues()(svd.singularValues().size() - 1);
	EcReal conditionNum = (minSingularValue == 0) ? INFINITE : (svd.singularValues()(0) / minSingularValue);
	return KDL::sqrt(conditionNum);
};

Vector palletKinematics::vectorCrossOperator(const Vector& v1, const Vector& v2)
{
	return{ v1(1) * v2(2) - v2(1) * v1(2), v1(2) * v2(0) - v1(0) * v2(2), v1(0) * v2(1) - v2(0) * v1(1) };
}