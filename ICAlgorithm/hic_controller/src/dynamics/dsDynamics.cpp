#include "hic_controller/dynamics/dsDynamics.h"
#include <iostream>

dsDynamics::dsDynamics()
{
	m_NumJoints = 6;
	m_gx = 0;
	m_gy = 0;
	m_gz = -9.81;

	m_d1 = 0.220;		//默认设置为ElfinV5的杆长参数；
	m_d4 = 0.420;
	m_d6 = 0.180;
	m_a2 = 0.380;

	m_friTemperaturesParams.assign(m_NumJoints, 1.0);
	m_jointTemperatures.assign(m_NumJoints, 46.0);


	m_zeros.assign(m_NumJoints, 0.0);
}


dsDynamics::~dsDynamics()
{

}



void dsDynamics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

}


void dsDynamics::setGravityVector
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
}

void dsDynamics::calculateGravityJointTorques
(
	const EcRealVector& q,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	calculateEstimateJointToqrues(q, m_zeros, m_zeros, parms, tau);
}


// calculate forward dynamics of elfin
EcBoolean dsDynamics::calculateEstimateJointToqrues
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	if (
		q.size() != m_NumJoints ||
		dq.size() != m_NumJoints ||
		ddq.size() != m_NumJoints)
	{
		return false;
	}
	double x0 = cos(q[1]);
	double x1 = -x0;
	double x2 = -dq[0];
	double x3 = x1 * x2;
	double x4 = m_a2 * dq[1];
	double x5 = sin(q[1]);
	double x6 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x5) * (x5));
	double x7 = -ddq[0];
	double x8 = x5 * x7;
	double x9 = cos(q[0]);
	double x10 = sin(q[0]);
	double x11 = m_gx * x10 - m_gy * x9;
	double x12 = x11 + x3 * x4 + x6 * x8;
	double x13 = x2 * x5;
	double x14 = -parms[19];
	double x15 = x13 * x6;
	double x16 = parms[20] * x3 + parms[22] * x15 + x13 * x14;
	double x17 = dq[1] * x13 + x1 * x7;
	double x18 = -m_gy * x10 - m_gx * x9;
	double x19 = m_a2 * ddq[1] + x0 * x18 + m_gz * x5;
	double x20 = -parms[21];
	double x21 = cos(q[2]);
	double x22 = ddq[1] + ddq[2];
	double x23 = cos(q[4]);
	double x24 = -x23;
	double x25 = -dq[3];
	double x26 = -x25;
	double x27 = dq[1] + dq[2];
	double x28 = sin(q[3]);
	double x29 = x27 * x28;
	double x30 = -x28;
	double x31 = x21 * x4;
	double x32 = sin(q[2]);
	double x33 = -x3;
	double x34 = x13 * x21 + x32 * x33;
	double x35 = cos(q[3]);
	double x36 = x34 * x35;
	double x37 = x15 * x35 - m_d4 * x29 + x30 * x31 - m_d4 * x36;
	double x38 = x22 * x35;
	double x39 = -m_gz * x0 + x18 * x5;
	double x40 = -x32;
	double x41 = x32 * x4;
	double x42 = -dq[2];
	double x43 = x19 * x21 + x39 * x40 + x41 * x42;
	double x44 = -m_d4 * x28;
	double x45 = x13 * x32 + x21 * x3;
	double x46 = dq[1] * x33 + x8;
	double x47 = x17 * x40 + x21 * x46 + x42 * x45;
	double x48 = x12 * x28 + x26 * x37 + x35 * x43 + m_d4 * x38 + x44 * x47;
	double x49 = dq[2] * x31 + x19 * x32 + x21 * x39;
	double x50 = -x49;
	double x51 = sin(q[4]);
	double x52 = x28 * x34;
	double x53 = x27 * x35;
	double x54 = x15 * x28 + x31 * x35 - m_d4 * x52 + m_d4 * x53;
	double x55 = -x41;
	double x56 = x24 * x55 + x51 * x54;
	double x57 = -dq[4];
	double x58 = x23 * x48 + x50 * x51 + x56 * x57;
	double x59 = -parms[59];
	double x60 = x23 * x54 + x51 * x55;
	double x61 = x29 + x36;
	double x62 = -x45;
	double x63 = x25 + x62;
	double x64 = x23 * x61 + x51 * x63;
	double x65 = -parms[60];
	double x66 = x24 * x63 + x51 * x61;
	double x67 = parms[58] * x66 + parms[61] * x37 + x64 * x65;
	double x68 = -x66;
	double x69 = -x52 + x53;
	double x70 = dq[4] + x69;
	double x71 = parms[59] * x68 + parms[60] * x70 + parms[61] * x60;
	double x72 = -x71;
	double x73 = x22 * x28;
	double x74 = x35 * x47;
	double x75 = x12 * x35 + x25 * x54 + x30 * x43 - m_d4 * x73 - m_d4 * x74;
	double x76 = x26 * x69 + x73 + x74;
	double x77 = dq[2] * x34 + x17 * x21 + x32 * x46;
	double x78 = -x77;
	double x79 = -ddq[3] + x78;
	double x80 = x23 * x76 + x51 * x79 + x57 * x66;
	double x81 = parms[52] * x64 + parms[53] * x70 + parms[54] * x66 + parms[59] * x56 + x37 * x65;
	double x82 = cos(q[5]);
	double x83 = sin(q[5]);
	double x84 = x64 * x83;
	double x85 = x70 * x82;
	double x86 = x37 * x83 + x60 * x82 - m_d6 * x84 + m_d6 * x85;
	double x87 = -dq[5];
	double x88 = x25 * x61 + x30 * x47 + x38;
	double x89 = ddq[4] + x88;
	double x90 = x83 * x89;
	double x91 = -x83;
	double x92 = x80 * x82;
	double x93 = x58 * x91 + x75 * x82 + x86 * x87 - m_d6 * x90 - m_d6 * x92;
	double x94 = x70 * x83;
	double x95 = x64 * x82;
	double x96 = x37 * x82 + x60 * x91 - m_d6 * x94 - m_d6 * x95;
	double x97 = -x84 + x85;
	double x98 = dq[5] + x66;
	double x99 = -x98;
	double x100 = parms[72] * x99 + parms[73] * x97 + parms[74] * x86;
	double x101 = dq[4] * x64 + x24 * x79 + x51 * x76;
	double x102 = ddq[5] + x101;
	double x103 = x94 + x95;
	double x104 = -parms[73];
	double x105 = parms[65] * x103 + parms[66] * x97 + parms[67] * x98 + parms[72] * x56 + x104 * x96;
	double x106 = -x97;
	double x107 = parms[71] * x98 + parms[74] * x96 + x103 * x104;
	double x108 = dq[5] * x97 + x90 + x92;
	double x109 = x82 * x89;
	double x110 = -m_d6 * x83;
	double x111 = dq[5] * x96 + m_d6 * x109 + x110 * x80 + x58 * x82 + x75 * x83;
	double x112 = -parms[72];
	double x113 = x103 * x87 + x109 + x80 * x91;
	double x114 = -x56;
	double x115 = parms[66] * x103 + parms[68] * x97 + parms[69] * x98 + parms[71] * x114 + parms[73] * x86;
	double x116 = parms[67] * x108 + parms[69] * x113 + parms[70] * x102 + parms[71] * x93 - x100 * x96 + x103 * x115 + x105 * x106 + x107 * x86 + x111 * x112;
	double x117 = parms[53] * x64 + parms[55] * x70 + parms[56] * x66 + parms[58] * x114 + parms[60] * x60;
	double x118 = parms[54] * x80 + parms[56] * x89 + parms[57] * x101 + parms[58] * x75 + x116 + x117 * x64 + x37 * x72 + x58 * x59 + x60 * x67 - x70 * x81;
	double x119 = -x69;
	double x120 = parms[46] * x119 + parms[47] * x63 + parms[48] * x54;
	double x121 = -parms[47];
	double x122 = parms[39] * x61 + parms[40] * x63 + parms[41] * x69 + parms[46] * x37 + x121 * x55;
	double x123 = -parms[71];
	double x124 = parms[72] * x103 + parms[74] * x56 + x123 * x97;
	double x125 = -x103;
	double x126 = parms[71] * x102 + parms[74] * x93 + x100 * x98 + x104 * x108 + x124 * x125;
	double x127 = x126 * x82;
	double x128 = -x60;
	double x129 = parms[54] * x64 + parms[56] * x70 + parms[57] * x66 + parms[58] * x37 + parms[59] * x128;
	double x130 = -x86;
	double x131 = parms[67] * x103 + parms[69] * x97 + parms[70] * x98 + parms[71] * x96 + parms[72] * x130;
	double x132 = dq[4] * x60 + x24 * x50 + x48 * x51;
	double x133 = parms[65] * x108 + parms[66] * x113 + parms[67] * x102 + parms[72] * x132 + x104 * x93 + x107 * x114 + x115 * x99 + x124 * x96 + x131 * x97;
	double x134 = -parms[58];
	double x135 = parms[59] * x64 + parms[61] * x56 + x134 * x70;
	double x136 = parms[66] * x108 + parms[68] * x113 + parms[69] * x102 + parms[73] * x111 + x100 * x56 + x105 * x98 + x123 * x132 + x124 * x130 + x125 * x131;
	double x137 = parms[73] * x113 + parms[74] * x111 + x102 * x112 + x107 * x99 + x124 * x97;
	double x138 = parms[52] * x80 + parms[53] * x89 + parms[54] * x101 + parms[59] * x132 + x110 * x137 + x114 * x67 + x117 * x68 - m_d6 * x127 + x129 * x70 + x133 * x82 + x135 * x37 + x136 * x91 + x65 * x75;
	double x139 = -x54;
	double x140 = parms[41] * x61 + parms[43] * x63 + parms[44] * x69 + parms[45] * x55 + parms[46] * x139;
	double x141 = -x61;
	double x142 = -parms[45];
	double x143 = parms[46] * x61 + parms[48] * x37 + x142 * x63;
	double x144 = -parms[40] * x76 - parms[42] * x79 - parms[43] * x88 - parms[47] * x48 - x118 * x24 - x120 * x37 - x122 * x69 - x138 * x51 - x139 * x143 - x140 * x141 - x142 * x75;
	double x145 = parms[27] * x34 + parms[29] * x27 + parms[30] * x45 + parms[32] * x55 + parms[34] * x31;
	double x146 = -parms[34];
	double x147 = parms[26] * x34 + parms[27] * x27 + parms[28] * x45 + parms[33] * x41 + x146 * x15;
	double x148 = -x27;
	double x149 = parms[33] * x62 + parms[34] * x27 + parms[35] * x31;
	double x150 = -x15;
	double x151 = parms[32] * x45 + parms[35] * x15 + x146 * x34;
	double x152 = parms[28] * x47 + parms[30] * x22 + parms[31] * x77 + parms[32] * x12 - parms[33] * x43 + x144 + x145 * x34 + x147 * x148 + x149 * x150 + x151 * x31;
	double x153 = parms[45] * x69 + parms[48] * x55 + x121 * x61;
	double x154 = parms[40] * x61 + parms[42] * x63 + parms[43] * x69 + parms[47] * x54 + x142 * x37;
	double x155 = parms[39] * x76 + parms[40] * x79 + parms[41] * x88 + parms[46] * x75 + x118 * x51 + x119 * x154 + x121 * x50 + x138 * x23 + x140 * x63 + x143 * x55 - x153 * x37;
	double x156 = -x120;
	double x157 = -x64;
	double x158 = parms[46] * x76 + parms[48] * x75 + parms[58] * x101 + parms[61] * x75 + x127 + x135 * x157 + x137 * x83 + x142 * x79 + x153 * x61 + x156 * x63 + x65 * x80 + x66 * x71;
	double x159 = x158 * x35;
	double x160 = parms[32] * x148 + parms[33] * x34 + parms[35] * x41;
	double x161 = x137 * x82;
	double x162 = parms[60] * x89 + parms[61] * x58 + x101 * x59 + x126 * x91 + x135 * x70 + x161 + x67 * x68;
	double x163 = parms[59] * x80 + parms[61] * x132 + parms[72] * x108 + parms[74] * x132 + x100 * x106 + x103 * x107 + x113 * x123 + x134 * x89 + x64 * x67 + x70 * x72;
	double x164 = -parms[46];
	double x165 = parms[47] * x79 + parms[48] * x48 + x119 * x153 + x143 * x63 + x162 * x23 + x163 * x51 + x164 * x88;
	double x166 = parms[53] * x80 + parms[55] * x89 + parms[56] * x101 + parms[60] * x58 + x110 * x126 + x128 * x135 + x129 * x157 + x132 * x134 + x133 * x83 + x136 * x82 + m_d6 * x161 + x56 * x71 + x66 * x81;  double x167 = parms[41] * x76 + parms[43] * x79 + parms[44] * x88 + parms[45] * x50 - x122 * x63 + x153 * x54 + x154 * x61 + x156 * x55 + x164 * x48 + x166;
	double x168 = -x31;
	double x169 = parms[28] * x34 + parms[30] * x27 + parms[31] * x45 + parms[32] * x15 + parms[33] * x168;
	double x170 = parms[26] * x47 + parms[27] * x22 + parms[28] * x77 + parms[33] * x49 + x12 * x146 + x145 * x62 + x15 * x160 + x151 * x55 + x155 * x35 - m_d4 * x159 + x165 * x44 + x167 * x30 + x169 * x27;
	double x171 = dq[1] * parms[19] + parms[21] * x33 + parms[22] * x4;
	double x172 = dq[1] * parms[18] + parms[15] * x3 + parms[17] * x13 + parms[19] * x4;
	double x173 = dq[1] * parms[17] + parms[14] * x3 + parms[16] * x13 + parms[19] * x150;
	double x174 = -dq[1];
	double x175 = -x34;
	double x176 = parms[20] * x174 + parms[21] * x13;
	double x177 = -x13;
	double x178 = dq[1] * parms[15] + parms[13] * x3 + parms[14] * x13 + parms[20] * x15 + x20 * x4;
	double x179 = x165 * x35;
	double x180 = parms[27] * x47 + parms[29] * x22 + parms[30] * x77 + parms[32] * x50 + parms[34] * x43 + x147 * x45 + x149 * x41 + x155 * x28 + x158 * x44 + x160 * x168 + x167 * x35 + x169 * x175 + m_d4 * x179;
	//
	tau[0] = ddq[0] * parms[10] + dq[0] * parms[11] + parms[12] * sign(dq[0]) - parms[3] * x7 + parms[6] * x11 - parms[8] * x18 - x1 * (ddq[1] * parms[15] + parms[13] * x17 + parms[14] * x46 + parms[20] * x12 + x13 * x172 + x150 * x171 + x152 * x21 + x16 * x4 + x170 * x40 + x173 * x174 + x19 * x20) - x5 * x6 * (parms[20] * x17 + parms[22] * x12 + parms[32] * x77 + parms[35] * x12 + x14 * x46 + x146 * x47 + x149 * x45 + x159 + x160 * x175 + x165 * x28 + x171 * x3 + x176 * x177) - x5 * (ddq[1] * parms[17] + dq[1] * x178 + parms[14] * x17 + parms[16] * x46 + parms[21] * x39 + x12 * x14 + x15 * x176 + x152 * x32 + x170 * x21 + x172 * x33);
	tau[1] = ddq[1] * parms[18] + m_a2 * ddq[1] * parms[19] + ddq[1] * parms[23] + dq[1] * parms[24] + m_a2 * dq[1] * x176 + parms[15] * x17 + parms[17] * x46 + parms[19] * x19 - parms[20] * x39 + m_a2 * parms[22] * x19 + parms[25] * sign(dq[1]) + m_a2 * x16 * x33 + m_a2 * x17 * x20 + x173 * x3 - x176 * x4 + x177 * x178 + x180 + m_a2 * x21 * (parms[33] * x78 + parms[34] * x22 + parms[35] * x43 + x151 * x62 + x158 * x30 + x160 * x27 + x179) + m_a2 * x32 * (-parms[32] * x22 + parms[33] * x47 + parms[35] * x49 - parms[45] * x88 - parms[48] * x50 - x120 * x69 - x121 * x76 - x141 * x143 + x148 * x149 + x151 * x34 - x162 * x51 - x163 * x24);
	tau[2] = ddq[2] * parms[36] + dq[2] * parms[37] + parms[38] * sign(dq[2]) + x180;
	tau[3] = ddq[3] * parms[49] + dq[3] * parms[50] + parms[51] * sign(dq[3]) + x144;
	tau[4] = ddq[4] * parms[62] + dq[4] * parms[63] + parms[64] * sign(dq[4]) + x166;
	tau[5] = ddq[5] * parms[75] + dq[5] * parms[76] + parms[77] * sign(dq[5]) + x116;

	// 
	// 考虑温度对粘性摩擦力矩的影响
	//		以45°C作为参考标准；
	EcRealVector temperatureFactor(6);
	for (int i = 0; i < m_NumJoints; i++)
	{
		temperatureFactor[i] = (m_friTemperaturesParams[i] - 1.0) * std::pow((46.0 - m_jointTemperatures[i]) / 20.0, 0.9);
		tau[i] += temperatureFactor[i] * dq[i] * parms[13 * i + 11];
	}

	return true;
}


// exclude gravity, friction, motor inertia;
void dsDynamics::calculateMomentumEstimatedJointTorques
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	double x0 = sin(q[1]);
	double x1 = cos(q[1]);
	double x2 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x1) * (x1));
	double x3 = m_a2 * dq[1];
	double x4 = sin(q[2]);
	double x5 = x3 * x4;
	double x6 = -dq[0];
	double x7 = -x1;
	double x8 = x6 * x7;
	double x9 = -x8;
	double x10 = cos(q[2]);
	double x11 = x0 * x6;
	double x12 = x10 * x11 + x4 * x9;
	double x13 = dq[1] + dq[2];
	double x14 = -x13;
	double x15 = parms[26] * x14 + parms[27] * x12 + parms[29] * x5;
	double x16 = -x12;
	double x17 = cos(q[3]);
	double x18 = x13 * x17;
	double x19 = sin(q[3]);
	double x20 = x12 * x19;
	double x21 = x18 - x20;
	double x22 = -x21;
	double x23 = -dq[3];
	double x24 = x10 * x8 + x11 * x4;
	double x25 = -x24;
	double x26 = x23 + x25;
	double x27 = x10 * x3;
	double x28 = x11 * x2;
	double x29 = x17 * x27 + m_d4 * x18 + x19 * x28 - m_d4 * x20;
	double x30 = parms[37] * x22 + parms[38] * x26 + parms[39] * x29;
	double x31 = -x26;
	double x32 = -ddq[0];
	double x33 = x0 * x32;
	double x34 = x2 * x33 + x3 * x8;
	double x35 = dq[1] * x11 + x32 * x7;
	double x36 = -x4;
	double x37 = dq[1] * x9 + x33;
	double x38 = -dq[2];
	double x39 = x10 * x37 + x24 * x38 + x35 * x36;
	double x40 = x17 * x39;
	double x41 = ddq[1] + ddq[2];
	double x42 = x19 * x41;
	double x43 = m_a2 * ddq[1];
	double x44 = x10 * x43 + x38 * x5;
	double x45 = -x19;
	double x46 = x17 * x34 + x23 * x29 - m_d4 * x40 - m_d4 * x42 + x44 * x45;
	double x47 = sin(q[4]);
	double x48 = cos(q[4]);
	double x49 = x13 * x19;
	double x50 = x12 * x17;
	double x51 = x49 + x50;
	double x52 = x26 * x47 + x48 * x51;
	double x53 = dq[4] + x21;
	double x54 = -parms[46];
	double x55 = -x48;
	double x56 = -x5;
	double x57 = x29 * x47 + x55 * x56;
	double x58 = parms[47] * x52 + parms[49] * x57 + x53 * x54;
	double x59 = -x52;
	double x60 = dq[2] * x12 + x10 * x35 + x37 * x4;
	double x61 = -x60;
	double x62 = -ddq[3] + x61;
	double x63 = -x23;
	double x64 = x21 * x63 + x40 + x42;
	double x65 = x26 * x55 + x47 * x51;
	double x66 = -dq[4];
	double x67 = x47 * x62 + x48 * x64 + x65 * x66;
	double x68 = -parms[48];
	double x69 = cos(q[5]);
	double x70 = dq[2] * x27 + x4 * x43;
	double x71 = -x70;
	double x72 = x17 * x41;
	double x73 = -m_d4 * x19;
	double x74 = x17 * x28 + x27 * x45 - m_d4 * x49 - m_d4 * x50;
	double x75 = x17 * x44 + x19 * x34 + x39 * x73 + x63 * x74 + m_d4 * x72;
	double x76 = x47 * x71 + x48 * x75 + x57 * x66;
	double x77 = sin(q[5]);
	double x78 = -x77;
	double x79 = x67 * x69;
	double x80 = x23 * x51 + x39 * x45 + x72;
	double x81 = ddq[4] + x80;
	double x82 = x77 * x81;
	double x83 = x52 * x77;
	double x84 = x53 * x69;
	double x85 = x29 * x48 + x47 * x56;
	double x86 = x69 * x85 + x74 * x77 - m_d6 * x83 + m_d6 * x84;
	double x87 = -dq[5];
	double x88 = x46 * x69 + x76 * x78 - m_d6 * x79 - m_d6 * x82 + x86 * x87;
	double x89 = -x83 + x84;
	double x90 = dq[5] * x89 + x79 + x82;
	double x91 = -parms[58];
	double x92 = dq[4] * x52 + x47 * x64 + x55 * x62;
	double x93 = ddq[5] + x92;
	double x94 = dq[5] + x65;
	double x95 = -parms[57];
	double x96 = parms[58] * x89 + parms[59] * x86 + x94 * x95;
	double x97 = x53 * x77;
	double x98 = x52 * x69;
	double x99 = x97 + x98;
	double x100 = -parms[56];
	double x101 = parms[57] * x99 + parms[59] * x57 + x100 * x89;
	double x102 = -x101;
	double x103 = parms[56] * x93 + parms[59] * x88 + x102 * x99 + x90 * x91 + x94 * x96;
	double x104 = x103 * x69;
	double x105 = -x65;
	double x106 = parms[47] * x105 + parms[48] * x53 + parms[49] * x85;
	double x107 = x69 * x74 + x78 * x85 - m_d6 * x97 - m_d6 * x98;
	double x108 = parms[56] * x94 + parms[59] * x107 + x91 * x99;
	double x109 = -x94;
	double x110 = x69 * x81;
	double x111 = x110 + x67 * x78 + x87 * x99;
	double x112 = -m_d6 * x77;
	double x113 = dq[5] * x107 + m_d6 * x110 + x112 * x67 + x46 * x77 + x69 * x76;
	double x114 = parms[58] * x111 + parms[59] * x113 + x101 * x89 + x108 * x109 + x93 * x95;
	double x115 = -parms[36];
	double x116 = -parms[38];
	double x117 = parms[36] * x21 + parms[39] * x56 + x116 * x51;
	double x118 = parms[37] * x64 + parms[39] * x46 + parms[46] * x92 + parms[49] * x46 + x104 + x106 * x65 + x114 * x77 + x115 * x62 + x117 * x51 + x30 * x31 + x58 * x59 + x67 * x68;
	double x119 = x118 * x17;
	double x120 = -parms[28];
	double x121 = parms[27] * x25 + parms[28] * x13 + parms[29] * x27;
	double x122 = x114 * x69;
	double x123 = -parms[47];
	double x124 = parms[46] * x65 + parms[49] * x74 + x52 * x68;
	double x125 = parms[48] * x81 + parms[49] * x76 + x103 * x78 + x105 * x124 + x122 + x123 * x92 + x53 * x58;
	double x126 = -parms[37];
	double x127 = -x96;
	double x128 = dq[4] * x85 + x47 * x75 + x55 * x71;
	double x129 = -x53;
	double x130 = parms[47] * x67 + parms[49] * x128 + parms[57] * x90 + parms[59] * x128 + x100 * x111 + x106 * x129 + x108 * x99 + x124 * x52 + x127 * x89 + x54 * x81;
	double x131 = parms[37] * x51 + parms[39] * x74 + x115 * x26;
	double x132 = parms[38] * x62 + parms[39] * x75 + x117 * x22 + x125 * x48 + x126 * x80 + x130 * x47 + x131 * x26;
	double x133 = -parms[16];
	double x134 = dq[1] * parms[16] + parms[18] * x9 + parms[19] * x3;
	double x135 = -dq[1];
	double x136 = parms[17] * x135 + parms[18] * x11;
	double x137 = -x136;
	double x138 = parms[17] * x8 + parms[19] * x28 + x11 * x133;
	double x139 = -x28;
	double x140 = dq[1] * parms[14] + parms[11] * x8 + parms[13] * x11 + parms[16] * x139;
	double x141 = -parms[18];
	double x142 = parms[30] * x51 + parms[31] * x26 + parms[32] * x21 + parms[37] * x74 + x116 * x56;
	double x143 = -x74;
	double x144 = parms[31] * x51 + parms[33] * x26 + parms[34] * x21 + parms[36] * x143 + parms[38] * x29;
	double x145 = parms[52] * x99 + parms[54] * x89 + parms[55] * x94 + parms[56] * x107 + x86 * x95;
	double x146 = parms[51] * x99 + parms[53] * x89 + parms[54] * x94 + parms[58] * x86 + x100 * x57;
	double x147 = -x57;
	double x148 = parms[50] * x90 + parms[51] * x111 + parms[52] * x93 + parms[57] * x128 + x101 * x107 + x108 * x147 + x109 * x146 + x145 * x89 + x88 * x91;
	double x149 = -x85;
	double x150 = parms[50] * x99 + parms[51] * x89 + parms[52] * x94 + parms[57] * x57 + x107 * x91;
	double x151 = parms[51] * x90 + parms[53] * x111 + parms[54] * x93 + parms[58] * x113 + x100 * x128 + x102 * x86 - x145 * x99 + x150 * x94 + x57 * x96;
	double x152 = parms[40] * x52 + parms[41] * x53 + parms[42] * x65 + parms[47] * x57 + x68 * x74;
	double x153 = parms[42] * x52 + parms[44] * x53 + parms[45] * x65 + parms[46] * x74 + parms[47] * x149;
	double x154 = parms[41] * x67 + parms[43] * x81 + parms[44] * x92 + parms[48] * x76 + x103 * x112 + x106 * x57 + m_d6 * x122 + x128 * x54 + x148 * x77 + x149 * x58 + x151 * x69 + x152 * x65 + x153 * x59;
	double x155 = parms[32] * x64 + parms[34] * x62 + parms[35] * x80 + parms[36] * x71 + x117 * x29 + x126 * x75 + x142 * x31 + x144 * x51 + x154 - x30 * x56;
	double x156 = -x27;
	double x157 = parms[22] * x12 + parms[24] * x13 + parms[25] * x24 + parms[26] * x28 + parms[27] * x156;
	double x158 = parms[26] * x24 + parms[29] * x28 + x12 * x120;
	double x159 = parms[21] * x12 + parms[23] * x13 + parms[24] * x24 + parms[26] * x56 + parms[28] * x27;
	double x160 = -x29;
	double x161 = parms[32] * x51 + parms[34] * x26 + parms[35] * x21 + parms[36] * x56 + parms[37] * x160;
	double x162 = parms[41] * x52 + parms[43] * x53 + parms[44] * x65 + parms[46] * x147 + parms[48] * x85;
	double x163 = parms[52] * x90 + parms[54] * x111 + parms[55] * x93 + parms[56] * x88 + x107 * x127 + x108 * x86 + x113 * x95 + x146 * x99 - x150 * x89;
	double x164 = parms[42] * x67 + parms[44] * x81 + parms[45] * x92 + parms[46] * x46 + x106 * x143 + x123 * x76 + x124 * x85 + x129 * x152 + x162 * x52 + x163;
	double x165 = parms[40] * x67 + parms[41] * x81 + parms[42] * x92 + parms[47] * x128 - m_d6 * x104 + x105 * x162 + x112 * x114 + x124 * x147 + x148 * x69 + x151 * x78 + x153 * x53 + x46 * x68 + x58 * x74;
	double x166 = parms[30] * x64 + parms[31] * x62 + parms[32] * x80 + parms[37] * x46 + x116 * x71 + x117 * x143 + x131 * x56 + x144 * x22 + x161 * x26 + x164 * x47 + x165 * x48;
	double x167 = parms[20] * x39 + parms[21] * x41 + parms[22] * x60 + parms[27] * x70 - m_d4 * x119 + x120 * x34 + x13 * x157 + x132 * x73 + x15 * x28 + x155 * x45 + x158 * x56 + x159 * x25 + x166 * x17;
	double x168 = dq[1] * parms[15] + parms[12] * x8 + parms[14] * x11 + parms[16] * x3;
	double x169 = -x51;
	double x170 = -parms[31] * x64 - parms[33] * x62 - parms[34] * x80 - parms[38] * x75 - x115 * x46 - x131 * x160 - x142 * x21 - x161 * x169 - x164 * x55 - x165 * x47 - x30 * x74;
	double x171 = parms[20] * x12 + parms[21] * x13 + parms[22] * x24 + parms[27] * x5 + x120 * x28;
	double x172 = parms[22] * x39 + parms[24] * x41 + parms[25] * x60 + parms[26] * x34 - parms[27] * x44 + x12 * x159 + x121 * x139 + x14 * x171 + x158 * x27 + x170;
	double x173 = dq[1] * parms[12] + parms[10] * x8 + parms[11] * x11 + parms[17] * x28 + x141 * x3;
	double x174 = x132 * x17;
	double x175 = parms[21] * x39 + parms[23] * x41 + parms[24] * x60 + parms[26] * x71 + parms[28] * x44 + x118 * x73 + x121 * x5 + x15 * x156 + x155 * x17 + x157 * x16 + x166 * x19 + x171 * x24 + m_d4 * x174;
	//
	tau[0] = -parms[3] * x32 - x0 * x2 * (parms[17] * x35 + parms[19] * x34 + parms[26] * x60 + parms[29] * x34 + x11 * x137 + x119 + x120 * x39 + x121 * x24 + x132 * x19 + x133 * x37 + x134 * x8 + x15 * x16) - x0 * (ddq[1] * parms[14] + dq[1] * x173 + parms[11] * x35 + parms[13] * x37 + x10 * x167 + x133 * x34 + x136 * x28 + x168 * x9 + x172 * x4) - x7 * (ddq[1] * parms[12] + parms[10] * x35 + parms[11] * x37 +
		parms[17] * x34 + x10 * x172 + x11 * x168 + x134 * x139 + x135 * x140 + x138 * x3 + x141 * x43 + x167 * x36);
	tau[1] = ddq[1] * parms[15] + m_a2 * ddq[1] * parms[16] + m_a2 * dq[1] * x136 + parms[12] * x35 + parms[14] * x37 + parms[16] * x43 + m_a2 * parms[19] * x43 + m_a2 * x10 * (parms[27] * x61 + parms[28] * x41 + parms[29] * x44 + x118 * x45 + x13 * x15 + x158 * x25 + x174) - x11 * x173 + x137 * x3 + m_a2 * x138 * x9 + x140 * x8 + m_a2 * x141 * x35 + x175 + m_a2 * x4 * (-parms[26] * x41 + parms[27] * x39 + parms[29] * x70 - parms[36] * x80 - parms[39] * x71 - x116 * x64 + x12 * x158 + x121 * x14 - x125 * x47 - x130 * x55 - x131 * x169 - x21 * x30);
	tau[2] = x175;
	tau[3] = x170;
	tau[4] = x154;
	tau[5] = x163;
	return;
}
