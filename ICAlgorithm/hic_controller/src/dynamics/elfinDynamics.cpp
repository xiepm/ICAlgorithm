#include "hic_controller/dynamics/elfinDynamics.h"
#include <iostream>

elfinDynamics::elfinDynamics()
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


elfinDynamics::~elfinDynamics()
{

}



void elfinDynamics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d4 = kinematcisParam[1];
	m_d6 = kinematcisParam[2];
	m_a2 = kinematcisParam[3];

}

void elfinDynamics::setGravityVector
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
}

void elfinDynamics::calculateGravityJointTorques
(
	const EcRealVector& q,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	calculateEstimateJointToqrues(q, m_zeros, m_zeros, parms, tau);
}


// calculate forward dynamics of elfin
EcBoolean elfinDynamics::calculateEstimateJointToqrues
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
	double x0 = sin(q[0]);
	double x1 = cos(q[0]);
	double x2 = -m_gy * x0 - m_gx * x1;
	double x3 = sin(q[1]);
	double x4 = cos(q[1]);
	double x5 = -m_a2 * ((x3) * (x3)) - m_a2 * ((x4) * (x4));
	double x6 = -m_a2 * dq[1];
	double x7 = -dq[1];
	double x8 = dq[0] * x4;
	double x9 = -x8;
	double x10 = parms[19] * x7 + parms[21] * x9 + parms[22] * x6;
	double x11 = dq[0] * x3;
	double x12 = -x7;
	double x13 = parms[20] * x12 + parms[21] * x11;
	double x14 = -x13;
	double x15 = sin(q[2]);
	double x16 = cos(q[2]);
	double x17 = x11 * x16 + x15 * x9;
	double x18 = ddq[0] * x3;
	double x19 = x18 + x7 * x9;
	double x20 = ddq[0] * x4 + x11 * x7;
	double x21 = dq[2] * x17 + x15 * x19 + x16 * x20;
	double x22 = -parms[34];
	double x23 = x11 * x15 + x16 * x8;
	double x24 = -dq[2];
	double x25 = -x15;
	double x26 = x16 * x19 + x20 * x25 + x23 * x24;
	double x27 = dq[2] + x7;
	double x28 = x16 * x6;
	double x29 = -x23;
	double x30 = parms[33] * x29 + parms[34] * x27 + parms[35] * x28;
	double x31 = -x27;
	double x32 = x15 * x6;
	double x33 = parms[32] * x31 + parms[33] * x17 + parms[35] * x32;
	double x34 = -x17;
	double x35 = x11 * x5;
	double x36 = sin(q[3]);
	double x37 = cos(q[3]);
	double x38 = x27 * x37;
	double x39 = x17 * x36;
	double x40 = x28 * x37 + x35 * x36 + m_d4 * x38 - m_d4 * x39;
	double x41 = x38 - x39;
	double x42 = -x41;
	double x43 = -dq[3];
	double x44 = x29 + x43;
	double x45 = parms[46] * x42 + parms[47] * x44 + parms[48] * x40;
	double x46 = -x45;
	double x47 = -parms[47];
	double x48 = x17 * x37;
	double x49 = x27 * x36;
	double x50 = x48 + x49;
	double x51 = -x32;
	double x52 = parms[45] * x41 + parms[48] * x51 + x47 * x50;
	double x53 = -x21;
	double x54 = -ddq[3] + x53;
	double x55 = -parms[45];
	double x56 = dq[4] + x41;
	double x57 = -parms[58];
	double x58 = cos(q[4]);
	double x59 = sin(q[4]);
	double x60 = x44 * x59 + x50 * x58;
	double x61 = -x58;
	double x62 = x40 * x59 + x51 * x61;
	double x63 = parms[59] * x60 + parms[61] * x62 + x56 * x57;
	double x64 = -x60;
	double x65 = x44 * x61 + x50 * x59;
	double x66 = x40 * x58 + x51 * x59;
	double x67 = -x65;
	double x68 = parms[59] * x67 + parms[60] * x56 + parms[61] * x66;
	double x69 = cos(q[5]);
	double x70 = -x43;
	double x71 = -ddq[1];
	double x72 = ddq[2] + x71;
	double x73 = x36 * x72;
	double x74 = x26 * x37;
	double x75 = x41 * x70 + x73 + x74;
	double x76 = dq[4] * x60 + x54 * x61 + x59 * x75;
	double x77 = ddq[5] + x76;
	double x78 = sin(q[5]);
	double x79 = x60 * x78;
	double x80 = x56 * x69;
	double x81 = -x79 + x80;
	double x82 = -parms[71];
	double x83 = x60 * x69;
	double x84 = x56 * x78;
	double x85 = x83 + x84;
	double x86 = parms[72] * x85 + parms[74] * x62 + x81 * x82;
	double x87 = -x85;
	double x88 = -x36;
	double x89 = x37 * x72;
	double x90 = x26 * x88 + x43 * x50 + x89;
	double x91 = ddq[4] + x90;
	double x92 = x78 * x91;
	double x93 = -dq[4];
	double x94 = x54 * x59 + x58 * x75 + x65 * x93;
	double x95 = x69 * x94;
	double x96 = dq[5] * x81 + x92 + x95;
	double x97 = -parms[73];
	double x98 = dq[5] + x65;
	double x99 = -x98;
	double x100 = x28 * x88 + x35 * x37 - m_d4 * x48 - m_d4 * x49;
	double x101 = x100 * x78 + x66 * x69 - m_d6 * x79 + m_d6 * x80;
	double x102 = parms[72] * x99 + parms[73] * x81 + parms[74] * x101;
	double x103 = -m_d4 * x36;
	double x104 = m_gx * x0 - m_gy * x1;
	double x105 = x104 + x18 * x5 + x6 * x8;
	double x106 = -x2 * x3 - m_gz * x4;
	double x107 = -m_a2 * ddq[1] + x2 * x4 - m_gz * x3;
	double x108 = x106 * x25 + x107 * x16 + x24 * x32;
	double x109 = x100 * x70 + x103 * x26 + x105 * x36 + x108 * x37 + m_d4 * x89;
	double x110 = dq[2] * x28 + x106 * x16 + x107 * x15;
	double x111 = -x110;
	double x112 = x109 * x58 + x111 * x59 + x62 * x93;
	double x113 = -x78;
	double x114 = x105 * x37 + x108 * x88 + x40 * x43 - m_d4 * x73 - m_d4 * x74;
	double x115 = -dq[5];
	double x116 = x101 * x115 + x112 * x113 + x114 * x69 - m_d6 * x92 - m_d6 * x95;
	double x117 = parms[71] * x77 + parms[74] * x116 + x102 * x98 + x86 * x87 + x96 * x97;
	double x118 = x117 * x69;
	double x119 = x100 * x69 + x113 * x66 - m_d6 * x83 - m_d6 * x84;
	double x120 = parms[71] * x98 + parms[74] * x119 + x85 * x97;
	double x121 = -parms[72];
	double x122 = x69 * x91;
	double x123 = -m_d6 * x78;
	double x124 = dq[5] * x119 + x112 * x69 + x114 * x78 + m_d6 * x122 + x123 * x94;
	double x125 = x113 * x94 + x115 * x85 + x122;
	double x126 = parms[73] * x125 + parms[74] * x124 + x120 * x99 + x121 * x77 + x81 * x86;
	double x127 = -parms[60];
	double x128 = parms[46] * x75 + parms[48] * x114 + parms[58] * x76 + parms[61] * x114 + x118 + x126 * x78 + x127 * x94 + x44 * x46 + x50 * x52 + x54 * x55 + x63 * x64 + x65 * x68;
	double x129 = x128 * x37;
	double x130 = -x68;
	double x131 = dq[4] * x66 + x109 * x59 + x111 * x61;
	double x132 = -x102;
	double x133 = parms[58] * x65 + parms[61] * x100 + x127 * x60;
	double x134 = parms[59] * x94 + parms[61] * x131 + parms[72] * x96 + parms[74] * x131 + x120 * x85 + x125 * x82 + x130 * x56 + x132 * x81 + x133 * x60 + x57 * x91;
	double x135 = parms[46] * x50 + parms[48] * x100 + x44 * x55;
	double x136 = x126 * x69;
	double x137 = -parms[59];
	double x138 = parms[60] * x91 + parms[61] * x112 + x113 * x117 + x133 * x67 + x136 + x137 * x76 + x56 * x63;
	double x139 = -parms[46];
	double x140 = parms[47] * x54 + parms[48] * x109 + x134 * x59 + x135 * x44 + x138 * x58 + x139 * x90 + x42 * x52;
	double x141 = -parms[19];
	double x142 = parms[15] * x8 + parms[17] * x11 + parms[18] * x7 + parms[19] * x6;
	double x143 = -x35;
	double x144 = parms[27] * x17 + parms[29] * x27 + parms[30] * x23 + parms[32] * x51 + parms[34] * x28;
	double x145 = -x40;
	double x146 = parms[41] * x50 + parms[43] * x44 + parms[44] * x41 + parms[45] * x51 + parms[46] * x145;
	double x147 = -x50;
	double x148 = -x62;
	double x149 = parms[66] * x85 + parms[68] * x81 + parms[69] * x98 + parms[71] * x148 + parms[73] * x101;
	double x150 = parms[65] * x85 + parms[66] * x81 + parms[67] * x98 + parms[72] * x62 + x119 * x97;
	double x151 = parms[67] * x96 + parms[69] * x125 + parms[70] * x77 + parms[71] * x116 + x101 * x120 + x119 * x132 + x121 * x124 + x149 * x85 - x150 * x81;
	double x152 = parms[53] * x60 + parms[55] * x56 + parms[56] * x65 + parms[58] * x148 + parms[60] * x66;
	double x153 = parms[52] * x60 + parms[53] * x56 + parms[54] * x65 + parms[59] * x62 + x100 * x127;
	double x154 = parms[54] * x94 + parms[56] * x91 + parms[57] * x76 + parms[58] * x114 + x100 * x130 + x112 * x137 + x133 * x66 + x151 + x152 * x60 - x153 * x56;
	double x155 = parms[39] * x50 + parms[40] * x44 + parms[41] * x41 + parms[46] * x100 + x47 * x51;
	double x156 = -x101;
	double x157 = parms[67] * x85 + parms[69] * x81 + parms[70] * x98 + parms[71] * x119 + parms[72] * x156;
	double x158 = parms[65] * x96 + parms[66] * x125 + parms[67] * x77 + parms[72] * x131 + x116 * x97 + x119 * x86 + x120 * x148 + x149 * x99 + x157 * x81;
	double x159 = -x66;
	double x160 = parms[54] * x60 + parms[56] * x56 + parms[57] * x65 + parms[58] * x100 + parms[59] * x159;
	double x161 = parms[66] * x96 + parms[68] * x125 + parms[69] * x77 + parms[73] * x124 + x102 * x62 + x131 * x82 + x150 * x98 + x156 * x86 + x157 * x87;
	double x162 = parms[52] * x94 + parms[53] * x91 + parms[54] * x76 + parms[59] * x131 + x100 * x63 + x113 * x161 + x114 * x127 - m_d6 * x118 + x123 * x126 + x133 * x148 + x152 * x67 + x158 * x69 + x160 * x56;
	double x163 = -parms[40] * x75 - parms[42] * x54 - parms[43] * x90 - parms[47] * x109 - x100 * x45 - x114 * x55 - x135 * x145 - x146 * x147 - x154 * x61 - x155 * x41 - x162 * x59;
	double x164 = parms[26] * x17 + parms[27] * x27 + parms[28] * x23 + parms[33] * x32 + x22 * x35;
	double x165 = parms[32] * x23 + parms[35] * x35 + x17 * x22;
	double x166 = parms[28] * x26 + parms[30] * x72 + parms[31] * x21 + parms[32] * x105 - parms[33] * x108 + x143 * x30 + x144 * x17 + x163 + x164 * x31 + x165 * x28;
	double x167 = -parms[21];
	double x168 = parms[14] * x8 + parms[16] * x11 + parms[17] * x7 + parms[19] * x143;
	double x169 = -x28;
	double x170 = parms[28] * x17 + parms[30] * x27 + parms[31] * x23 + parms[32] * x35 + parms[33] * x169;
	double x171 = -x100;
	double x172 = parms[40] * x50 + parms[42] * x44 + parms[43] * x41 + parms[45] * x171 + parms[47] * x40;
	double x173 = parms[53] * x94 + parms[55] * x91 + parms[56] * x76 + parms[60] * x112 + x117 * x123 + x131 * x57 + m_d6 * x136 + x153 * x65 + x158 * x78 + x159 * x63 + x160 * x64 + x161 * x69 + x62 * x68;
	double x174 = parms[41] * x75 + parms[43] * x54 + parms[44] * x90 + parms[45] * x111 + x109 * x139 - x155 * x44 + x172 * x50 + x173 + x40 * x52 + x46 * x51;
	double x175 = parms[39] * x75 + parms[40] * x54 + parms[41] * x90 + parms[46] * x114 + x111 * x47 + x135 * x51 + x146 * x44 + x154 * x59 + x162 * x58 + x171 * x52 + x172 * x42;
	double x176 = parms[26] * x26 + parms[27] * x72 + parms[28] * x21 + parms[33] * x110 + x103 * x140 + x105 * x22 - m_d4 * x129 + x144 * x29 + x165 * x51 + x170 * x27 + x174 * x88 + x175 * x37 + x33 * x35;
	double x177 = parms[20] * x8 + parms[22] * x35 + x11 * x141;
	double x178 = parms[13] * x8 + parms[14] * x11 + parms[15] * x7 + parms[20] * x35 + x167 * x6;
	double x179 = x140 * x37;
	double x180 = parms[27] * x26 + parms[29] * x72 + parms[30] * x21 + parms[32] * x111 + parms[34] * x108 + x103 * x128 + x164 * x23 + x169 * x33 + x170 * x34 + x174 * x37 + x175 * x36 + m_d4 * x179 + x30 * x32;
	//


	tau[0] = ddq[0] * parms[10] + ddq[0] * parms[3] + dq[0] * parms[11] + parms[12] * sign(dq[0]) + parms[6] * x104 + parms[8] * x2 + x3 * x5 * (parms[20] * x20 + parms[22] * x105 + parms[32] * x21 + parms[35] * x105 + x10 * x8 + x11 * x14 + x129 + x140 * x36 + x141 * x19 + x22 * x26 + x23 * x30 + x33 * x34) + x3 * (parms[14] * x20 + parms[16] * x19 + parms[17] * x71 + parms[21] * x106 + x105 * x141 + x13 * x35 + x142 * x9 + x15 * x166 + x16 * x176 + x178 * x7) + x4 * (parms[13] * x20 + parms[14] * x19 + parms[15] * x71 + parms[20] * x105 + x10 * x143 + x107 * x167 + x11 * x142 + x12 * x168 + x16 * x166 + x176 * x25 + x177 * x6);
	tau[1] = ddq[1] * parms[23] + dq[1] * parms[24] - parms[15] * x20 - parms[17] * x19 - parms[18] * x71 - parms[19] * x107 - m_a2 * parms[19] * x71 + parms[20] * x106 - m_a2 * parms[22] * x107 + parms[25] * sign(dq[1]) + x11 * x178 - m_a2 * x13 * x7 - x14 * x6 - m_a2 * x15 * (-parms[32] * x72 + parms[33] * x26 + parms[35] * x110 - parms[45] * x90 - parms[48] * x111 - x134 * x61 - x135 * x147 - x138 * x59 + x165 * x17 + x30 * x31 - x41 * x45 - x47 * x75) - m_a2 * x16 * (parms[33] * x53 + parms[34] * x72 + parms[35] * x108 + x128 * x88 + x165 * x29 + x179 + x27 * x33) - m_a2 * x167 * x20 - x168 * x8 - m_a2 * x177 * x9 - x180;
	tau[2] = ddq[2] * parms[36] + dq[2] * parms[37] + parms[38] * sign(dq[2]) + x180;
	tau[3] = ddq[3] * parms[49] + dq[3] * parms[50] + parms[51] * sign(dq[3]) + x163;
	tau[4] = ddq[4] * parms[62] + dq[4] * parms[63] + parms[64] * sign(dq[4]) + x173;
	tau[5] = ddq[5] * parms[75] + dq[5] * parms[76] + parms[77] * sign(dq[5]) + x151;

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
void elfinDynamics::calculateMomentumEstimatedJointTorques
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	double x0 = cos(q[1]);
	double x1 = dq[0] * x0;
	double x2 = -m_a2 * dq[1];
	double x3 = sin(q[1]);
	double x4 = ddq[0] * x3;
	double x5 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x3) * (x3));
	double x6 = x1 * x2 + x4 * x5;
	double x7 = cos(q[2]);
	double x8 = x2 * x7;
	double x9 = -m_a2 * ddq[1];
	double x10 = sin(q[2]);
	double x11 = dq[2] * x8 + x10 * x9;
	double x12 = dq[0] * x3;
	double x13 = x12 * x5;
	double x14 = -x1;
	double x15 = x10 * x14 + x12 * x7;
	double x16 = -parms[28];
	double x17 = x1 * x7 + x10 * x12;
	double x18 = parms[26] * x17 + parms[29] * x13 + x15 * x16;
	double x19 = x10 * x2;
	double x20 = -x19;
	double x21 = -dq[1];
	double x22 = x14 * x21 + x4;
	double x23 = ddq[0] * x0 + x12 * x21;
	double x24 = -x10;
	double x25 = -dq[2];
	double x26 = x17 * x25 + x22 * x7 + x23 * x24;
	double x27 = -ddq[1];
	double x28 = ddq[2] + x27;
	double x29 = dq[2] * x15 + x10 * x22 + x23 * x7;
	double x30 = dq[2] + x21;
	double x31 = parms[21] * x15 + parms[23] * x30 + parms[24] * x17 + parms[26] * x20 + parms[28] * x8;
	double x32 = -x17;
	double x33 = cos(q[3]);
	double x34 = x30 * x33;
	double x35 = sin(q[3]);
	double x36 = x15 * x35;
	double x37 = x34 - x36;
	double x38 = -parms[38];
	double x39 = x30 * x35;
	double x40 = x15 * x33;
	double x41 = x39 + x40;
	double x42 = parms[36] * x37 + parms[39] * x20 + x38 * x41;
	double x43 = -x29;
	double x44 = -ddq[3] + x43;
	double x45 = -parms[36];
	double x46 = x28 * x35;
	double x47 = x26 * x33;
	double x48 = x19 * x25 + x7 * x9;
	double x49 = -x35;
	double x50 = x13 * x35 + x33 * x8 + m_d4 * x34 - m_d4 * x36;
	double x51 = -dq[3];
	double x52 = x33 * x6 - m_d4 * x46 - m_d4 * x47 + x48 * x49 + x50 * x51;
	double x53 = cos(q[4]);
	double x54 = -x53;
	double x55 = -x51;
	double x56 = x37 * x55 + x46 + x47;
	double x57 = sin(q[4]);
	double x58 = x32 + x51;
	double x59 = x41 * x53 + x57 * x58;
	double x60 = dq[4] * x59 + x44 * x54 + x56 * x57;
	double x61 = cos(q[5]);
	double x62 = ddq[5] + x60;
	double x63 = x20 * x54 + x50 * x57;
	double x64 = x59 * x61;
	double x65 = dq[4] + x37;
	double x66 = sin(q[5]);
	double x67 = x65 * x66;
	double x68 = x64 + x67;
	double x69 = x61 * x65;
	double x70 = x59 * x66;
	double x71 = x69 - x70;
	double x72 = -parms[56];
	double x73 = parms[57] * x68 + parms[59] * x63 + x71 * x72;
	double x74 = -x68;
	double x75 = x41 * x57 + x54 * x58;
	double x76 = -dq[4];
	double x77 = x44 * x57 + x53 * x56 + x75 * x76;
	double x78 = x61 * x77;
	double x79 = x28 * x33;
	double x80 = x26 * x49 + x41 * x51 + x79;
	double x81 = ddq[4] + x80;
	double x82 = x66 * x81;
	double x83 = -x11;
	double x84 = x13 * x33 - m_d4 * x39 - m_d4 * x40 + x49 * x8;
	double x85 = -m_d4 * x35;
	double x86 = x26 * x85 + x33 * x48 + x35 * x6 + x55 * x84 + m_d4 * x79;
	double x87 = x53 * x86 + x57 * x83 + x63 * x76;
	double x88 = -x66;
	double x89 = x20 * x57 + x50 * x53;
	double x90 = x61 * x89 + x66 * x84 + m_d6 * x69 - m_d6 * x70;
	double x91 = -dq[5];
	double x92 = x52 * x61 - m_d6 * x78 - m_d6 * x82 + x87 * x88 + x90 * x91;
	double x93 = dq[5] + x75;
	double x94 = -x93;
	double x95 = parms[57] * x94 + parms[58] * x71 + parms[59] * x90;
	double x96 = dq[5] * x71 + x78 + x82;
	double x97 = -parms[58];
	double x98 = parms[56] * x62 + parms[59] * x92 + x73 * x74 + x93 * x95 + x96 * x97;
	double x99 = x61 * x98;
	double x100 = -parms[48];
	double x101 = x61 * x81;
	double x102 = x101 + x68 * x91 + x77 * x88;
	double x103 = -parms[57];
	double x104 = x61 * x84 - m_d6 * x64 - m_d6 * x67 + x88 * x89;
	double x105 = -m_d6 * x66;
	double x106 = dq[5] * x104 + m_d6 * x101 + x105 * x77 + x52 * x66 + x61 * x87;
	double x107 = parms[56] * x93 + parms[59] * x104 + x68 * x97;
	double x108 = parms[58] * x102 + parms[59] * x106 + x103 * x62 + x107 * x94 + x71 * x73;
	double x109 = -parms[47];
	double x110 = parms[48] * x65 + parms[49] * x89 + x109 * x75;
	double x111 = -parms[46];
	double x112 = parms[47] * x59 + parms[49] * x63 + x111 * x65;
	double x113 = -x59;
	double x114 = -x37;
	double x115 = parms[37] * x114 + parms[38] * x58 + parms[39] * x50;
	double x116 = -x115;
	double x117 = parms[37] * x56 + parms[39] * x52 + parms[46] * x60 + parms[49] * x52 + x100 * x77 + x108 * x66 + x110 * x75 + x112 * x113 + x116 * x58 + x41 * x42 + x44 * x45 + x99;
	double x118 = x117 * x33;
	double x119 = -x84;
	double x120 = parms[31] * x41 + parms[33] * x58 + parms[34] * x37 + parms[36] * x119 + parms[38] * x50;
	double x121 = parms[42] * x59 + parms[44] * x65 + parms[45] * x75 + parms[46] * x84 + x109 * x89;
	double x122 = -x63;
	double x123 = parms[51] * x68 + parms[53] * x71 + parms[54] * x93 + parms[56] * x122 + parms[58] * x90;
	double x124 = dq[4] * x89 + x54 * x83 + x57 * x86;
	double x125 = -x90;
	double x126 = parms[52] * x68 + parms[54] * x71 + parms[55] * x93 + parms[56] * x104 + parms[57] * x125;
	double x127 = parms[50] * x96 + parms[51] * x102 + parms[52] * x62 + parms[57] * x124 + x104 * x73 + x107 * x122 + x123 * x94 + x126 * x71 + x92 * x97;
	double x128 = parms[40] * x59 + parms[41] * x65 + parms[42] * x75 + parms[47] * x63 + x100 * x84;
	double x129 = x108 * x61;
	double x130 = parms[50] * x68 + parms[51] * x71 + parms[52] * x93 + parms[57] * x63 + x104 * x97;
	double x131 = parms[51] * x96 + parms[53] * x102 + parms[54] * x62 + parms[58] * x106 + x124 * x72 + x125 * x73 + x126 * x74 + x130 * x93 + x63 * x95;
	double x132 = parms[41] * x77 + parms[43] * x81 + parms[44] * x60 + parms[48] * x87 + x105 * x98 + x110 * x63 + x111 * x124 - x112 * x89 + x113 * x121 + x127 * x66 + x128 * x75 + m_d6 * x129 + x131 * x61;
	double x133 = -parms[37];
	double x134 = parms[30] * x41 + parms[31] * x58 + parms[32] * x37 + parms[37] * x84 + x20 * x38;
	double x135 = parms[32] * x56 + parms[34] * x44 + parms[35] * x80 + parms[36] * x83 + x116 * x20 + x120 * x41 + x132 + x133 * x86 - x134 * x58 + x42 * x50;
	double x136 = parms[37] * x41 + parms[39] * x84 + x45 * x58;
	double x137 = -x50;
	double x138 = parms[32] * x41 + parms[34] * x58 + parms[35] * x37 + parms[36] * x20 + parms[37] * x137;
	double x139 = parms[46] * x75 + parms[49] * x84 + x100 * x59;
	double x140 = parms[41] * x59 + parms[43] * x65 + parms[44] * x75 + parms[46] * x122 + parms[48] * x89;
	double x141 = -x75;
	double x142 = parms[40] * x77 + parms[41] * x81 + parms[42] * x60 + parms[47] * x124 + x100 * x52 + x105 * x108 + x112 * x84 + x121 * x65 + x122 * x139 + x127 * x61 + x131 * x88 + x140 * x141 - m_d6 * x99;
	double x143 = -x95;
	double x144 = parms[52] * x96 + parms[54] * x102 + parms[55] * x62 + parms[56] * x92 + x103 * x106 + x104 * x143 + x107 * x90 + x123 * x68 - x130 * x71;
	double x145 = -x110;
	double x146 = parms[42] * x77 + parms[44] * x81 + parms[45] * x60 + parms[46] * x52 + x109 * x87 - x128 * x65 + x139 * x89 + x140 * x59 + x144 + x145 * x84;
	double x147 = parms[30] * x56 + parms[31] * x44 + parms[32] * x80 + parms[37] * x52 + x114 * x120 + x119 * x42 + x136 * x20 + x138 * x58 + x142 * x53 + x146 * x57 + x38 * x83;
	double x148 = -x8;
	double x149 = parms[22] * x15 + parms[24] * x30 + parms[25] * x17 + parms[26] * x13 + parms[27] * x148;
	double x150 = -x30;
	double x151 = parms[26] * x150 + parms[27] * x15 + parms[29] * x19;
	double x152 = parms[48] * x81 + parms[49] * x87 + x109 * x60 + x112 * x65 + x129 + x139 * x141 + x88 * x98;
	double x153 = parms[47] * x77 + parms[49] * x124 + parms[57] * x96 + parms[59] * x124 + x102 * x72 + x107 * x68 + x111 * x81 + x139 * x59 + x143 * x71 + x145 * x65;
	double x154 = parms[38] * x44 + parms[39] * x86 + x114 * x42 + x133 * x80 + x136 * x58 + x152 * x53 + x153 * x57;
	double x155 = parms[20] * x26 + parms[21] * x28 + parms[22] * x29 + parms[27] * x11 - m_d4 * x118 + x13 * x151 + x135 * x49 + x147 * x33 + x149 * x30 + x154 * x85 + x16 * x6 + x18 * x20 + x31 * x32;
	double x156 = -parms[18];
	double x157 = -parms[16];
	double x158 = parms[17] * x1 + parms[19] * x13 + x12 * x157;
	double x159 = parms[12] * x1 + parms[14] * x12 + parms[15] * x21 + parms[16] * x2;
	double x160 = -x13;
	double x161 = parms[11] * x1 + parms[13] * x12 + parms[14] * x21 + parms[16] * x160;
	double x162 = -x21;
	double x163 = -x41;
	double x164 = -parms[31] * x56 - parms[33] * x44 - parms[34] * x80 - parms[38] * x86 - x115 * x84 - x134 * x37 - x136 * x137 - x138 * x163 - x142 * x57 - x146 * x54 - x45 * x52;
	double x165 = parms[20] * x15 + parms[21] * x30 + parms[22] * x17 + parms[27] * x19 + x13 * x16;
	double x166 = parms[27] * x32 + parms[28] * x30 + parms[29] * x8;
	double x167 = parms[22] * x26 + parms[24] * x28 + parms[25] * x29 + parms[26] * x6 - parms[27] * x48 + x15 * x31 + x150 * x165 + x160 * x166 + x164 + x18 * x8;
	double x168 = parms[16] * x21 + parms[18] * x14 + parms[19] * x2;
	double x169 = parms[17] * x162 + parms[18] * x12;
	double x170 = parms[10] * x1 + parms[11] * x12 + parms[12] * x21 + parms[17] * x13 + x156 * x2;
	double x171 = -x15;
	double x172 = -x169;
	double x173 = x154 * x33;
	double x174 = parms[21] * x26 + parms[23] * x28 + parms[24] * x29 + parms[26] * x83 + parms[28] * x48 + x117 * x85 + x135 * x33 + x147 * x35 + x148 * x151 + x149 * x171 + x165 * x17 + x166 * x19 + m_d4 * x173;
	//
	tau[0] = ddq[0] * parms[3] + x0 * (parms[10] * x23 + parms[11] * x22 + parms[12] * x27 + parms[17] * x6 + x12 * x159 + x155 * x24 + x156 * x9 + x158 * x2 + x160 * x168 + x161 * x162 + x167 * x7) + x3 * x5 * (parms[17] * x23 + parms[19] * x6 + parms[26] * x29 + parms[29] * x6 + x1 * x168 + x118 + x12 * x172 + x151 * x171 + x154 * x35 + x157 * x22 + x16 * x26 + x166 * x17) + x3 * (parms[11] * x23 + parms[13] * x22 + parms[14] * x27 + x10 * x167 + x13 * x169 + x14 * x159 + x155 * x7 + x157 * x6 + x170 * x21);
	tau[1] = -parms[12] * x23 - parms[14] * x22 - parms[15] * x27 - m_a2 * parms[16] * x27 - parms[16] * x9 - m_a2 * parms[19] * x9 - x1 * x161 - m_a2 * x10 * (-parms[26] * x28 + parms[27] * x26 + parms[29] * x11 - parms[36] * x80 - parms[39] * x83 - x115 * x37 - x136 * x163 + x15 * x18 + x150 * x166 - x152 * x57 - x153 * x54 - x38 * x56) + x12 * x170 - m_a2 * x14 * x158 - m_a2 * x156 * x23 - m_a2 * x169 * x21 - x172 * x2 - x174 - m_a2 * x7 * (parms[27] * x43 + parms[28] * x28 + parms[29] * x48 + x117 * x49 + x151 * x30 + x173 + x18 * x32);
	tau[2] = x174;
	tau[3] = x164;
	tau[4] = x132;
	tau[5] = x144;
	//
	return;
}

void elfinDynamics::calculateJointFricition(const EcRealVector& dq, const EcRealVector& parms, EcReal coeffColomb, EcReal coeffViscous, EcRealVector& tau)
{
	tau[0] = coeffViscous * dq[0] * parms[11] + coeffColomb * parms[12] * sign(dq[0]);
	tau[1] = coeffViscous * dq[1] * parms[24] + coeffColomb * parms[25] * sign(dq[1]);
	tau[2] = coeffViscous * dq[2] * parms[37] + coeffColomb * parms[38] * sign(dq[2]);
	tau[3] = coeffViscous * dq[3] * parms[50] + coeffColomb * parms[51] * sign(dq[3]);
	tau[4] = coeffViscous * dq[4] * parms[63] + coeffColomb * parms[64] * sign(dq[4]);
	tau[5] = coeffViscous * dq[5] * parms[76] + coeffColomb * parms[77] * sign(dq[5]);
}

