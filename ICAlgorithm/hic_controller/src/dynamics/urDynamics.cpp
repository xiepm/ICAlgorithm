#include "hic_controller/dynamics/urDynamics.h"
#include <iostream>

urDynamics::urDynamics()
{
	m_NumJoints = 6;
	m_gx = 0;
	m_gy = 0;
	m_gz = -9.81;

	m_d1 = 0.220;		//默认设置为ElfinV5的杆长参数；
	m_d2 = 0.420;
	m_d3 = 0.420;
	m_d4 = 0.420;
	m_d6 = 0.180;
	m_a2 = 0.380;

	m_friTemperaturesParams.assign(m_NumJoints, 1.0);
	m_jointTemperatures.assign(m_NumJoints, 46.0);


	m_zeros.assign(m_NumJoints, 0.0);
}


urDynamics::~urDynamics()
{

}



void urDynamics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d2 = kinematcisParam[1];
	m_d3 = kinematcisParam[2];
	m_d4 = kinematcisParam[3];
	m_d5 = kinematcisParam[4];
	m_d6 = kinematcisParam[5];
	m_a2 = -kinematcisParam[6];
	m_a3 = -kinematcisParam[7];
}

void urDynamics::setGravityVector
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
}

void urDynamics::calculateGravityJointTorques
(
	const EcRealVector& q,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	calculateEstimateJointToqrues(q, m_zeros, m_zeros, parms, tau);
}


// calculate forward dynamics of elfin
EcBoolean urDynamics::calculateEstimateJointToqrues
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
	double x1 = sin(q[1]);
	double x2 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x1) * (x1));
	double x3 = m_a2 * dq[1];
	double x4 = dq[0] * x1;
	double x5 = -m_d2 * x4;
	double x6 = x3 + x5;
	double x7 = -parms[21];
	double x8 = dq[1] * parms[19] + parms[22] * x6 + x4 * x7;
	double x9 = ddq[0] * x0;
	double x10 = cos(q[0]);
	double x11 = sin(q[0]);
	double x12 = m_gy * x10 - m_gx * x11;
	double x13 = x12 + x2 * x9 + x3 * x4;
	double x14 = -dq[1];
	double x15 = dq[0] * x0;
	double x16 = m_d2 * x15;
	double x17 = parms[20] * x14 + parms[21] * x15 + parms[22] * x16;
	double x18 = -x15;
	double x19 = sin(q[2]);
	double x20 = x19 * x4;
	double x21 = -x20;
	double x22 = cos(q[2]);
	double x23 = x15 * x22;
	double x24 = x21 + x23;
	double x25 = -parms[33];
	double x26 = dq[1] + dq[2];
	double x27 = x16 * x22 + x19 * x6 - m_d3 * x20 + m_d3 * x23;
	double x28 = parms[34] * x24 + parms[35] * x27 + x25 * x26;
	double x29 = -x28;
	double x30 = x22 * x4;
	double x31 = x15 * x19;
	double x32 = x30 + x31;
	double x33 = m_a3 * dq[2];
	double x34 = ((x22) * (x22));
	double x35 = ((x19) * (x19));
	double x36 = -m_a3 * x34 - m_a3 * x35;
	double x37 = ddq[0] * x1;
	double x38 = dq[1] * x15 + x37;
	double x39 = -x19;
	double x40 = x38 * x39;
	double x41 = x14 * x4 + x9;
	double x42 = x22 * x41;
	double x43 = x13 + x32 * x33 + x36 * x40 + x36 * x42;
	double x44 = x19 * x41;
	double x45 = x22 * x38;
	double x46 = dq[2] * x24 + x44 + x45;
	double x47 = -dq[2];
	double x48 = x32 * x47 + x40 + x42;
	double x49 = -parms[32];
	double x50 = -parms[34];
	double x51 = m_a3 * x34 + m_a3 * x35;
	double x52 = dq[1] * x51 + x16 * x39 + x22 * x6 - m_d3 * x30 - m_d3 * x31;
	double x53 = x33 + x52;
	double x54 = parms[32] * x26 + parms[35] * x53 + x32 * x50;
	double x55 = cos(q[3]);
	double x56 = x24 * x55;
	double x57 = sin(q[3]);
	double x58 = x32 * x57;
	double x59 = -x56 + x58;
	double x60 = dq[3] + x26;
	double x61 = -x59;
	double x62 = x27 * x55 + x53 * x57 + m_d4 * x56 - m_d4 * x58;
	double x63 = parms[46] * x61 + parms[47] * x60 + parms[48] * x62;
	double x64 = cos(q[4]);
	double x65 = sin(q[4]);
	double x66 = x60 * x65;
	double x67 = x32 * x55;
	double x68 = x24 * x57;
	double x69 = x67 + x68;
	double x70 = x64 * x69;
	double x71 = x66 + x70;
	double x72 = cos(q[5]);
	double x73 = x71 * x72;
	double x74 = -dq[4];
	double x75 = x61 + x74;
	double x76 = sin(q[5]);
	double x77 = x75 * x76;
	double x78 = x73 + x77;
	double x79 = -dq[5];
	double x80 = -dq[3];
	double x81 = x46 * x55;
	double x82 = x48 * x57;
	double x83 = x59 * x80 + x81 + x82;
	double x84 = x64 * x83;
	double x85 = ddq[1] + ddq[2];
	double x86 = ddq[3] + x85;
	double x87 = x65 * x86;
	double x88 = x60 * x64;
	double x89 = x65 * x69;
	double x90 = x88 - x89;
	double x91 = -x74;
	double x92 = x84 + x87 + x90 * x91;
	double x93 = -x76;
	double x94 = x46 * x57;
	double x95 = -x55;
	double x96 = dq[3] * x69 + x48 * x95 + x94;
	double x97 = -x96;
	double x98 = -ddq[4] + x97;
	double x99 = x72 * x98;
	double x100 = x78 * x79 + x92 * x93 + x99;
	double x101 = -parms[71];
	double x102 = x72 * x75;
	double x103 = x71 * x76;
	double x104 = x102 - x103;
	double x105 = x15 * x2;
	double x106 = x105 + x21 * x36 + x23 * x36;
	double x107 = x106 * x65 + x62 * x64 + m_d5 * x88 - m_d5 * x89;
	double x108 = x27 * x57 + x53 * x95 + m_d4 * x67 + m_d4 * x68;
	double x109 = -x108;
	double x110 = m_d6 * x102 - m_d6 * x103 + x107 * x72 + x109 * x76;
	double x111 = dq[5] + x90;
	double x112 = -x111;
	double x113 = parms[72] * x112 + parms[73] * x104 + parms[74] * x110;
	double x114 = -x113;
	double x115 = m_d4 * x55;
	double x116 = -m_gx * x10 - m_gy * x11;
	double x117 = m_a2 * ddq[1] - m_gz * x0 - x1 * x116 + x14 * x16 - m_d2 * x37;
	double x118 = dq[1] * x5 + x0 * x116 - m_gz * x1 + m_d2 * x9;
	double x119 = -m_d3 * x19;
	double x120 = dq[2] * x52 + x117 * x19 + x118 * x22 + x119 * x38 + x14 * x33 + m_d3 * x42;
	double x121 = ddq[1] * x51 + m_a3 * ddq[2] + x117 * x22 + x118 * x39 + x27 * x47 - m_d3 * x44 - m_d3 * x45;
	double x122 = x108 * x80 + x115 * x48 + x120 * x55 + x121 * x57 - m_d4 * x94;
	double x123 = -x65;
	double x124 = x107 * x74 + x122 * x123 + x43 * x64 - m_d5 * x84 - m_d5 * x87;
	double x125 = x72 * x92;
	double x126 = x76 * x98;
	double x127 = dq[5] * x104 + x125 + x126;
	double x128 = x107 * x93 + x109 * x72 - m_d6 * x73 - m_d6 * x77;
	double x129 = -parms[73];
	double x130 = parms[71] * x111 + parms[74] * x128 + x129 * x78;
	double x131 = -parms[60];
	double x132 = parms[58] * x90 + parms[61] * x109 + x131 * x71;
	double x133 = -parms[58];
	double x134 = -x90;
	double x135 = parms[59] * x134 + parms[60] * x75 + parms[61] * x107;
	double x136 = -x135;
	double x137 = parms[59] * x92 + parms[61] * x124 + parms[72] * x127 + parms[74] * x124 + x100 * x101 + x104 * x114 + x130 * x78 + x132 * x71 + x133 * x98 + x136 * x75;
	double x138 = x137 * x64;
	double x139 = -x60;
	double x140 = parms[45] * x139 + parms[46] * x69 + parms[48] * x108;
	double x141 = -x69;
	double x142 = x106 * x64 + x123 * x62 - m_d5 * x66 - m_d5 * x70;
	double x143 = parms[72] * x78 + parms[74] * x142 + x101 * x104;
	double x144 = x64 * x86;
	double x145 = x123 * x83 + x144 + x71 * x74;
	double x146 = ddq[5] + x145;
	double x147 = -parms[72];
	double x148 = -m_d5 * x65;
	double x149 = x122 * x64 + x142 * x91 + m_d5 * x144 + x148 * x83 + x43 * x65;
	double x150 = -m_d6 * x76;
	double x151 = dq[3] * x62 + x120 * x57 + x121 * x95 + m_d4 * x81 + m_d4 * x82;
	double x152 = -x151;
	double x153 = dq[5] * x128 + x149 * x72 + x150 * x92 + x152 * x76 + m_d6 * x99;
	double x154 = parms[73] * x100 + parms[74] * x153 + x104 * x143 + x112 * x130 + x146 * x147;
	double x155 = x154 * x72;
	double x156 = parms[59] * x71 + parms[61] * x142 + x133 * x75;
	double x157 = -parms[59];
	double x158 = x110 * x79 - m_d6 * x125 - m_d6 * x126 + x149 * x93 + x152 * x72;
	double x159 = -x78;
	double x160 = parms[71] * x146 + parms[74] * x158 + x111 * x113 + x127 * x129 + x143 * x159;
	double x161 = parms[60] * x98 + parms[61] * x149 + x132 * x134 + x145 * x157 + x155 + x156 * x75 + x160 * x93;
	double x162 = -parms[47];
	double x163 = parms[33] * x46 + parms[35] * x43 + parms[45] * x96 + parms[48] * x43 + x138 + x140 * x141 + x161 * x65 + x162 * x83 + x24 * x29 +
		x32 * x54 + x48 * x49 + x59 * x63;
	double x164 = -parms[19];
	double x165 = x160 * x72;
	double x166 = -x71;
	double x167 = parms[45] * x59 + parms[48] * x106 + x162 * x69;
	double x168 = -parms[45] * x86 + parms[46] * x83 + parms[48] * x151 - parms[58] * x145 - parms[61] * x152 - x131 * x92 - x135 * x90 + x139 * x63
		- x154 * x76 - x156 * x166 - x165 + x167 * x69;
	double x169 = x161 * x64;
	double x170 = parms[46] * x97 + parms[47] * x86 + parms[48] * x122 + x123 * x137 + x140 * x60 + x167 * x61 + x169;
	double x171 = x170 * x57;
	double x172 = parms[33] * x32 + parms[35] * x106 + x24 * x49;
	double x173 = -x32;
	double x174 = parms[32] * x85 + parms[35] * x121 + x168 * x95 + x171 + x172 * x173 + x26 * x28 + x46 * x50;
	double x175 = x174 * x22;
	double x176 = parms[20] * x4 + parms[22] * x105 + x15 * x164;
	double x177 = -x4;
	double x178 = -x26;
	double x179 = x168 * x57;
	double x180 = parms[34] * x48 + parms[35] * x120 + x170 * x55 + x172 * x24 + x178 * x54 + x179 + x25 * x85;
	double x181 = ddq[1] * parms[19] + dq[1] * x17 + parms[22] * x117 + x175 + x176 * x177 + x180 * x19 + x38 * x7;
	double x182 = -x106;
	double x183 = -x107;
	double x184 = parms[52] * x71 + parms[53] * x75 + parms[54] * x90 + parms[59] * x142 + x109 * x131;
	double x185 = parms[54] * x71 + parms[56] * x75 + parms[57] * x90 + parms[58] * x109 + parms[59] * x183;
	double x186 = parms[66] * x78 + parms[68] * x104 + parms[69] * x111 + parms[73] * x110 + x101 * x142;
	double x187 = -x110;
	double x188 = parms[67] * x78 + parms[69] * x104 + parms[70] * x111 + parms[71] * x128 + parms[72] * x187;
	double x189 = -x142;
	double x190 = parms[65] * x127 + parms[66] * x100 + parms[67] * x146 + parms[72] * x124 + x104 * x188 + x112 * x186 + x128 * x143 + x129 * x158 + x130 * x189;
	double x191 = parms[65] * x78 + parms[66] * x104 + parms[67] * x111 + parms[72] * x142 + x128 * x129;
	double x192 = parms[66] * x127 + parms[68] * x100 + parms[69] * x146 + parms[73] * x153 + x101 * x124 + x111 * x191 + x113 * x142 + x143 * x187 + x159 * x188;
	double x193 = -parms[53] * x92 - parms[55] * x98 - parms[56] * x145 - parms[60] * x149 - x124 * x133 - x135 * x142 - x150 * x160 - m_d6 * x155 -
		x156 * x183 - x166 * x185 - x184 * x90 - x190 * x76 - x192 * x72;
	double x194 = parms[39] * x69 + parms[40] * x60 + parms[41] * x59 + parms[46] * x108 + x106 * x162;
	double x195 = parms[40] * x69 + parms[42] * x60 + parms[43] * x59 + parms[45] * x109 + parms[47] * x62;
	double x196 = parms[41] * x83 + parms[43] * x86 + parms[44] * x96 + parms[45] * x43 - parms[46] * x122 + x139 * x194 + x167 * x62 + x182 * x63 +
		x193 + x195 * x69;
	double x197 = parms[28] * x32 + parms[30] * x24 + parms[31] * x26 + parms[32] * x53 + x25 * x27;
	double x198 = parms[27] * x32 + parms[29] * x24 + parms[30] * x26 + parms[32] * x182 + parms[34] * x27;
	double x199 = parms[53] * x71 + parms[55] * x75 + parms[56] * x90 + parms[58] * x189 + parms[60] * x107;
	double x200 = parms[67] * x127 + parms[69] * x100 + parms[70] * x146 + parms[71] * x158 - x104 * x191 + x110 * x130 + x114 * x128 + x147 * x153 + x186 * x78;
	double x201 = parms[54] * x92 + parms[56] * x98 + parms[57] * x145 + parms[58] * x152 + x107 * x132 + x109 * x136 + x149 * x157 - x184 * x75 + x199 * x71 + x200;
	double x202 = -x62;
	double x203 = parms[41] * x69 + parms[43] * x60 + parms[44] * x59 + parms[45] * x106 + parms[46] * x202;
	double x204 = parms[52] * x92 + parms[53] * x98 + parms[54] * x145 + parms[59] * x124 + x109 * x156 + x131 * x152 + x132 * x189 + x134 * x199 + x150 * x154 - m_d6 * x165 + x185 * x75 + x190 * x72 + x192 * x93;
	double x205 = parms[39] * x83 + parms[40] * x86 + parms[41] * x96 + parms[46] * x151 + x106 * x140 + x109 * x167 + x123 * x201 - m_d5 * x138 + x148 * x161 + x162 * x43 + x195 * x61 + x203 * x60 + x204 * x64;
	double x206 = parms[26] * x46 + parms[27] * x48 + parms[28] * x85 + parms[33] * x43 + x115 * x168 + x121 * x50 - m_d4 * x171 + x172 * x53 + x178 * x198 + x182 * x54 + x196 * x57 + x197 * x24 + x205 * x55;
	double x207 = dq[1] * parms[15] + parms[13] * x4 + parms[14] * x15 + parms[20] * x105 + x6 * x7;
	double x208 = parms[26] * x32 + parms[27] * x24 + parms[28] * x26 + parms[33] * x106 + x50 * x53;
	double x209 = parms[27] * x46 + parms[29] * x48 + parms[30] * x85 + parms[34] * x120 + x106 * x28 + x115 * x170 - x172 * x27 + x173 * x197 + m_d4 * x179 + x196 * x95 + x205 * x57 + x208 * x26 + x43 * x49;
	double x210 = -x16;
	double x211 = dq[1] * parms[18] + parms[15] * x4 + parms[17] * x15 + parms[19] * x6 + parms[20] * x210;
	double x212 = x180 * x22;
	double x213 = x163 * x36;
	double x214 = -parms[20];
	double x215 = dq[1] * parms[17] + parms[14] * x4 + parms[16] * x15 + parms[21] * x16 + x105 * x164;
	double x216 = parms[40] * x83 + parms[42] * x86 + parms[43] * x96 + parms[45] * x152 + parms[47] * x122 + x108 * x63 + x137 * x148 + x140 * x202
		+ x141 * x203 + m_d5 * x169 + x194 * x59 + x201 * x64 + x204 * x65;
	double x217 = parms[28] * x46 + parms[30] * x48 + parms[31] * x85 + parms[32] * x121 + x120 * x25 + x198 * x32 - x208 * x24 + x216 + x27 * x54 +
		x29 * x53;
	//
	tau[0] = ddq[0] * parms[10] + ddq[0] * parms[3] + dq[0] * parms[11] + parms[12] * sign(dq[0]) - parms[6] * x12 + parms[8] * x116 + x0 * x2 * (parms[20] * x38 + parms[22] * x13 + x163 + x164 * x41 + x17 * x18 + x4 * x8) + m_d2 * x0 * (ddq[1] * x214 + parms[21] * x41 + parms[22] * x118 + x14 * x8 + x15 * x176 + x174 * x39 + x212) + x0 * (ddq[1] * parms[17] + dq[1] * x207 + parms[14] * x38 + parms[16] * x41 + parms[21] * x118 + x105 * x17 +
		x119 * x174 + x13 * x164 + x176 * x210 + x177 * x211 + x19 * x206 + x209 * x22 + m_d3 * x212 + x213 * x22) - m_d2 * x1 * x181 + x1 * (ddq[1] * parms[15]
			+ parms[13] * x38 + parms[14] * x41 + parms[20] * x13 - x105 * x8 + x117 * x7 + x119 * x180 + x14 * x215 + x15 * x211 - m_d3 * x175 + x176 * x6 + x206 * x22 + x209 * x39 + x213 * x39);
	tau[1] = ddq[1] * parms[18] + ddq[1] * parms[23] + dq[1] * parms[24] + parms[15] * x38 + parms[17] * x41 + parms[19] * x117 + parms[25] * sign(dq[1]) + x118 * x214 + x16 * x8 - x17 * x6 + x174 * x51 + x18 * x207 + m_a2 * x181 + x215 * x4 + x217;
	tau[2] = ddq[2] * parms[36] + dq[2] * parms[37] + parms[38] * sign(dq[2]) + m_a3 * x174 + x217;
	tau[3] = ddq[3] * parms[49] + dq[3] * parms[50] + parms[51] * sign(dq[3]) + x216;
	tau[4] = ddq[4] * parms[62] + dq[4] * parms[63] + parms[64] * sign(dq[4]) + x193;
	tau[5] = ddq[5] * parms[75] + dq[5] * parms[76] + parms[77] * sign(dq[5]) + x200;

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
void urDynamics::calculateMomentumEstimatedJointTorques
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	double x0 = sin(q[1]);
	double x1 = dq[0] * x0;
	double x2 = -parms[18];
	double x3 = m_a2 * dq[1];
	double x4 = -m_d2 * x1;
	double x5 = x3 + x4;
	double x6 = dq[1] * parms[16] + parms[19] * x5 + x1 * x2;
	double x7 = cos(q[1]);
	double x8 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x7) * (x7));
	double x9 = dq[0] * x7;
	double x10 = x8 * x9;
	double x11 = -x10;
	double x12 = sin(q[3]);
	double x13 = dq[1] + dq[2];
	double x14 = dq[3] + x13;
	double x15 = sin(q[4]);
	double x16 = x14 * x15;
	double x17 = cos(q[2]);
	double x18 = x17 * x9;
	double x19 = sin(q[2]);
	double x20 = x1 * x19;
	double x21 = -x20;
	double x22 = x18 + x21;
	double x23 = x12 * x22;
	double x24 = cos(q[3]);
	double x25 = x1 * x17;
	double x26 = x19 * x9;
	double x27 = x25 + x26;
	double x28 = x24 * x27;
	double x29 = x23 + x28;
	double x30 = cos(q[4]);
	double x31 = x29 * x30;
	double x32 = x16 + x31;
	double x33 = -dq[4];
	double x34 = x12 * x27;
	double x35 = x22 * x24;
	double x36 = x34 - x35;
	double x37 = -dq[3];
	double x38 = -dq[2];
	double x39 = ddq[0] * x7;
	double x40 = -dq[1];
	double x41 = x1 * x40 + x39;
	double x42 = x17 * x41;
	double x43 = ddq[0] * x0;
	double x44 = dq[1] * x9 + x43;
	double x45 = -x19;
	double x46 = x44 * x45;
	double x47 = x27 * x38 + x42 + x46;
	double x48 = x12 * x47;
	double x49 = x19 * x41;
	double x50 = x17 * x44;
	double x51 = dq[2] * x22 + x49 + x50;
	double x52 = x24 * x51;
	double x53 = x36 * x37 + x48 + x52;
	double x54 = -x15;
	double x55 = ddq[1] + ddq[2];
	double x56 = ddq[3] + x55;
	double x57 = x30 * x56;
	double x58 = x32 * x33 + x53 * x54 + x57;
	double x59 = x15 * x29;
	double x60 = x14 * x30;
	double x61 = -x59 + x60;
	double x62 = -x61;
	double x63 = m_a3 * dq[2];
	double x64 = ((x17) * (x17));
	double x65 = ((x19) * (x19));
	double x66 = m_a3 * x64 + m_a3 * x65;
	double x67 = m_d2 * x9;
	double x68 = dq[1] * x66 + x17 * x5 - m_d3 * x25 - m_d3 * x26 + x45 * x67;
	double x69 = x63 + x68;
	double x70 = x17 * x67 + m_d3 * x18 + x19 * x5 - m_d3 * x20;
	double x71 = x12 * x69 + x24 * x70 - m_d4 * x34 + m_d4 * x35;
	double x72 = -m_a3 * x64 - m_a3 * x65;
	double x73 = x10 + x18 * x72 + x21 * x72;
	double x74 = x15 * x73 + x30 * x71 - m_d5 * x59 + m_d5 * x60;
	double x75 = -x36;
	double x76 = x33 + x75;
	double x77 = parms[47] * x62 + parms[48] * x76 + parms[49] * x74;
	double x78 = -x24;
	double x79 = dq[1] * x4 + m_d2 * x39;
	double x80 = m_a2 * ddq[1] + x40 * x67 - m_d2 * x43;
	double x81 = ddq[1] * x66 + m_a3 * ddq[2] + x17 * x80 + x38 * x70 + x45 * x79 - m_d3 * x49 - m_d3 * x50;
	double x82 = -m_d3 * x19;
	double x83 = dq[2] * x68 + x17 * x79 + x19 * x80 + x40 * x63 + m_d3 * x42 + x44 * x82;
	double x84 = dq[3] * x71 + x12 * x83 + m_d4 * x48 + m_d4 * x52 + x78 * x81;
	double x85 = -x84;
	double x86 = cos(q[5]);
	double x87 = ddq[5] + x58;
	double x88 = x76 * x86;
	double x89 = sin(q[5]);
	double x90 = x32 * x89;
	double x91 = x88 - x90;
	double x92 = -x33;
	double x93 = x15 * x56;
	double x94 = x30 * x53;
	double x95 = x61 * x92 + x93 + x94;
	double x96 = x86 * x95;
	double x97 = x12 * x51;
	double x98 = dq[3] * x29 + x47 * x78 + x97;
	double x99 = -x98;
	double x100 = -ddq[4] + x99;
	double x101 = x100 * x89;
	double x102 = dq[5] * x91 + x101 + x96;
	double x103 = -parms[58];
	double x104 = x32 * x86;
	double x105 = x76 * x89;
	double x106 = x104 + x105;
	double x107 = -parms[56];
	double x108 = -m_d5 * x16 + x30 * x73 - m_d5 * x31 + x54 * x71;
	double x109 = parms[57] * x106 + parms[59] * x108 + x107 * x91;
	double x110 = -x106;
	double x111 = dq[5] + x61;
	double x112 = -x111;
	double x113 = x12 * x70 + m_d4 * x23 + m_d4 * x28 + x69 * x78;
	double x114 = -x113;
	double x115 = x114 * x89 + x74 * x86 + m_d6 * x88 - m_d6 * x90;
	double x116 = parms[57] * x112 + parms[58] * x91 + parms[59] * x115;
	double x117 = -dq[5];
	double x118 = x1 * x3 + x39 * x8;
	double x119 = x118 + x27 * x63 + x42 * x72 + x46 * x72;
	double x120 = -m_d5 * x15;
	double x121 = m_d4 * x24;
	double x122 = x113 * x37 + x12 * x81 + x121 * x47 + x24 * x83 - m_d4 * x97;
	double x123 = x108 * x92 + x119 * x15 + x120 * x53 + x122 * x30 + m_d5 * x57;
	double x124 = -x89;
	double x125 = -m_d6 * x101 + x115 * x117 + x123 * x124 + x85 * x86 - m_d6 * x96;
	double x126 = parms[56] * x87 + parms[59] * x125 + x102 * x103 + x109 * x110 + x111 * x116;
	double x127 = x126 * x86;
	double x128 = -parms[46];
	double x129 = parms[47] * x32 + parms[49] * x108 + x128 * x76;
	double x130 = -x32;
	double x131 = -m_d6 * x89;
	double x132 = x100 * x86;
	double x133 = -m_d6 * x104 - m_d6 * x105 + x114 * x86 + x124 * x74;
	double x134 = dq[5] * x133 + x123 * x86 + x131 * x95 + m_d6 * x132 + x85 * x89;
	double x135 = parms[56] * x111 + parms[59] * x133 + x103 * x106;
	double x136 = x106 * x117 + x124 * x95 + x132;
	double x137 = -parms[57];
	double x138 = parms[58] * x136 + parms[59] * x134 + x109 * x91 + x112 * x135 + x137 * x87;
	double x139 = -parms[48];
	double x140 = -parms[38];
	double x141 = parms[36] * x36 + parms[39] * x73 + x140 * x29;
	double x142 = parms[37] * x75 + parms[38] * x14 + parms[39] * x71;
	double x143 = -x14;
	double x144 = -parms[36] * x56 + parms[37] * x53 + parms[39] * x84 - parms[46] * x58 - parms[49] * x85 - x127 - x129 * x130 - x138 * x89 - x139 * x95 + x141 * x29 + x142 * x143 - x61 * x77;
	double x145 = x12 * x144;
	double x146 = -x70;
	double x147 = parms[22] * x27 + parms[24] * x22 + parms[25] * x13 + parms[26] * x69 + parms[27] * x146;
	double x148 = -x27;
	double x149 = parms[36] * x143 + parms[37] * x29 + parms[39] * x113;
	double x150 = parms[46] * x61 + parms[49] * x114 + x139 * x32;
	double x151 = x119 * x30 + x122 * x54 + x33 * x74 - m_d5 * x93 - m_d5 * x94;
	double x152 = -x76;
	double x153 = -x91;
	double x154 = parms[47] * x95 + parms[49] * x151 + parms[57] * x102 + parms[59] * x151 + x100 * x128 + x106 * x135 + x107 * x136 + x116 * x153 + x150 * x32 + x152 * x77;
	double x155 = x138 * x86;
	double x156 = -parms[47];
	double x157 = parms[48] * x100 + parms[49] * x123 + x124 * x126 + x129 * x76 + x150 * x62 + x155 + x156 * x58;
	double x158 = x157 * x30;
	double x159 = parms[37] * x99 + parms[38] * x56 + parms[39] * x122 + x14 * x149 + x141 * x75 + x154 * x54 + x158;
	double x160 = -x13;
	double x161 = parms[27] * x160 + parms[28] * x22 + parms[29] * x70;
	double x162 = -parms[26];
	double x163 = parms[27] * x27 + parms[29] * x73 + x162 * x22;
	double x164 = -parms[28];
	double x165 = parms[20] * x27 + parms[21] * x22 + parms[22] * x13 + parms[27] * x73 + x164 * x69;
	double x166 = parms[31] * x29 + parms[33] * x14 + parms[34] * x36 + parms[36] * x114 + parms[38] * x71;
	double x167 = parms[30] * x29 + parms[31] * x14 + parms[32] * x36 + parms[37] * x113 + x140 * x73;
	double x168 = parms[40] * x32 + parms[41] * x76 + parms[42] * x61 + parms[47] * x108 + x114 * x139;
	double x169 = -x115;
	double x170 = parms[52] * x106 + parms[54] * x91 + parms[55] * x111 + parms[56] * x133 + parms[57] * x169;
	double x171 = parms[50] * x106 + parms[51] * x91 + parms[52] * x111 + parms[57] * x108 + x103 * x133;
	double x172 = parms[51] * x102 + parms[53] * x136 + parms[54] * x87 + parms[58] * x134 + x107 * x151 + x108 * x116 + x109 * x169 + x110 * x170 + x111 * x171;
	double x173 = -x74;
	double x174 = parms[42] * x32 + parms[44] * x76 + parms[45] * x61 + parms[46] * x114 + parms[47] * x173;
	double x175 = -x108;
	double x176 = parms[51] * x106 + parms[53] * x91 + parms[54] * x111 + parms[56] * x175 + parms[58] * x115;
	double x177 = parms[50] * x102 + parms[51] * x136 + parms[52] * x87 + parms[57] * x151 + x103 * x125 + x109 * x133 + x112 * x176 + x135 * x175 + x170 * x91;
	double x178 = -parms[41] * x95 - parms[43] * x100 - parms[44] * x58 - parms[48] * x123 - x108 * x77 - x126 * x131 - x128 * x151 - x129 * x173 - x130 * x174 - m_d6 * x155 - x168 * x61 - x172 * x86 - x177 * x89;
	double x179 = -x73;
	double x180 = parms[32] * x53 + parms[34] * x56 + parms[35] * x98 + parms[36] * x119 - parms[37] * x122 + x141 * x71 + x142 * x179 + x143 * x167 + x166 * x29 + x178;
	double x181 = x154 * x30;
	double x182 = parms[41] * x32 + parms[43] * x76 + parms[44] * x61 + parms[46] * x175 + parms[48] * x74;
	double x183 = parms[52] * x102 + parms[54] * x136 + parms[55] * x87 + parms[56] * x125 + x106 * x176 + x115 * x135 - x116 * x133 + x134 * x137 + x153 * x171;
	double x184 = parms[42] * x95 + parms[44] * x100 + parms[45] * x58 + parms[46] * x85 - x114 * x77 + x123 * x156 + x150 * x74 + x152 * x168 + x182 * x32 + x183;
	double x185 = parms[40] * x95 + parms[41] * x100 + parms[42] * x58 + parms[47] * x151 + x114 * x129 + x124 * x172 - m_d6 * x127 + x131 * x138 + x139 * x85 + x150 * x175 + x174 * x76 + x177 * x86 + x182 * x62;
	double x186 = -x71;
	double x187 = parms[32] * x29 + parms[34] * x14 + parms[35] * x36 + parms[36] * x73 + parms[37] * x186;
	double x188 = parms[30] * x53 + parms[31] * x56 + parms[32] * x98 + parms[37] * x84 + x114 * x141 + x119 * x140 + x120 * x157 + x14 * x187 + x149 * x73 + x166 * x75 - m_d5 * x181 + x184 * x54 + x185 * x30;
	double x189 = parms[21] * x51 + parms[23] * x47 + parms[24] * x55 + parms[28] * x83 + x119 * x162 + x12 * x188 + x121 * x159 + x13 * x165 + m_d4 * x145 + x146 * x163 + x147 * x148 + x161 * x73 + x180 * x78;
	double x190 = -x29;
	double x191 = -x22;
	double x192 = parms[26] * x13 + parms[29] * x69 + x164 * x27;
	double x193 = parms[27] * x51 + parms[29] * x119 + parms[36] * x98 + parms[39] * x119 + x140 * x53 + x142 * x36 + x149 * x190 + x15 * x157 + x161 * x191 + x162 * x47 + x181 + x192 * x27;
	double x194 = x193 * x72;
	double x195 = x12 * x159;
	double x196 = parms[26] * x55 + parms[29] * x81 + x13 * x161 + x144 * x78 + x148 * x163 + x164 * x51 + x195;
	double x197 = x17 * x196;
	double x198 = -parms[16];
	double x199 = parms[17] * x1 + parms[19] * x10 + x198 * x9;
	double x200 = -x67;
	double x201 = dq[1] * parms[15] + parms[12] * x1 + parms[14] * x9 + parms[16] * x5 + parms[17] * x200;
	double x202 = dq[1] * parms[14] + parms[11] * x1 + parms[13] * x9 + parms[16] * x11 + parms[18] * x67;
	double x203 = parms[21] * x27 + parms[23] * x22 + parms[24] * x13 + parms[26] * x179 + parms[28] * x70;
	double x204 = parms[20] * x51 + parms[21] * x47 + parms[22] * x55 + parms[27] * x119 + x12 * x180 + x121 * x144 + x147 * x22 + x160 * x203 + x163 * x69 + x164 * x81 + x179 * x192 + x188 * x24 - m_d4 * x195;
	double x205 = -parms[27];
	double x206 = parms[28] * x47 + parms[29] * x83 + x145 + x159 * x24 + x160 * x192 + x163 * x22 + x205 * x55;
	double x207 = -x1;
	double x208 = x17 * x206;
	double x209 = dq[1] * parms[12] + parms[10] * x1 + parms[11] * x9 + parms[17] * x10 + x2 * x5;
	double x210 = parms[17] * x40 + parms[18] * x9 + parms[19] * x67;
	double x211 = ddq[1] * parms[16] + dq[1] * x210 + parms[19] * x80 + x19 * x206 + x197 + x199 * x207 + x2 * x44;
	double x212 = -parms[17];
	double x213 = -x9;
	double x214 = parms[31] * x53 + parms[33] * x56 + parms[34] * x98 + parms[36] * x85 + parms[38] * x122 + x113 * x142 + x120 * x154 + x149 * x186 + x15 * x185 + m_d5 * x158 + x167 * x36 + x184 * x30 + x187 * x190;
	double x215 = parms[22] * x51 + parms[24] * x47 + parms[25] * x55 + parms[26] * x81 - x161 * x69 + x165 * x191 + x192 * x70 + x203 * x27 + x205 * x83 + x214;
	//
	tau[0] = ddq[0] * parms[3] - m_d2 * x0 * x211 + x0 * (ddq[1] * parms[12] + parms[10] * x44 + parms[11] * x41 + parms[17] * x118 + x11 * x6 + x17 * x204 + x189 * x45 + x194 * x45 - m_d3 * x197 + x199 * x5 + x2 * x80 + x201 * x9 + x202 * x40 + x206 * x82) + x7 * x8 * (parms[17] * x44 + parms[19] * x118 + x1 * x6 + x193 + x198 * x41 + x210 * x213) + m_d2 * x7 * (ddq[1] * x212 + parms[18] * x41 + parms[19] * x79 + x196 * x45 + x199 * x9 + x208 + x40 * x6) + x7 * (ddq[1] * parms[14] + dq[1] * x209 + parms[11] * x44 + parms[13] * x41 + parms[18] * x79 + x10 * x210 + x118 * x198 + x17 * x189 + x17 * x194 + x19 * x204 + x196 * x82 + x199 * x200 + x201 * x207 + m_d3 * x208);
	tau[1] = ddq[1] * parms[15] + parms[12] * x44 + parms[14] * x41 + parms[16] * x80 + x1 * x202 + x196 * x66 + x209 * x213 - x210 * x5 + m_a2 * x211 + x212 * x79 + x215 + x6 * x67;
	tau[2] = m_a3 * x196 + x215;
	tau[3] = x214;
	tau[4] = x178;
	tau[5] = x183;

	//
	return;
}


void urDynamics::calculateJointFricition(const EcRealVector& dq, const EcRealVector& parms, EcReal coeffColomb, EcReal coeffViscous, EcRealVector& tau)
{
	tau[0] = coeffViscous * dq[0] * parms[11] + coeffColomb * parms[12] * sign(dq[0]);
	tau[1] = coeffViscous * dq[1] * parms[24] + coeffColomb * parms[25] * sign(dq[1]);
	tau[2] = coeffViscous * dq[2] * parms[37] + coeffColomb * parms[38] * sign(dq[2]);
	tau[3] = coeffViscous * dq[3] * parms[50] + coeffColomb * parms[51] * sign(dq[3]);
	tau[4] = coeffViscous * dq[4] * parms[63] + coeffColomb * parms[64] * sign(dq[4]);
	tau[5] = coeffViscous * dq[5] * parms[76] + coeffColomb * parms[77] * sign(dq[5]);
}