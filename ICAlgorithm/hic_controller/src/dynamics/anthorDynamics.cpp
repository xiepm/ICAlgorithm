#include "hic_controller/dynamics/anthorDynamics.h"
#include <iostream>

anthorDynamics::anthorDynamics()
{
	m_NumJoints = 7;
	m_gx = 0;
	m_gy = 0;
	m_gz = -9.81;

	m_d1 = 0.220;		//默认设置为ElfinV5的杆长参数；
	m_d2 = 0.420;
	m_d3 = 0.180;
	m_a1 = 0.380;
	m_a2 = 0.380;

	m_friTemperaturesParams.assign(m_NumJoints, 1.0);
	m_jointTemperatures.assign(m_NumJoints, 46.0);


	m_zeros.assign(m_NumJoints, 0.0);
}


anthorDynamics::~anthorDynamics()
{

}



void anthorDynamics::setRobotDHParameters
(
	const EcRealVector& kinematcisParam
)
{
	m_d1 = kinematcisParam[0];
	m_d2 = kinematcisParam[1];
	m_d3 = kinematcisParam[2];
	m_a1 = kinematcisParam[3];
	m_a2 = kinematcisParam[3];

}


void anthorDynamics::setGravityVector
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
}

void anthorDynamics::calculateGravityJointTorques
(
	const EcRealVector& q,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	calculateEstimateJointToqrues(q, m_zeros, m_zeros, parms, tau);
}


// calculate forward dynamics of elfin
EcBoolean anthorDynamics::calculateEstimateJointToqrues
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
	double x2 = m_gx * x0 - m_gy * x1;
	double x3 = -ddq[0];
	double x4 = -m_gy * x0 - m_gx * x1;
	double x5 = sin(q[1]);
	double x6 = -dq[0];
	double x7 = x5 * x6;
	double x8 = cos(q[1]);
	double x9 = -x8;
	double x10 = x6 * x9;
	double x11 = dq[1] * parms[16] + parms[14] * x7 + parms[17] * x10;
	double x12 = cos(q[2]);
	double x13 = m_d2 * x12;
	double x14 = sin(q[2]);
	double x15 = x14 * x7;
	double x16 = -dq[1];
	double x17 = x12 * x16 + x15;
	double x18 = -dq[2];
	double x19 = ddq[1] * x14;
	double x20 = x10 * x16 + x3 * x5;
	double x21 = x12 * x20;
	double x22 = x17 * x18 + x19 + x21;
	double x23 = sin(q[3]);
	double x24 = -x23;
	double x25 = dq[1] * x7 + x3 * x9;
	double x26 = ddq[2] + x25;
	double x27 = cos(q[3]);
	double x28 = dq[1] * x14;
	double x29 = x12 * x7;
	double x30 = x28 + x29;
	double x31 = dq[2] + x10;
	double x32 = x23 * x31 + x27 * x30;
	double x33 = -dq[3];
	double x34 = x22 * x24 + x26 * x27 + x32 * x33;
	double x35 = x24 * x30 + x27 * x31;
	double x36 = -x35;
	double x37 = dq[1] * x13 - m_d2 * x15;
	double x38 = x27 * x37;
	double x39 = -x17;
	double x40 = x33 + x39;
	double x41 = parms[46] * x36 + parms[47] * x40 + parms[48] * x38;
	double x42 = -x12;
	double x43 = x4 * x8 + m_gz * x5;
	double x44 = dq[2] * x37 + x14 * x43 + m_d2 * x19 + x2 * x42 + m_d2 * x21;
	double x45 = -x44;
	double x46 = -x33;
	double x47 = x22 * x27 + x23 * x26 + x35 * x46;
	double x48 = -parms[47];
	double x49 = sin(q[4]);
	double x50 = -x49;
	double x51 = cos(q[4]);
	double x52 = m_d2 * x28 + m_d2 * x29;
	double x53 = -x52;
	double x54 = x40 * x49;
	double x55 = x32 * x51;
	double x56 = x38 * x50 + x51 * x53 - m_d3 * x54 - m_d3 * x55;
	double x57 = -dq[4];
	double x58 = -x57;
	double x59 = x14 * x20;
	double x60 = ddq[1] * x42 + dq[2] * x30 + x59;
	double x61 = -x60;
	double x62 = -ddq[3] + x61;
	double x63 = x51 * x62;
	double x64 = -m_d3 * x49;
	double x65 = x24 * x37;
	double x66 = x4 * x5 - m_gz * x8;
	double x67 = ddq[1] * x13 + x12 * x43 + x14 * x2 + x18 * x52 - m_d2 * x59;
	double x68 = x23 * x66 + x27 * x67 + x46 * x65;
	double x69 = x45 * x49 + x47 * x64 + x51 * x68 + x56 * x58 + m_d3 * x63;
	double x70 = x32 * x49;
	double x71 = x40 * x51;
	double x72 = -x70 + x71;
	double x73 = -x72;
	double x74 = x54 + x55;
	double x75 = -parms[60];
	double x76 = -x65;
	double x77 = parms[58] * x72 + parms[61] * x76 + x74 * x75;
	double x78 = cos(q[5]);
	double x79 = -dq[5];
	double x80 = x73 + x79;
	double x81 = sin(q[6]);
	double x82 = sin(q[5]);
	double x83 = x36 + x57;
	double x84 = -x78;
	double x85 = x74 * x82 + x83 * x84;
	double x86 = cos(q[6]);
	double x87 = x80 * x81 + x85 * x86;
	double x88 = -dq[6];
	double x89 = x47 * x50 + x57 * x74 + x63;
	double x90 = -x89;
	double x91 = -ddq[5] + x90;
	double x92 = x86 * x91;
	double x93 = x82 * x83;
	double x94 = x74 * x78;
	double x95 = x93 + x94;
	double x96 = -x79;
	double x97 = x47 * x51;
	double x98 = x49 * x62;
	double x99 = x58 * x72 + x97 + x98;
	double x100 = -x34;
	double x101 = -ddq[4] + x100;
	double x102 = x101 * x84 + x82 * x99 + x95 * x96;
	double x103 = -x81;
	double x104 = x102 * x103;
	double x105 = x104 + x87 * x88 + x92;
	double x106 = -parms[84];
	double x107 = x80 * x86;
	double x108 = -x85;
	double x109 = x108 * x81;
	double x110 = x107 + x109;
	double x111 = dq[6] * x110 + x102 * x86 + x81 * x91;
	double x112 = m_a2 * dq[6];
	double x113 = ((x81) * (x81));
	double x114 = ((x86) * (x86));
	double x115 = -m_a2 * x113 - m_a2 * x114;
	double x116 = x38 * x51 + x49 * x53 - m_d3 * x70 + m_d3 * x71;
	double x117 = x116 * x82 + x76 * x84;
	double x118 = m_a1 * ((x78) * (x78)) + m_a1 * ((x82) * (x82));
	double x119 = x24 * x67 + x27 * x66 + x33 * x38;
	double x120 = -x119;
	double x121 = m_a1 * ddq[5] + x117 * x79 + x118 * x89 + x120 * x82 + x69 * x78;
	double x122 = x104 * x115 + x112 * x87 + x115 * x92 + x121;
	double x123 = dq[6] + x95;
	double x124 = -x123;
	double x125 = -x56;
	double x126 = x118 * x93 + x118 * x94 + x125;
	double x127 = x117 * x86 + x126 * x81;
	double x128 = parms[85] * x124 + parms[86] * x110 + parms[87] * x127;
	double x129 = -x110;
	double x130 = m_a2 * x113 + m_a2 * x114;
	double x131 = x103 * x117 + x126 * x86 + x130 * x95;
	double x132 = x112 + x131;
	double x133 = -parms[86];
	double x134 = parms[84] * x123 + parms[87] * x132 + x133 * x87;
	double x135 = parms[85] * x111 + parms[87] * x122 + x105 * x106 + x128 * x129 + x134 * x87;
	double x136 = -x95;
	double x137 = parms[72] * x136 + parms[73] * x80 + parms[74] * x117;
	double x138 = -x80;
	double x139 = -parms[71];
	double x140 = parms[71] * x95 + parms[73] * x108 + parms[74] * x126;
	double x141 = parms[72] * x102 + parms[74] * x121 + x135 + x137 * x138 + x139 * x91 + x140 * x85;
	double x142 = m_a1 * dq[5];
	double x143 = x116 * x78 + x118 * x72 + x76 * x82;
	double x144 = x142 + x143;
	double x145 = parms[72] * x85 + parms[74] * x144 + x139 * x80;
	double x146 = x120 * x84 + x142 * x73 + x143 * x96 + x69 * x82;
	double x147 = x116 * x57 + x45 * x51 + x50 * x68 - m_d3 * x97 - m_d3 * x98;
	double x148 = -x147;
	double x149 = x101 * x82;
	double x150 = x78 * x99;
	double x151 = x108 * x142 + x118 * x149 + x118 * x150 + x148;
	double x152 = x149 + x150 + x79 * x85;
	double x153 = m_a2 * ddq[6] + x103 * x146 + x127 * x88 + x130 * x152 + x151 * x86;
	double x154 = ddq[6] + x152;
	double x155 = x107 * x115 + x109 * x115 + x144;
	double x156 = parms[85] * x87 + parms[87] * x155 + x106 * x110;
	double x157 = -x87;
	double x158 = parms[84] * x154 + parms[87] * x153 + x111 * x133 + x123 * x128 + x156 * x157;
	double x159 = -parms[72];
	double x160 = -parms[85];
	double x161 = dq[6] * x131 + x112 * x136 + x146 * x86 + x151 * x81;
	double x162 = parms[86] * x105 + parms[87] * x161 + x110 * x156 + x124 * x134 + x154 * x160;
	double x163 = parms[73] * x91 + parms[74] * x146 + x103 * x158 + x136 * x140 + x145 * x80 + x152 * x159 + x162 * x86;
	double x164 = -x83;
	double x165 = parms[58] * x164 + parms[59] * x74 + parms[61] * x56;
	double x166 = parms[59] * x90 + parms[60] * x101 + parms[61] * x69 + x141 * x78 + x163 * x82 + x165 * x83 + x73 * x77;
	double x167 = -parms[73];
	double x168 = parms[71] * x152 + parms[74] * x151 + x102 * x167 + x108 * x145 + x137 * x95 + x158 * x86 + x162 * x81;
	double x169 = parms[59] * x73 + parms[60] * x83 + parms[61] * x116;
	double x170 = -parms[58] * x101 + parms[59] * x99 + parms[61] * x147 + x164 * x169 - x168 + x74 * x77;
	double x171 = x170 * x51;
	double x172 = -x40;
	double x173 = parms[45] * x172 + parms[46] * x32 + parms[48] * x65;
	double x174 = -x32;
	double x175 = -parms[34];
	double x176 = parms[32] * x17 + x175 * x30;
	double x177 = parms[33] * x39 + parms[34] * x31 + parms[35] * x37;
	double x178 = -x31;
	double x179 = -parms[32] * x26 + parms[33] * x22 + parms[35] * x44 - parms[45] * x34 - parms[48] * x45 - x166 * x49 - x171 - x173 * x174 + x176 * x30 + x177 * x178 - x35 * x41 - x47 * x48;
	double x180 = parms[27] * x30 + parms[29] * x31 + parms[30] * x17 + parms[32] * x53 + parms[34] * x37;
	double x181 = parms[26] * x30 + parms[27] * x31 + parms[28] * x17 + parms[33] * x52;
	double x182 = -x116;
	double x183 = parms[54] * x74 + parms[56] * x83 + parms[57] * x72 + parms[58] * x76 + parms[59] * x182;
	double x184 = parms[65] * x85 + parms[66] * x80 + parms[67] * x95 + parms[72] * x144 + x126 * x167;
	double x185 = parms[78] * x87 + parms[79] * x110 + parms[80] * x123 + parms[85] * x155 + x132 * x133;
	double x186 = -x155;
	double x187 = parms[79] * x87 + parms[81] * x110 + parms[82] * x123 + parms[84] * x186 + parms[86] * x127;
	double x188 = parms[80] * x111 + parms[82] * x105 + parms[83] * x154 + parms[84] * x153 + x127 * x134 - x128 * x132 + x129 * x185 + x160 * x161 + x187 * x87;
	double x189 = parms[66] * x85 + parms[68] * x80 + parms[69] * x95 + parms[73] * x117 + x139 * x144;
	double x190 = parms[67] * x102 + parms[69] * x91 + parms[70] * x152 + parms[71] * x151 + x117 * x140 - x126 * x137 + x130 * x158 + x138 * x184 + x146 * x159 + x188 + x189 * x85;
	double x191 = parms[53] * x74 + parms[55] * x83 + parms[56] * x72 + parms[58] * x125 + parms[60] * x116;
	double x192 = -x127;
	double x193 = parms[80] * x87 + parms[82] * x110 + parms[83] * x123 + parms[84] * x132 + parms[85] * x192;
	double x194 = parms[79] * x111 + parms[81] * x105 + parms[82] * x154 + parms[86] * x161 + x106 * x122 + x123 * x185 + x128 * x155 + x156 * x192 + x157 * x193;
	double x195 = -x117;
	double x196 = parms[67] * x85 + parms[69] * x80 + parms[70] * x95 + parms[71] * x126 + parms[72] * x195;
	double x197 = x115 * x135;
	double x198 = parms[78] * x111 + parms[79] * x105 + parms[80] * x154 + parms[85] * x122 + x110 * x193 + x124 * x187 + x132 * x156 + x133 * x153 + x134 * x186;
	double x199 = parms[65] * x102 + parms[66] * x91 + parms[67] * x152 + parms[72] * x121 + x103 * x194 + x103 * x197 + x126 * x145 + x136 * x189 - x140 * x144 + x151 * x167 + x196 * x80 + x198 * x86;
	double x200 = x118 * x168;
	double x201 = parms[52] * x99 + parms[53] * x101 + parms[54] * x89 + parms[59] * x147 + x120 * x75 + x125 * x77 + x165 * x76 + x183 * x83 + x190 * x78 + x191 * x73 + x199 * x82 + x200 * x78;
	double x202 = parms[39] * x32 + parms[40] * x40 + parms[41] * x35 + parms[46] * x65 + x48 * x53;
	double x203 = x166 * x51;
	double x204 = -x38;
	double x205 = parms[41] * x32 + parms[43] * x40 + parms[44] * x35 + parms[45] * x53 + parms[46] * x204;
	double x206 = -parms[66] * x102 - parms[68] * x91 - parms[69] * x152 - parms[73] * x146 - x108 * x196 - x121 * x139 - x137 * x144 - x145 * x195 - x184 * x95 - x194 * x86 - x197 * x86 - x198 * x81;
	double x207 = parms[52] * x74 + parms[53] * x83 + parms[54] * x72 + parms[59] * x56 + x75 * x76;
	double x208 = parms[54] * x99 + parms[56] * x101 + parms[57] * x89 + parms[58] * x120 - parms[59] * x69 + x116 * x77 + x118 * x141 + x164 * x207 - x169 * x76 + x191 * x74 + x206;
	double x209 = -parms[40] * x47 - parms[42] * x62 - parms[43] * x34 - parms[45] * x120 - parms[47] * x68 - x170 * x64 - x173 * x204 - x174 * x205 - x201 * x49 - x202 * x35 - m_d3 * x203 - x208 * x51 - x41 * x65;
	double x210 = parms[28] * x22 + parms[30] * x26 + parms[31] * x60 + parms[32] * x66 - parms[33] * x67 + x176 * x37 + x178 * x181 + x180 * x30 + x209;
	double x211 = parms[32] * x178 + parms[33] * x30 + parms[35] * x52;
	double x212 = parms[45] * x35 + parms[48] * x53 + x32 * x48;
	double x213 = -x74;
	double x214 = parms[33] * x61 + parms[34] * x26 + parms[35] * x67 + x176 * x39 + x211 * x31 + x24 * (-parms[45] * x62 + parms[46] * x47 + parms[48] * x119 - parms[58] * x89 - parms[61] * x120 - x141 * x82 - x163 * x84 - x165 * x213 - x169 * x72 + x172 * x41 + x212 * x32 - x75 * x99) + x27 * (parms[46] * x100 + parms[47] * x62 + parms[48] * x68 + x170 * x50 + x173 * x40 + x203 + x212 * x36);
	double x215 = dq[1] * parms[17] + parms[15] * x7 + parms[18] * x10;
	double x216 = parms[40] * x32 + parms[42] * x40 + parms[43] * x35 + parms[45] * x76 + parms[47] * x38;
	double x217 = -parms[53] * x99 - parms[55] * x101 - parms[56] * x89 - parms[58] * x148 - parms[60] * x69 - x165 * x182 - x169 * x56 - x183 * x213 - x190 * x82 - x199 * x84 - x200 * x82 - x207 * x72;
	double x218 = parms[41] * x47 + parms[43] * x62 + parms[44] * x34 + parms[45] * x45 - parms[46] * x68 + x172 * x202 + x212 * x38 + x216 * x32 + x217 - x41 * x53;
	double x219 = -x37;
	double x220 = parms[28] * x30 + parms[30] * x31 + parms[31] * x17 + parms[33] * x219;
	double x221 = parms[39] * x47 + parms[40] * x62 + parms[41] * x34 + parms[46] * x119 + x166 * x64 - m_d3 * x171 + x173 * x53 + x201 * x51 + x205 * x40 + x208 * x50 + x212 * x76 + x216 * x36 + x45 * x48;
	double x222 = parms[26] * x22 + parms[27] * x26 + parms[28] * x60 + parms[33] * x44 + x175 * x66 + x176 * x53 + x180 * x39 + x218 * x24 + x220 * x31 + x221 * x27;
	double x223 = dq[1] * parms[14] + parms[13] * x7 + parms[15] * x10;
	double x224 = parms[27] * x22 + parms[29] * x26 + parms[30] * x60 + parms[32] * x45 + parms[34] * x67 + x17 * x181 + x177 * x52 + x211 * x219 + x218 * x27 - x220 * x30 + x221 * x23;
	//
	tau[0] = ddq[0] * parms[10] + dq[0] * parms[11] + parms[12] * sign(dq[0]) - parms[3] * x3 + parms[6] * x2 - parms[8] * x4 - x5 * (ddq[1] * parms[14] + dq[1] * x215 + parms[13] * x20 + parms[15] * x25 + parms[20] * x66 - parms[21] * x2 - x10 * x11 + x12 * x222 + x13 * x179 + x14 * x210 - m_d2 * x14 * x214) - x9 * (ddq[1] * parms[17] + parms[15] * x20 + parms[18] * x25 + parms[19] * x2 - parms[20] * x43 + x11 * x7 + x16 * x223 + x224);
	tau[1] = ddq[1] * parms[16] + ddq[1] * parms[23] + dq[1] * parms[24] + parms[14] * x20 + parms[17] * x25 - parms[19] * x66 + parms[21] * x43 + parms[25] * sign(dq[1]) + x10 * x223 + x13 * x214 +
		m_d2 * x14 * x179 + x14 * x222 + x210 * x42 - x215 * x7;
	tau[2] = ddq[2] * parms[36] + dq[2] * parms[37] + parms[38] * sign(dq[2]) + x224;
	tau[3] = ddq[3] * parms[49] + dq[3] * parms[50] + parms[51] * sign(dq[3]) + x209;
	tau[4] = ddq[4] * parms[62] + dq[4] * parms[63] + parms[64] * sign(dq[4]) + x217;
	tau[5] = ddq[5] * parms[75] + dq[5] * parms[76] + parms[77] * sign(dq[5]) + m_a1 * x141 + x206;
	tau[6] = ddq[6] * parms[88] + dq[6] * parms[89] + parms[90] * sign(dq[6]) + m_a2 * x158 + x188;




	return true;
}


// exclude gravity, friction, motor inertia;
void anthorDynamics::calculateMomentumEstimatedJointTorques
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	double x0 = -ddq[0];
	double x1 = -cos(q[1]);
	double x2 = cos(q[2]);
	double x3 = m_d2 * x2;
	double x4 = sin(q[1]);
	double x5 = -dq[1];
	double x6 = -dq[0];
	double x7 = x1 * x6;
	double x8 = x0 * x4 + x5 * x7;
	double x9 = sin(q[2]);
	double x10 = x8 * x9;
	double x11 = dq[1] * x9;
	double x12 = x4 * x6;
	double x13 = x12 * x2;
	double x14 = m_d2 * x11 + m_d2 * x13;
	double x15 = -dq[2];
	double x16 = ddq[1] * x3 - m_d2 * x10 + x14 * x15;
	double x17 = x12 * x9;
	double x18 = x17 + x2 * x5;
	double x19 = x11 + x13;
	double x20 = dq[2] + x7;
	double x21 = parms[22] * x19 + parms[23] * x20 + parms[24] * x18 + parms[29] * x14;
	double x22 = sin(q[3]);
	double x23 = sin(q[5]);
	double x24 = cos(q[3]);
	double x25 = x19 * x24 + x20 * x22;
	double x26 = sin(q[4]);
	double x27 = x25 * x26;
	double x28 = -dq[3];
	double x29 = -x18;
	double x30 = x28 + x29;
	double x31 = cos(q[4]);
	double x32 = x30 * x31;
	double x33 = -x27 + x32;
	double x34 = -x33;
	double x35 = -dq[5];
	double x36 = x34 + x35;
	double x37 = dq[1] * x3 - m_d2 * x17;
	double x38 = -x22;
	double x39 = x37 * x38;
	double x40 = -x39;
	double x41 = cos(q[5]);
	double x42 = -x41;
	double x43 = -x14;
	double x44 = x24 * x37;
	double x45 = x26 * x43 - m_d3 * x27 + x31 * x44 + m_d3 * x32;
	double x46 = x23 * x45 + x40 * x42;
	double x47 = -x46;
	double x48 = -dq[4];
	double x49 = x19 * x38 + x20 * x24;
	double x50 = -x49;
	double x51 = x48 + x50;
	double x52 = x23 * x51;
	double x53 = x25 * x31;
	double x54 = x26 * x30;
	double x55 = x53 + x54;
	double x56 = x41 * x55;
	double x57 = x52 + x56;
	double x58 = -x26;
	double x59 = x31 * x43 + x44 * x58 - m_d3 * x53 - m_d3 * x54;
	double x60 = -x59;
	double x61 = m_a1 * ((x23) * (x23)) + m_a1 * ((x41) * (x41));
	double x62 = x52 * x61 + x56 * x61 + x60;
	double x63 = x23 * x55 + x42 * x51;
	double x64 = parms[57] * x63 + parms[59] * x36 + parms[60] * x57 + parms[61] * x62 + parms[62] * x47;
	double x65 = -x63;
	double x66 = parms[61] * x57 + parms[63] * x65 + parms[64] * x62;
	double x67 = x23 * x40 + x33 * x61 + x41 * x45;
	double x68 = m_a1 * dq[5];
	double x69 = x67 + x68;
	double x70 = -x69;
	double x71 = sin(q[6]);
	double x72 = -x71;
	double x73 = ((x71) * (x71));
	double x74 = cos(q[6]);
	double x75 = ((x74) * (x74));
	double x76 = -m_a2 * x73 - m_a2 * x75;
	double x77 = x36 * x71 + x63 * x74;
	double x78 = m_a2 * dq[6];
	double x79 = -x2;
	double x80 = ddq[1] * x79 + dq[2] * x19 + x10;
	double x81 = -x80;
	double x82 = -ddq[3] + x81;
	double x83 = x31 * x82;
	double x84 = dq[1] * x12 + x0 * x1;
	double x85 = ddq[2] + x84;
	double x86 = ddq[1] * x9;
	double x87 = x2 * x8;
	double x88 = x15 * x18 + x86 + x87;
	double x89 = -x28;
	double x90 = x22 * x85 + x24 * x88 + x49 * x89;
	double x91 = x48 * x55 + x58 * x90 + x83;
	double x92 = -x91;
	double x93 = -ddq[5] + x92;
	double x94 = x74 * x93;
	double x95 = -x48;
	double x96 = x16 * x24 + x39 * x89;
	double x97 = -m_d3 * x26;
	double x98 = dq[2] * x37 + m_d2 * x86 + m_d2 * x87;
	double x99 = -x98;
	double x100 = x26 * x99 + x31 * x96 + x59 * x95 + m_d3 * x83 + x90 * x97;
	double x101 = x16 * x38 + x28 * x44;
	double x102 = -x101;
	double x103 = m_a1 * ddq[5] + x100 * x41 + x102 * x23 + x35 * x46 + x61 * x91;
	double x104 = -x35;
	double x105 = x24 * x85 + x25 * x28 + x38 * x88;
	double x106 = -x105;
	double x107 = -ddq[4] + x106;
	double x108 = x26 * x82;
	double x109 = x31 * x90;
	double x110 = x108 + x109 + x33 * x95;
	double x111 = x104 * x57 + x107 * x42 + x110 * x23;
	double x112 = x111 * x72;
	double x113 = x103 + x112 * x76 + x76 * x94 + x77 * x78;
	double x114 = -dq[6];
	double x115 = x112 + x114 * x77 + x94;
	double x116 = -parms[72];
	double x117 = x65 * x71;
	double x118 = x36 * x74;
	double x119 = x117 + x118;
	double x120 = dq[6] * x119 + x111 * x74 + x71 * x93;
	double x121 = m_a2 * x73 + m_a2 * x75;
	double x122 = x121 * x57 + x46 * x72 + x62 * x74;
	double x123 = x122 + x78;
	double x124 = dq[6] + x57;
	double x125 = -parms[74];
	double x126 = parms[72] * x124 + parms[75] * x123 + x125 * x77;
	double x127 = x46 * x74 + x62 * x71;
	double x128 = -x124;
	double x129 = parms[73] * x128 + parms[74] * x119 + parms[75] * x127;
	double x130 = -x129;
	double x131 = parms[73] * x120 + parms[75] * x113 + x115 * x116 + x119 * x130 + x126 * x77;
	double x132 = x131 * x76;
	double x133 = -parms[61];
	double x134 = parms[62] * x63 + parms[64] * x69 + x133 * x36;
	double x135 = parms[56] * x63 + parms[58] * x36 + parms[59] * x57 + parms[61] * x70 + parms[63] * x46;
	double x136 = -x57;
	double x137 = x110 * x41;
	double x138 = -m_d3 * x108 - m_d3 * x109 + x31 * x99 + x45 * x48 + x58 * x96;
	double x139 = -x138;
	double x140 = x107 * x23;
	double x141 = x137 * x61 + x139 + x140 * x61 + x65 * x68;
	double x142 = -parms[63];
	double x143 = x137 + x140 + x35 * x63;
	double x144 = x100 * x23 + x102 * x42 + x104 * x67 + x34 * x68;
	double x145 = m_a2 * ddq[6] + x114 * x127 + x121 * x143 + x141 * x74 + x144 * x72;
	double x146 = x117 * x76 + x118 * x76 + x69;
	double x147 = -x146;
	double x148 = ddq[6] + x143;
	double x149 = parms[73] * x77 + parms[75] * x146 + x116 * x119;
	double x150 = parms[67] * x77 + parms[69] * x119 + parms[70] * x124 + parms[72] * x147 + parms[74] * x127;
	double x151 = -x127;
	double x152 = parms[68] * x77 + parms[70] * x119 + parms[71] * x124 + parms[72] * x123 + parms[73] * x151;
	double x153 = parms[66] * x120 + parms[67] * x115 + parms[68] * x148 + parms[73] * x113 + x119 * x152 + x123 * x149 + x125 * x145 + x126 * x147 + x128 * x150;
	double x154 = -x77;
	double x155 = parms[66] * x77 + parms[67] * x119 + parms[68] * x124 + parms[73] * x146 + x123 * x125;
	double x156 = dq[6] * x122 + x136 * x78 + x141 * x71 + x144 * x74;
	double x157 = parms[67] * x120 + parms[69] * x115 + parms[70] * x148 + parms[74] * x156 + x113 * x116 + x124 * x155 + x129 * x146 + x149 * x151 + x152 * x154;
	double x158 = parms[55] * x111 + parms[56] * x93 + parms[57] * x143 + parms[62] * x103 + x132 * x72 + x134 * x62 + x135 * x136 + x141 * x142 + x153 * x74 + x157 * x72 + x36 * x64 + x66 * x70;
	double x159 = -parms[52];
	double x160 = parms[72] * x148 + parms[75] * x145 + x120 * x125 + x124 * x129 + x149 * x154;
	double x161 = parms[55] * x63 + parms[56] * x36 + parms[57] * x57 + parms[62] * x69 + x142 * x62;
	double x162 = -x36;
	double x163 = -parms[73];
	double x164 = parms[68] * x120 + parms[70] * x115 + parms[71] * x148 + parms[72] * x145 - x119 * x155 + x123 * x130 + x126 * x127 + x150 * x77 + x156 * x163;
	double x165 = -parms[62];
	double x166 = parms[62] * x136 + parms[63] * x36 + parms[64] * x46;
	double x167 = parms[57] * x111 + parms[59] * x93 + parms[60] * x143 + parms[61] * x141 + x121 * x160 + x135 * x63 + x144 * x165 + x161 * x162 + x164 - x166 * x62 + x46 * x66;
	double x168 = parms[45] * x55 + parms[47] * x51 + parms[48] * x33 + parms[50] * x60 + parms[52] * x45;
	double x169 = -x45;
	double x170 = parms[46] * x55 + parms[48] * x51 + parms[49] * x33 + parms[50] * x40 + parms[51] * x169;
	double x171 = -x51;
	double x172 = parms[50] * x171 + parms[51] * x55 + parms[53] * x59;
	double x173 = parms[50] * x33 + parms[53] * x40 + x159 * x55;
	double x174 = parms[74] * x115 + parms[75] * x156 + x119 * x149 + x126 * x128 + x148 * x163;
	double x175 = parms[61] * x143 + parms[64] * x141 + x111 * x142 + x134 * x65 + x160 * x74 + x166 * x57 + x174 * x71;
	double x176 = x175 * x61;
	double x177 = parms[44] * x110 + parms[45] * x107 + parms[46] * x91 + parms[51] * x138 + x102 * x159 + x158 * x23 + x167 * x41 + x168 * x34 + x170 * x51 + x172 * x40 + x173 * x60 + x176 * x41;
	double x178 = -x30;
	double x179 = parms[39] * x178 + parms[40] * x25 + parms[42] * x39;
	double x180 = parms[51] * x34 + parms[52] * x51 + parms[53] * x45;
	double x181 = -parms[50] * x107 + parms[51] * x110 + parms[53] * x138 + x171 * x180 + x173 * x55 - x175;
	double x182 = x181 * x31;
	double x183 = -parms[41];
	double x184 = parms[39] * x49 + parms[42] * x43 + x183 * x25;
	double x185 = -x44;
	double x186 = parms[35] * x25 + parms[37] * x30 + parms[38] * x49 + parms[39] * x43 + parms[40] * x185;
	double x187 = parms[34] * x25 + parms[36] * x30 + parms[37] * x49 + parms[39] * x40 + parms[41] * x44;
	double x188 = -parms[56] * x111 - parms[58] * x93 - parms[59] * x143 - parms[63] * x144 - x103 * x133 - x132 * x74 - x134 * x47 - x153 * x71 - x157 * x74 - x161 * x57 - x166 * x69 - x64 * x65;
	double x189 = parms[44] * x55 + parms[45] * x51 + parms[46] * x33 + parms[51] * x59 + x159 * x40;
	double x190 = parms[62] * x111 + parms[64] * x103 + x131 + x133 * x93 + x162 * x166 + x63 * x66;
	double x191 = parms[46] * x110 + parms[48] * x107 + parms[49] * x91 + parms[50] * x102 - parms[51] * x100 + x168 * x55 + x171 * x189 + x173 * x45 - x180 * x40 + x188 + x190 * x61;
	double x192 = parms[63] * x93 + parms[64] * x144 + x134 * x36 + x136 * x66 + x143 * x165 + x160 * x72 + x174 * x74;
	double x193 = parms[51] * x92 + parms[52] * x107 + parms[53] * x100 + x172 * x51 + x173 * x34 + x190 * x41 + x192 * x23;
	double x194 = parms[33] * x90 + parms[34] * x82 + parms[35] * x105 + parms[40] * x101 + x177 * x31 + x179 * x43 - m_d3 * x182 + x183 * x99 + x184 * x40 + x186 * x30 + x187 * x50 + x191 * x58 + x193 * x97;
	double x195 = -x37;
	double x196 = parms[24] * x19 + parms[26] * x20 + parms[27] * x18 + parms[29] * x195;
	double x197 = -x19;
	double x198 = -x20;
	double x199 = parms[28] * x198 + parms[29] * x19 + parms[31] * x14;
	double x200 = parms[40] * x50 + parms[41] * x30 + parms[42] * x44;
	double x201 = parms[33] * x25 + parms[34] * x30 + parms[35] * x49 + parms[40] * x39 + x183 * x43;
	double x202 = -x55;
	double x203 = -parms[45] * x110 - parms[47] * x107 - parms[48] * x91 - parms[50] * x139 - parms[52] * x100 - x158 * x42 - x167 * x23 - x169 * x172 - x170 * x202 - x176 * x23 - x180 * x59 - x189 * x33;
	double x204 = parms[35] * x90 + parms[37] * x82 + parms[38] * x105 + parms[39] * x99 - parms[40] * x96 + x178 * x201 + x184 * x44 + x187 * x25 - x200 * x43 + x203;
	double x205 = parms[29] * x29 + parms[30] * x20 + parms[31] * x37;
	double x206 = parms[23] * x88 + parms[25] * x85 + parms[26] * x80 + parms[28] * x99 + parms[30] * x16 + x14 * x205 + x18 * x21 + x194 * x22 + x195 * x199 + x196 * x197 + x204 * x24;
	double x207 = dq[1] * parms[14] + parms[12] * x12 + parms[15] * x7;
	double x208 = dq[1] * parms[12] + parms[11] * x12 + parms[13] * x7;
	double x209 = dq[1] * parms[15] + parms[13] * x12 + parms[16] * x7;
	double x210 = parms[28] * x18 + parms[30] * x197;
	double x211 = -x25;
	double x212 = x193 * x31;
	double x213 = -parms[34] * x90 - parms[36] * x82 - parms[37] * x105 - parms[39] * x102 - parms[41] * x96 - x177 * x26 - x179 * x185 - x181 * x97 - x186 * x211 - x191 * x31 - x200 * x39 - x201 * x49 - m_d3 * x212;
	double x214 = parms[23] * x19 + parms[25] * x20 + parms[26] * x18 + parms[28] * x43 + parms[30] * x37;
	double x215 = parms[24] * x88 + parms[26] * x85 + parms[27] * x80 - parms[29] * x16 + x19 * x214 + x198 * x21 + x210 * x37 + x213;
	double x216 = parms[29] * x81 + parms[30] * x85 + parms[31] * x16 + x199 * x20 + x210 * x29 + x24 * (parms[40] * x106 + parms[41] * x82 + parms[42] * x96 + x179 * x30 + x181 * x58 + x184 * x50 + x212) +
		x38 * (-parms[39] * x82 + parms[40] * x90 + parms[42] * x101 - parms[50] * x91 - parms[53] * x102 - x110 * x159 - x172 * x202 + x178 * x200 - x180 * x33 + x184 * x25 - x190 * x23 - x192 * x42);
	double x217 = parms[22] * x88 + parms[23] * x85 + parms[24] * x80 + parms[29] * x98 + x194 * x24 + x196 * x20 + x204 * x38 + x210 * x43 + x214 * x29;
	double x218 = -parms[28] * x85 + parms[29] * x88 + parms[31] * x98 - parms[39] * x105 - parms[42] * x99 - x179 * x211 - x182 - x183 * x90 + x19 * x210 - x193 * x26 + x198 * x205 - x200 * x49;
	//
	tau[0] = ddq[0] * parms[10] - parms[3] * x0 - x1 * (ddq[1] * parms[15] + parms[13] * x8 + parms[16] * x84 + x12 * x207 + x206 + x208 * x5) - x4 * (ddq[1] * parms[12] + dq[1] * x209 + parms[11] * x8 + parms[13] * x84 + x2 * x217 - x207 * x7 + x215 * x9 - m_d2 * x216 * x9 + x218 * x3);
	tau[1] = ddq[1] * parms[14] + ddq[1] * parms[21] + parms[12] * x8 + parms[15] * x84 - x12 * x209 + x208 * x7 + x215 * x79 + x216 * x3 + x217 * x9 + m_d2 * x218 * x9;
	tau[2] = ddq[2] * parms[32] + x206;
	tau[3] = ddq[3] * parms[43] + x213;
	tau[4] = ddq[4] * parms[54] + x203;
	tau[5] = ddq[5] * parms[65] + x188 + m_a1 * x190;
	tau[6] = ddq[6] * parms[76] + m_a2 * x160 + x164;
	//
	return;
}
