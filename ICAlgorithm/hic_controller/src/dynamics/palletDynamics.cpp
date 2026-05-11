#include "hic_controller/dynamics/palletDynamics.h"
#include <iostream>

palletDynamics::palletDynamics()
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


palletDynamics::~palletDynamics()
{

}



void palletDynamics::setRobotDHParameters
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

void palletDynamics::setGravityVector
(
	const EcReal gx, const EcReal gy, const EcReal gz
)
{
	m_gx = gx;
	m_gy = gy;
	m_gz = gz;
}

void palletDynamics::calculateGravityJointTorques
(
	const EcRealVector& q,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	calculateEstimateJointToqrues(q, m_zeros, m_zeros, parms, tau);
}


// calculate forward dynamics of elfin
EcBoolean palletDynamics::calculateEstimateJointToqrues
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	double x0 = cos(q[1]);
	double x1 = sin(q[2]);
	double x2 = sin(q[1]);
	double x3 = dq[0] * x2;
	double x4 = x1 * x3;
	double x5 = -x4;
	double x6 = dq[0] * x0;
	double x7 = cos(q[2]);
	double x8 = x6 * x7;
	double x9 = x5 + x8;
	double x10 = ddq[0] * x2;
	double x11 = dq[1] * x6 + x10;
	double x12 = x11 * x7;
	double x13 = ddq[0] * x0;
	double x14 = -dq[1];
	double x15 = x13 + x14 * x3;
	double x16 = x1 * x15;
	double x17 = dq[2] * x9 + x12 + x16;
	double x18 = -parms[34];
	double x19 = ddq[1] + ddq[2];
	double x20 = -dq[2];
	double x21 = m_d2 * x6;
	double x22 = -m_d2 * x3;
	double x23 = m_a2 * dq[1];
	double x24 = x22 + x23;
	double x25 = x1 * x24 + x21 * x7 - m_d3 * x4 + m_d3 * x8;
	double x26 = ((x7) * (x7));
	double x27 = ((x1) * (x1));
	double x28 = m_a3 * x26 + m_a3 * x27;
	double x29 = sin(q[0]);
	double x30 = cos(q[0]);
	double x31 = -m_gy * x29 - m_gx * x30;
	double x32 = m_a2 * ddq[1] - m_gz * x0 - m_d2 * x10 + x14 * x21 - x2 * x31;
	double x33 = dq[1] * x22 + x0 * x31 + m_d2 * x13 - m_gz * x2;
	double x34 = -x1;
	double x35 = ddq[1] * x28 + m_a3 * ddq[2] - m_d3 * x12 - m_d3 * x16 + x20 * x25 + x32 * x7 + x33 * x34;
	double x36 = cos(q[3]);
	double x37 = -x36;
	double x38 = ddq[3] + x19;
	double x39 = sin(q[4]);
	double x40 = x38 * x39;
	double x41 = sin(q[3]);
	double x42 = x1 * x6;
	double x43 = x3 * x7;
	double x44 = x42 + x43;
	double x45 = x41 * x44;
	double x46 = x36 * x9;
	double x47 = x45 - x46;
	double x48 = -dq[3];
	double x49 = x17 * x36;
	double x50 = x15 * x7;
	double x51 = x11 * x34;
	double x52 = x20 * x44 + x50 + x51;
	double x53 = x41 * x52;
	double x54 = x47 * x48 + x49 + x53;
	double x55 = cos(q[4]);
	double x56 = x54 * x55;
	double x57 = dq[1] + dq[2];
	double x58 = dq[3] + x57;
	double x59 = x55 * x58;
	double x60 = x36 * x44;
	double x61 = x41 * x9;
	double x62 = x60 + x61;
	double x63 = x39 * x62;
	double x64 = x59 - x63;
	double x65 = dq[4] * x64 + x40 + x56;
	double x66 = x39 * x58;
	double x67 = x55 * x62;
	double x68 = x66 + x67;
	double x69 = dq[4] + x47;
	double x70 = -parms[60];
	double x71 = -m_a3 * x26 - m_a3 * x27;
	double x72 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x2) * (x2));
	double x73 = x6 * x72;
	double x74 = x5 * x71 + x71 * x8 + x73;
	double x75 = dq[1] * x28 + x21 * x34 + x24 * x7 - m_d3 * x42 - m_d3 * x43;
	double x76 = m_a3 * dq[2];
	double x77 = x75 + x76;
	double x78 = x25 * x36 + x41 * x77 - m_d4 * x45 + m_d4 * x46;
	double x79 = -x39;
	double x80 = x55 * x74 - m_d5 * x66 - m_d5 * x67 + x78 * x79;
	double x81 = parms[58] * x69 + parms[61] * x80 + x68 * x70;
	double x82 = x38 * x55;
	double x83 = -dq[4];
	double x84 = x54 * x79 + x68 * x83 + x82;
	double x85 = -parms[58];
	double x86 = -m_d3 * x1;
	double x87 = dq[2] * x75 + x1 * x32 + x11 * x86 + x14 * x76 + x33 * x7 + m_d3 * x50;
	double x88 = dq[3] * x78 + x35 * x37 + x41 * x87 + m_d4 * x49 + m_d4 * x53;
	double x89 = x39 * x74 + x55 * x78 + m_d5 * x59 - m_d5 * x63;
	double x90 = -x69;
	double x91 = parms[59] * x90 + parms[60] * x64 + parms[61] * x89;
	double x92 = -x64;
	double x93 = -x47;
	double x94 = parms[46] * x93 + parms[47] * x58 + parms[48] * x78;
	double x95 = -x58;
	double x96 = -parms[45];
	double x97 = -parms[47];
	double x98 = parms[45] * x47 + parms[48] * x74 + x62 * x97;
	double x99 = parms[46] * x54 + parms[48] * x88 + parms[59] * x65 + parms[61] * x88 + x38 * x96 + x62 * x98 + x68 * x81 + x84 * x85 + x91 * x92 + x94 * x95;
	double x100 = x17 * x41;
	double x101 = dq[3] * x62 + x100 + x37 * x52;
	double x102 = ddq[4] + x101;
	double x103 = -parms[59];
	double x104 = x25 * x41 + x37 * x77 + m_d4 * x60 + m_d4 * x61;
	double x105 = parms[59] * x68 + parms[61] * x104 + x64 * x85;
	double x106 = m_d4 * x36;
	double x107 = -m_d4 * x100 + x104 * x48 + x106 * x52 + x35 * x41 + x36 * x87;
	double x108 = -m_d5 * x39;
	double x109 = -m_gx * x29 + m_gy * x30;
	double x110 = x109 + x13 * x72 + x23 * x3;
	double x111 = x110 + x44 * x76 + x50 * x71 + x51 * x71;
	double x112 = dq[4] * x80 + x107 * x55 + x108 * x54 + x111 * x39 + m_d5 * x82;
	double x113 = parms[60] * x84 + parms[61] * x112 + x102 * x103 + x105 * x64 + x81 * x90;
	double x114 = x113 * x55;
	double x115 = x107 * x79 + x111 * x55 - m_d5 * x40 - m_d5 * x56 + x83 * x89;
	double x116 = -x68;
	double x117 = parms[58] * x102 + parms[61] * x115 + x105 * x116 + x65 * x70 + x69 * x91;
	double x118 = -parms[46];
	double x119 = parms[46] * x62 + parms[48] * x104 + x58 * x96;
	double x120 = parms[47] * x38 + parms[48] * x107 + x101 * x118 + x114 + x117 * x79 + x119 * x58 + x93 * x98;
	double x121 = x120 * x41;
	double x122 = -parms[32];
	double x123 = parms[33] * x44 + parms[35] * x74 + x122 * x9;
	double x124 = -x44;
	double x125 = -x57;
	double x126 = parms[33] * x125 + parms[34] * x9 + parms[35] * x25;
	double x127 = parms[32] * x19 + parms[35] * x35 + x121 + x123 * x124 + x126 * x57 + x17 * x18 + x37 * x99;
	double x128 = -parms[19];
	double x129 = -x21;
	double x130 = dq[1] * parms[18] + parms[15] * x3 + parms[17] * x6 + parms[19] * x24 + parms[20] * x129;
	double x131 = -x3;
	double x132 = -x104;
	double x133 = x117 * x55;
	double x134 = -x78;
	double x135 = parms[41] * x62 + parms[43] * x58 + parms[44] * x47 + parms[45] * x74 + parms[46] * x134;
	double x136 = -x89;
	double x137 = parms[52] * x68 + parms[53] * x64 + parms[54] * x69 + parms[59] * x104 + x70 * x80;
	double x138 = parms[54] * x68 + parms[56] * x64 + parms[57] * x69 + parms[58] * x80 + parms[59] * x136;
	double x139 = parms[53] * x65 + parms[55] * x84 + parms[56] * x102 + parms[60] * x112 + x104 * x91 + x105 * x136 + x116 * x138 + x137 * x69 + x85 * x88;
	double x140 = parms[53] * x68 + parms[55] * x64 + parms[56] * x69 + parms[58] * x132 + parms[60] * x89;
	double x141 = parms[52] * x65 + parms[53] * x84 + parms[54] * x102 + parms[59] * x88 + x105 * x80 + x115 * x70 + x132 * x81 + x138 * x64 + x140 * x90;
	double x142 = parms[40] * x62 + parms[42] * x58 + parms[43] * x47 + parms[45] * x132 + parms[47] * x78;
	double x143 = parms[39] * x54 + parms[40] * x38 + parms[41] * x101 + parms[46] * x88 + x108 * x113 + x111 * x97 + x119 * x74 + x132 * x98 - m_d5 * x133 + x135 * x58 + x139 * x79 + x141 * x55 + x142 * x93;
	double x144 = parms[39] * x62 + parms[40] * x58 + parms[41] * x47 + parms[46] * x104 + x74 * x97;
	double x145 = parms[54] * x65 + parms[56] * x84 + parms[57] * x102 + parms[58] * x115 + x103 * x112 + x137 * x92 + x140 * x68 - x80 * x91 + x81 * x89;
	double x146 = -x74;
	double x147 = parms[41] * x54 + parms[43] * x38 + parms[44] * x101 + parms[45] * x111 + x107 * x118 + x142 * x62 + x144 * x95 + x145 + x146 * x94 + x78 * x98;
	double x148 = parms[26] * x44 + parms[27] * x9 + parms[28] * x57 + parms[33] * x74 + x18 * x77;
	double x149 = -x25;
	double x150 = parms[28] * x44 + parms[30] * x9 + parms[31] * x57 + parms[32] * x77 + parms[33] * x149;
	double x151 = x41 * x99;
	double x152 = parms[27] * x17 + parms[29] * x52 + parms[30] * x19 + parms[34] * x87 + x106 * x120 + x111 * x122 + x123 * x149 + x124 * x150 + x126 * x74 + x143 * x41 + x147 * x37 + x148 * x57 + m_d4 * x151;
	double x153 = parms[20] * x3 + parms[22] * x73 + x128 * x6;
	double x154 = parms[20] * x14 + parms[21] * x6 + parms[22] * x21;
	double x155 = parms[32] * x57 + parms[35] * x77 + x18 * x44;
	double x156 = parms[27] * x44 + parms[29] * x9 + parms[30] * x57 + parms[32] * x146 + parms[34] * x25;
	double x157 = parms[26] * x17 + parms[27] * x52 + parms[28] * x19 + parms[33] * x111 + x106 * x99 - m_d4 * x121 + x123 * x77 + x125 * x156 + x143 * x36 + x146 * x155 + x147 * x41 + x150 * x9 + x18 * x35;
	double x158 = -x62;
	double x159 = -x126;
	double x160 = parms[33] * x17 + parms[35] * x111 + parms[45] * x101 + parms[48] * x111 + x113 * x39 + x119 * x158 + x122 * x52 + x133 + x155 * x44 + x159 * x9 + x47 * x94 + x54 * x97;
	double x161 = x160 * x71;
	double x162 = -parms[21];
	double x163 = dq[1] * parms[15] + parms[13] * x3 + parms[14] * x6 + parms[20] * x73 + x162 * x24;
	double x164 = -parms[33];
	double x165 = parms[34] * x52 + parms[35] * x87 + x120 * x36 + x123 * x9 + x125 * x155 + x151 + x164 * x19;
	double x166 = x165 * x7;
	double x167 = -x6;
	double x168 = dq[1] * parms[19] + parms[22] * x24 + x162 * x3;
	double x169 = x127 * x7;
	double x170 = ddq[1] * parms[19] + dq[1] * x154 + parms[22] * x32 + x1 * x165 + x11 * x162 + x131 * x153 + x169;
	double x171 = -x73;
	double x172 = dq[1] * parms[17] + parms[14] * x3 + parms[16] * x6 + parms[19] * x171 + parms[21] * x21;
	double x173 = -parms[20];
	double x174 = parms[40] * x54 + parms[42] * x38 + parms[43] * x101 + parms[47] * x107 + x104 * x94 + x108 * x117 + m_d5 * x114 + x119 * x134 + x135 * x158 + x139 * x55 + x141 * x39 + x144 * x47 + x88 * x96;
	double x175 = parms[28] * x17 + parms[30] * x52 + parms[31] * x19 + parms[32] * x35 - x148 * x9 + x155 * x25 + x156 * x44 + x159 * x77 + x164 * x87 + x174;
	//
	tau[0] = ddq[0] * parms[10] + ddq[0] * parms[3] + dq[0] * parms[11] + parms[12] * sign(dq[0]) - parms[6] * x109 + parms[8] * x31 + x0 * x72 * (parms[20] * x11 + parms[22] * x110 + x128 * x15 + x154 * x167 + x160 + x168 * x3) + m_d2 * x0 * (ddq[1] * x173 + parms[21] * x15 + parms[22] * x33 + x127 * x34 + x14 * x168 + x153 * x6 + x166) + x0 * (ddq[1] * parms[17] + dq[1] * x163 + parms[14] * x11 + parms[16] * x15 + parms[21] * x33 + x1 * x157 + x110 * x128 + x127 * x86 + x129 * x153 + x130 * x131 + x152 * x7 + x154 * x73 + x161 * x7 + m_d3 * x166) - m_d2 * x170 * x2 + x2 * (ddq[1] * parms[15] + parms[13] * x11 + parms[14] * x15
		+ parms[20] * x110 + x130 * x6 + x14 * x172 + x152 * x34 + x153 * x24 + x157 * x7 + x161 * x34 + x162 * x32 + x165 * x86 + x168 * x171 - m_d3 * x169);
	tau[1] = ddq[1] * parms[18] + ddq[1] * parms[23] + dq[1] * parms[24] + parms[15] * x11 + parms[17] * x15 + parms[19] * x32 + parms[25] * sign(dq[1]) + x127 * x28 - x154 * x24 + x163 * x167 + x168 * x21
		+ m_a2 * x170 + x172 * x3 + x173 * x33 + x175;
	tau[2] = ddq[2] * parms[36] + dq[2] * parms[37] + parms[38] * sign(dq[2]) + m_a3 * x127 + x175;
	tau[3] = ddq[3] * parms[49] + dq[3] * parms[50] + parms[51] * sign(dq[3]) + x174;
	tau[4] = ddq[4] * parms[62] + dq[4] * parms[63] + parms[64] * sign(dq[4]) + x145;
	return true;
}


// exclude gravity, friction, motor inertia;
void palletDynamics::calculateMomentumEstimatedJointTorques
(
	const EcRealVector& q,
	const EcRealVector& dq,
	const EcRealVector& ddq,
	const EcRealVector& parms,
	EcRealVector& tau
)
{
	double x0 = cos(q[1]);
	double x1 = cos(q[2]);
	double x2 = dq[1] + dq[2];
	double x3 = m_a3 * dq[2];
	double x4 = m_a2 * dq[1];
	double x5 = sin(q[1]);
	double x6 = dq[0] * x5;
	double x7 = -m_d2 * x6;
	double x8 = x4 + x7;
	double x9 = dq[0] * x0;
	double x10 = sin(q[2]);
	double x11 = x10 * x9;
	double x12 = ((x10) * (x10));
	double x13 = ((x1) * (x1));
	double x14 = m_a3 * x12 + m_a3 * x13;
	double x15 = m_d2 * x9;
	double x16 = -x10;
	double x17 = x1 * x6;
	double x18 = dq[1] * x14 + x1 * x8 - m_d3 * x11 + x15 * x16 - m_d3 * x17;
	double x19 = x18 + x3;
	double x20 = x11 + x17;
	double x21 = -parms[28];
	double x22 = parms[26] * x2 + parms[29] * x19 + x20 * x21;
	double x23 = -x2;
	double x24 = cos(q[3]);
	double x25 = dq[3] + x2;
	double x26 = -parms[36];
	double x27 = sin(q[3]);
	double x28 = x10 * x6;
	double x29 = -x28;
	double x30 = x1 * x9;
	double x31 = x29 + x30;
	double x32 = x27 * x31;
	double x33 = x20 * x24;
	double x34 = x32 + x33;
	double x35 = x1 * x15 + x10 * x8 - m_d3 * x28 + m_d3 * x30;
	double x36 = -x24;
	double x37 = x19 * x36 + x27 * x35 + m_d4 * x32 + m_d4 * x33;
	double x38 = parms[37] * x34 + parms[39] * x37 + x25 * x26;
	double x39 = sin(q[4]);
	double x40 = -x39;
	double x41 = cos(q[4]);
	double x42 = x25 * x41;
	double x43 = x34 * x39;
	double x44 = x42 - x43;
	double x45 = x20 * x27;
	double x46 = x24 * x31;
	double x47 = x45 - x46;
	double x48 = -dq[3];
	double x49 = -dq[2];
	double x50 = ddq[0] * x5;
	double x51 = dq[1] * x9 + x50;
	double x52 = x16 * x51;
	double x53 = -dq[1];
	double x54 = ddq[0] * x0;
	double x55 = x53 * x6 + x54;
	double x56 = x1 * x55;
	double x57 = x20 * x49 + x52 + x56;
	double x58 = x27 * x57;
	double x59 = x10 * x55;
	double x60 = x1 * x51;
	double x61 = dq[2] * x31 + x59 + x60;
	double x62 = x24 * x61;
	double x63 = x47 * x48 + x58 + x62;
	double x64 = x41 * x63;
	double x65 = ddq[1] + ddq[2];
	double x66 = ddq[3] + x65;
	double x67 = x39 * x66;
	double x68 = dq[4] * x44 + x64 + x67;
	double x69 = -parms[48];
	double x70 = -m_a3 * x12 - m_a3 * x13;
	double x71 = -m_a2 * ((x0) * (x0)) - m_a2 * ((x5) * (x5));
	double x72 = x4 * x6 + x54 * x71;
	double x73 = x20 * x3 + x52 * x70 + x56 * x70 + x72;
	double x74 = m_a2 * ddq[1] + x15 * x53 - m_d2 * x50;
	double x75 = dq[1] * x7 + m_d2 * x54;
	double x76 = ddq[1] * x14 + m_a3 * ddq[2] + x1 * x74 + x16 * x75 + x35 * x49 - m_d3 * x59 - m_d3 * x60;
	double x77 = x27 * x61;
	double x78 = m_d4 * x24;
	double x79 = -m_d3 * x10;
	double x80 = dq[2] * x18 + x1 * x75 + x10 * x74 + x3 * x53 + x51 * x79 + m_d3 * x56;
	double x81 = x24 * x80 + x27 * x76 + x37 * x48 + x57 * x78 - m_d4 * x77;
	double x82 = x71 * x9;
	double x83 = x29 * x70 + x30 * x70 + x82;
	double x84 = x19 * x27 + x24 * x35 - m_d4 * x45 + m_d4 * x46;
	double x85 = x39 * x83 + x41 * x84 + m_d5 * x42 - m_d5 * x43;
	double x86 = -dq[4];
	double x87 = x40 * x81 + x41 * x73 - m_d5 * x64 - m_d5 * x67 + x85 * x86;
	double x88 = x34 * x41;
	double x89 = x25 * x39;
	double x90 = x88 + x89;
	double x91 = -parms[46];
	double x92 = parms[47] * x90 + parms[49] * x37 + x44 * x91;
	double x93 = -x90;
	double x94 = dq[3] * x34 + x36 * x57 + x77;
	double x95 = ddq[4] + x94;
	double x96 = dq[4] + x47;
	double x97 = -x96;
	double x98 = parms[47] * x97 + parms[48] * x44 + parms[49] * x85;
	double x99 = parms[46] * x95 + parms[49] * x87 + x68 * x69 + x92 * x93 + x96 * x98;
	double x100 = -parms[37];
	double x101 = -parms[47];
	double x102 = x41 * x66;
	double x103 = x102 + x40 * x63 + x86 * x90;
	double x104 = x40 * x84 + x41 * x83 - m_d5 * x88 - m_d5 * x89;
	double x105 = parms[46] * x96 + parms[49] * x104 + x69 * x90;
	double x106 = -m_d5 * x39;
	double x107 = dq[4] * x104 + m_d5 * x102 + x106 * x63 + x39 * x73 + x41 * x81;
	double x108 = parms[48] * x103 + parms[49] * x107 + x101 * x95 + x105 * x97 + x44 * x92;
	double x109 = x108 * x41;
	double x110 = -parms[38];
	double x111 = parms[36] * x47 + parms[39] * x83 + x110 * x34;
	double x112 = -x47;
	double x113 = parms[38] * x66 + parms[39] * x81 + x100 * x94 + x109 + x111 * x112 + x25 * x38 + x40 * x99;
	double x114 = -parms[26];
	double x115 = parms[27] * x20 + parms[29] * x83 + x114 * x31;
	double x116 = -parms[27];
	double x117 = -x98;
	double x118 = dq[3] * x84 + x27 * x80 + x36 * x76 + m_d4 * x58 + m_d4 * x62;
	double x119 = parms[37] * x112 + parms[38] * x25 + parms[39] * x84;
	double x120 = -x119;
	double x121 = parms[37] * x63 + parms[39] * x118 + parms[47] * x68 + parms[49] * x118 + x103 * x91 + x105 * x90 + x111 * x34 + x117 * x44 + x120 * x25 + x26 * x66;
	double x122 = x121 * x27;
	double x123 = parms[28] * x57 + parms[29] * x80 + x113 * x24 + x115 * x31 + x116 * x65 + x122 + x22 * x23;
	double x124 = x1 * x123;
	double x125 = -x34;
	double x126 = x41 * x99;
	double x127 = parms[27] * x23 + parms[28] * x31 + parms[29] * x35;
	double x128 = -x127;
	double x129 = parms[27] * x61 + parms[29] * x73 + parms[36] * x94 + parms[39] * x73 + x108 * x39 + x110 * x63 + x114 * x57 + x119 * x47 + x125 * x38 + x126 + x128 * x31 + x20 * x22;
	double x130 = x129 * x70;
	double x131 = parms[17] * x53 + parms[18] * x9 + parms[19] * x15;
	double x132 = -parms[18];
	double x133 = dq[1] * parms[12] + parms[10] * x6 + parms[11] * x9 + parms[17] * x82 + x132 * x8;
	double x134 = -x37;
	double x135 = -x85;
	double x136 = parms[40] * x90 + parms[41] * x44 + parms[42] * x96 + parms[47] * x37 + x104 * x69;
	double x137 = parms[42] * x90 + parms[44] * x44 + parms[45] * x96 + parms[46] * x104 + parms[47] * x135;
	double x138 = parms[41] * x68 + parms[43] * x103 + parms[44] * x95 + parms[48] * x107 + x118 * x91 + x135 * x92 + x136 * x96 + x137 * x93 + x37 * x98;
	double x139 = parms[31] * x34 + parms[33] * x25 + parms[34] * x47 + parms[36] * x134 + parms[38] * x84;
	double x140 = parms[41] * x90 + parms[43] * x44 + parms[44] * x96 + parms[48] * x85 + x37 * x91;
	double x141 = parms[40] * x68 + parms[41] * x103 + parms[42] * x95 + parms[47] * x118 + x104 * x92 + x105 * x134 + x137 * x44 + x140 * x97 + x69 * x87;
	double x142 = -x84;
	double x143 = parms[32] * x34 + parms[34] * x25 + parms[35] * x47 + parms[36] * x83 + parms[37] * x142;
	double x144 = parms[30] * x63 + parms[31] * x66 + parms[32] * x94 + parms[37] * x118 + x106 * x108 + x110 * x73 + x111 * x134 + x112 * x139 - m_d5 * x126 + x138 * x40 + x141 * x41 + x143 * x25 + x38 * x83;
	double x145 = -x35;
	double x146 = parms[30] * x34 + parms[31] * x25 + parms[32] * x47 + parms[37] * x37 + x110 * x83;
	double x147 = parms[42] * x68 + parms[44] * x103 + parms[45] * x95 + parms[46] * x87 + x101 * x107 + x104 * x117 + x105 * x85 - x136 * x44 + x140 * x90;
	double x148 = parms[32] * x63 + parms[34] * x66 + parms[35] * x94 + parms[36] * x73 + x100 * x81 + x111 * x84 + x120 * x83 + x139 * x34 - x146 * x25 + x147;
	double x149 = parms[22] * x20 + parms[24] * x31 + parms[25] * x2 + parms[26] * x19 + parms[27] * x145;
	double x150 = -x20;
	double x151 = parms[20] * x20 + parms[21] * x31 + parms[22] * x2 + parms[27] * x83 + x19 * x21;
	double x152 = parms[21] * x61 + parms[23] * x57 + parms[24] * x65 + parms[28] * x80 + x113 * x78 + x114 * x73 + x115 * x145 + m_d4 * x122 + x127 * x83 + x144 * x27 + x148 * x36 + x149 * x150 + x151 * x2;
	double x153 = -parms[16];
	double x154 = parms[17] * x6 + parms[19] * x82 + x153 * x9;
	double x155 = -x15;
	double x156 = x113 * x27;
	double x157 = parms[26] * x65 + parms[29] * x76 + x115 * x150 + x121 * x36 + x127 * x2 + x156 + x21 * x61;
	double x158 = parms[21] * x20 + parms[23] * x31 + parms[24] * x2 + parms[28] * x35 + x114 * x83;
	double x159 = parms[20] * x61 + parms[21] * x57 + parms[22] * x65 + parms[27] * x73 + x115 * x19 + x121 * x78 + x144 * x24 + x148 * x27 + x149 * x31 - m_d4 * x156 + x158 * x23 + x21 * x76 - x22 * x83;
	double x160 = dq[1] * parms[15] + parms[12] * x6 + parms[14] * x9 + parms[16] * x8 + parms[17] * x155;
	double x161 = -x6;
	double x162 = x1 * x157;
	double x163 = ddq[1] * parms[16] + dq[1] * x131 + parms[19] * x74 + x10 * x123 + x132 * x51 + x154 * x161 + x162;
	double x164 = -x9;
	double x165 = dq[1] * parms[16] + parms[19] * x8 + x132 * x6;
	double x166 = -parms[17];
	double x167 = dq[1] * parms[14] + parms[11] * x6 + parms[13] * x9 + parms[18] * x15 + x153 * x82;
	double x168 = parms[31] * x63 + parms[33] * x66 + parms[34] * x94 + parms[38] * x81 + x106 * x99 + m_d5 * x109 + x118 * x26 + x119 * x37 + x125 * x143 + x138 * x41 + x141 * x39 + x142 * x38 + x146 * x47;
	double x169 = parms[22] * x61 + parms[24] * x57 + parms[25] * x65 + parms[26] * x76 + x116 * x80 + x128 * x19 - x151 * x31 + x158 * x20 + x168 + x22 * x35;
	//
	tau[0] = ddq[0] * parms[3] + x0 * x71 * (parms[17] * x51 + parms[19] * x72 + x129 + x131 * x164 + x153 * x55 + x165 * x6) + m_d2 * x0 * (ddq[1] * x166 + parms[18] * x55 + parms[19] * x75 + x124 + x154 * x9 + x157 * x16 + x165 * x53) + x0 * (ddq[1] * parms[14] + dq[1] * x133 + parms[11] * x51 + parms[13] * x55 + parms[18] * x75 + x1 * x130 + x1 * x152 + x10 * x159 + m_d3 * x124 + x131 * x82 + x153 * x72 + x154 * x155 +
		x157 * x79 + x160 * x161) - m_d2 * x163 * x5 + x5 * (ddq[1] * parms[12] + parms[10] * x51 + parms[11] * x55 + parms[17] * x72 + x1 * x159 + x123 * x79 + x130 * x16 + x132 * x74 + x152 * x16 + x154 * x8 + x160 * x9 - m_d3 * x162 - x165 * x82 + x167 * x53);
	tau[1] = ddq[1] * parms[15] + parms[12] * x51 + parms[14] * x55 + parms[16] * x74 - x131 * x8 + x133 * x164 + x14 * x157 + x15 * x165 + m_a2 * x163 + x166 * x75 + x167 * x6 + x169;
	tau[2] = m_a3 * x157 + x169;
	tau[3] = x168;
	tau[4] = x147;
	return;
}


void palletDynamics::calculateJointFricition(const EcRealVector& dq, const EcRealVector& parms, EcReal coeffColomb, EcReal coeffViscous, EcRealVector& tau)
{
	tau[0] = coeffViscous * dq[0] * parms[11] + coeffColomb * parms[12] * sign(dq[0]);
	tau[1] = coeffViscous * dq[1] * parms[24] + coeffColomb * parms[25] * sign(dq[1]);
	tau[2] = coeffViscous * dq[2] * parms[37] + coeffColomb * parms[38] * sign(dq[2]);
	tau[3] = coeffViscous * dq[3] * parms[50] + coeffColomb * parms[51] * sign(dq[3]);
	tau[4] = coeffViscous * dq[4] * parms[63] + coeffColomb * parms[64] * sign(dq[4]);
	//tau[5] = coeffViscous * dq[5] * parms[76] + coeffColomb * parms[77] * sign(dq[5]);
}