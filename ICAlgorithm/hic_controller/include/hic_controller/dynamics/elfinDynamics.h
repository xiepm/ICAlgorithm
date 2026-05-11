#pragma once
#include "hic_common_types.h"
#include "hic_controller/dynamics/hic_dynamics_base.h"


using namespace KDL;

class elfinDynamics : public dynamicsBase
{
public:
	elfinDynamics();
	virtual ~elfinDynamics();

	virtual void setRobotDHParameters
	(
		const EcRealVector& kinematcisParam
	);

	virtual void setGravityVector
	(
		const EcReal gx, const EcReal gy, const EcReal gz
	);


	virtual void calculateGravityJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& parms,
		EcRealVector& tau
	);


	// calculate forward dynamics of elfin
	virtual EcBoolean calculateEstimateJointToqrues
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);


	// exclude gravity, friction, motor inertia;
	virtual void calculateMomentumEstimatedJointTorques
	(
		const EcRealVector& q,
		const EcRealVector& dq,
		const EcRealVector& ddq,
		const EcRealVector& parms,
		EcRealVector& tau
	);

	virtual void calculateJointFricition(const EcRealVector& dq, const EcRealVector& parms, EcReal coeffColomb, EcReal coeffViscous, EcRealVector& tau);


private:
	EcSizeT									m_NumJoints;
	EcReal									m_d1, m_d4, m_d6, m_a2; 		//Link length of Elfin

	EcReal									m_gx;							//Gravity component
	EcReal									m_gy;
	EcReal									m_gz;
	KDL::Vector								m_gravity;						//Gravity vector

	EcRealVector							m_friTemperaturesParams;
	EcRealVector							m_jointTemperatures;

	EcRealVector							m_zeros;

};
