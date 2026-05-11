#pragma once

#include "hic_controller/types/hic_types.h"

namespace hic
{

/// @brief 运动学适配器。
/// @note 该类只负责 FK、Jacobian 和末端 twist，不承担动力学补偿和电流换算。
/// @note 当前对外 include 层仅保留 base 与 adapter；
/// 具体机型类保留在 src/kinematics 中，由 adapter 在内部按 robotType 分发。
class HicKinematicsAdapter
{
public:
	HicKinematicsAdapter();
	~HicKinematicsAdapter();

	HicStatus initialize(
		int robotType,
		int jointCount,
		const double* kinematicParams);

	HicStatus computeForwardKinematics(
		const double* jointPosition,
		double pose[HIC_POSE_DIM]);

	HicStatus computeJacobian(
		const double* jointPosition,
		double* jacobianRowMajor);

	HicStatus computeEndEffectorTwist(
		const double* jointPosition,
		const double* jointVelocity,
		double twist[HIC_CARTESIAN_DIM]);

	void reset();

private:
	int robotType_;
	int jointCount_;
	bool initialized_;
	bool useMockModel_;
	void* backend_;
};

} // namespace hic
