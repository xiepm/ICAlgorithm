#pragma once

#include "hic_controller/types/hic_config.h"
#include "hic_controller/types/hic_types.h"

namespace hic
{

/// @brief 笛卡尔阻抗控制核心。
/// 仅负责阻抗数学和目标管理，不负责模式切换、安全保护和力矩转电流。
class HicCartesianImpedanceCore
{
public:
	HicCartesianImpedanceCore();
	~HicCartesianImpedanceCore();

	HicStatus initialize(int jointCount, double controlPeriod);
	HicStatus setGains(const HicImpedanceGains& gains);
	HicStatus setForceControlMode(HicForceControlMode forceControlMode);
	HicStatus setNullspaceConfig(const HicNullspaceControlConfig& config);
	HicStatus captureCurrentJointPositionAsNullspaceTarget(const double* jointPosition);
	HicStatus setFixedTargetPosition(const double targetPosition[3]);
	HicStatus setFixedTargetPose(const double targetPose[HIC_POSE_DIM]);
	HicStatus setOnlineTargetPose(
		const double targetPose[HIC_POSE_DIM],
		const double targetVelocity[HIC_CARTESIAN_DIM]);

	HicStatus computeJointTorque(
		const double currentPose[HIC_POSE_DIM],
		const double currentTwist[HIC_CARTESIAN_DIM],
		const double* jointPosition,
		const double* jointVelocity,
		const double* jacobianRowMajor,
		double* jointTorque);

	/// @brief 获取最近一次阻抗计算得到的笛卡尔 wrench 指令。
	/// 便于调试固定点与轨迹模式下的阻抗输出。
	HicStatus getLastCartesianWrenchCommand(double wrenchCommand[HIC_CARTESIAN_DIM]) const;
	/// @brief 获取最近一次阻抗计算中的笛卡尔误差。
	/// 误差在 base frame 下表达，姿态误差当前使用集中封装的近似实现。
	HicStatus getLastCartesianError(double cartesianError[HIC_CARTESIAN_DIM]) const;

	void reset();

private:
	HicStatus computeCartesianError(
		const double currentPose[HIC_POSE_DIM],
		double cartesianError[HIC_CARTESIAN_DIM]) const;

	HicStatus computeCartesianWrench(
		const double cartesianError[HIC_CARTESIAN_DIM],
		const double currentTwist[HIC_CARTESIAN_DIM],
		double wrenchCommand[HIC_CARTESIAN_DIM]) const;
	HicStatus applyNullspaceTorque(
		const double* jointPosition,
		const double* jointVelocity,
		const double* jacobianRowMajor,
		double* jointTorque) const;
	int getTaskDimension() const;

private:
	int jointCount_;
	double controlPeriod_;
	bool initialized_;
	HicForceControlMode forceControlMode_;
	HicImpedanceGains gains_;
	HicNullspaceControlConfig nullspaceConfig_;
	double targetPose_[HIC_POSE_DIM];
	double targetVelocity_[HIC_CARTESIAN_DIM];
	double lastCartesianWrenchCommand_[HIC_CARTESIAN_DIM];
	double lastCartesianError_[HIC_CARTESIAN_DIM];
};

} // namespace hic
