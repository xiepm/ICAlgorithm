#pragma once

#include "hic_controller/interface/hic_c_api.h"

namespace hic
{

/// @brief 关节空间阻抗控制核心。
/// @note 该类只负责关节空间阻抗数学，不负责状态读取、动力学补偿、模式切换或安全限幅。
/// @note 完整链路中，coordinator 会先从 observer 取 q/dq/tau_ext，再调用本类计算 tau_imp。
class HicJointImpedanceCore
{
public:
	HicJointImpedanceCore();
	~HicJointImpedanceCore();

	/// @brief 初始化核心参数。
	/// @param jointCount 有效关节数量。
	/// @param controlPeriod 控制周期，单位 s；当前 PD 控制律不直接使用，但保留供后续扩展。
	HicStatus initialize(int jointCount, double controlPeriod);

	/// @brief 设置关节阻抗参数。
	/// @note config.targetPosition 在进入本类前已转换为内部 rad。
	HicStatus setConfig(const HicJointImpedanceConfig& config);

	/// @brief 抓取当前关节位置作为阻抗平衡点。
	/// @param jointPosition 当前关节位置，单位 rad。
	HicStatus captureTargetPosition(const double* jointPosition);

	/// @brief 计算纯关节阻抗力矩，不含动力学补偿和安全限幅。
	/// @param jointPosition 当前关节位置，单位 rad。
	/// @param jointVelocity 当前关节速度，单位 rad/s。
	/// @param externalTorque 滤波后的外力矩估计，单位 N.m；传 nullptr 时视为全零。
	/// @param jointTorque 输出关节阻抗力矩，单位 N.m。
	/// @note tau = Kd * (q_d - q) + Dd * (dq_d - dq) - tau_ext_hat。
	HicStatus computeJointTorque(
		const double* jointPosition,
		const double* jointVelocity,
		const double* externalTorque,
		double* jointTorque);

	/// @brief 读取最近一次计算时的位置误差 q_d - q。
	HicStatus getLastPositionError(double* positionError) const;

	/// @brief 读取最近一次计算时的速度误差 dq_d - dq。
	HicStatus getLastVelocityError(double* velocityError) const;

	/// @brief 清空配置和最近误差缓存。
	void reset();

private:
	int jointCount_; ///< 有效关节数量。
	double controlPeriod_; ///< 控制周期，单位 s。
	bool initialized_; ///< 是否已完成 initialize()。
	HicJointImpedanceConfig config_; ///< 当前关节阻抗配置。
	double lastPositionError_[HIC_MAX_JOINTS]; ///< 最近一次计算的位置误差，单位 rad。
	double lastVelocityError_[HIC_MAX_JOINTS]; ///< 最近一次计算的速度误差，单位 rad/s。
};

} // namespace hic
