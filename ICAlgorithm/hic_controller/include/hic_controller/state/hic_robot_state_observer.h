#pragma once

#include "hic_controller/interface/hic_c_api.h"

namespace hic
{

/// @brief 机器人自身状态观测器。
/// 该类只负责机器人本体的关节位置、速度、加速度、电流和测量力矩状态管理，
/// 并提供基础滤波、有限值检查、范围检查以及电流到电机估计力矩的换算。
/// 注意：
/// 1. 该类不负责末端六维外力解释；
/// 2. 该类不负责扭矩传感器硬件通道映射；
/// 3. 该类输出的 motorEstimatedTorque 是基于电机参数的估计值，不等同于外界扰动力矩。
class HicRobotStateObserver
{
public:
	HicRobotStateObserver();
	~HicRobotStateObserver();

	/// @brief 初始化状态观测器。
	/// @param config 状态观测与滤波配置。
	/// @return HIC_STATUS_OK 表示初始化成功。
	HicStatus initialize(const HicRobotStateObserverConfig& config);

	/// @brief 在线更新状态观测器配置。
	/// @param config 新配置。
	/// @return 配置合法则返回 HIC_STATUS_OK。
	HicStatus setConfig(const HicRobotStateObserverConfig& config);

	/// @brief 读取当前配置。
	/// @param configOut 输出配置结构体。
	HicStatus getConfig(HicRobotStateObserverConfig& configOut) const;

	/// @brief 更新关节位置、速度、加速度。
	/// @param jointPosition 按关节顺序排列的位置数组。
	/// @param jointVelocity 按关节顺序排列的速度数组。
	/// @param jointAcceleration 按关节顺序排列的加速度数组。
	HicStatus updateJointState(
		const double* jointPosition,
		const double* jointVelocity,
		const double* jointAcceleration);

	/// @brief 更新电机电流测量值。
	/// @param motorCurrent 按关节顺序排列的电流数组。
	HicStatus updateMotorCurrent(const double* motorCurrent);

	/// @brief 更新关节测量力矩。
	/// @param jointMeasuredTorque 按关节顺序排列的关节测量力矩。
	HicStatus updateJointMeasuredTorque(const double* jointMeasuredTorque);

	/// @brief 便捷接口：一次性更新原始位置、电流和时间戳。
	/// @param jointPosition 当前关节位置。
	/// @param motorCurrent 当前电机电流。
	/// @param currentTime 当前控制周期时间戳，单位 s。
	/// @note 该接口会基于“当前位置 + 上一周期缓存 + 当前时间戳”内部差分估计：
	/// 1. 关节速度；
	/// 2. 关节加速度；
	/// 3. 基于电流参数的电机估计力矩。
	HicStatus updateRobotState(
		const double* jointPosition,
		const double* motorCurrent,
		double currentTime);

	/// @brief 输出标准化机器人状态。
	/// @param stateOut 输出滤波后的关节状态与电机估计力矩。
	HicStatus getRobotState(HicRobotState& stateOut) const;

	HicStatus getFilteredJointPosition(double* jointPosition) const;
	HicStatus getFilteredJointVelocity(double* jointVelocity) const;
	HicStatus getFilteredJointAcceleration(double* jointAcceleration) const;
	HicStatus getFilteredMotorCurrent(double* motorCurrent) const;
	HicStatus getFilteredJointMeasuredTorque(double* jointMeasuredTorque) const;
	HicStatus getMotorEstimatedTorque(double* motorEstimatedTorque) const;

	/// @brief 返回当前状态是否有效。
	/// 有效性由有限值检查和范围检查共同决定。
	bool isStateValid() const;

	/// @brief 清零内部原始值、滤波值和估计值。
	void reset();

private:
	HicStatus validateConfig(const HicRobotStateObserverConfig& config) const;
	HicStatus validateArrayFinite(const double* data, int length) const;
	void applyFirstOrderFilter(
		const double* input,
		double* output,
		const double* alpha,
		int length,
		bool enabled);
	HicStatus estimateMotorTorque();
	HicStatus validateStateRange() const;

private:
	bool initialized_;
	bool stateValid_;

	HicRobotStateObserverConfig config_;

	double rawJointPosition_[HIC_MAX_JOINTS];
	double rawJointVelocity_[HIC_MAX_JOINTS];
	double rawJointAcceleration_[HIC_MAX_JOINTS];
	double rawMotorCurrent_[HIC_MAX_JOINTS];
	double rawJointMeasuredTorque_[HIC_MAX_JOINTS];

	double filteredJointPosition_[HIC_MAX_JOINTS];
	double filteredJointVelocity_[HIC_MAX_JOINTS];
	double filteredJointAcceleration_[HIC_MAX_JOINTS];
	double filteredMotorCurrent_[HIC_MAX_JOINTS];
	double filteredJointMeasuredTorque_[HIC_MAX_JOINTS];

	double motorEstimatedTorque_[HIC_MAX_JOINTS];
	double previousJointPosition_[HIC_MAX_JOINTS];
	double previousJointVelocity_[HIC_MAX_JOINTS];
	double previousTimestamp_;
	bool hasPreviousState_;
};

} // namespace hic
