#pragma once

#include "hic_controller/interface/hic_c_api.h"

namespace hic
{

/// @brief 交互力观测器�?
/// 用于统一管理关节外力矩、末端六维外力和扭矩传感器原始输入，
/// 并完成标定、滤波、故障状态检查和标准化输出�?
/// 注意�?
/// 1. calibratedJointTorqueSensor 表示按配置修正后的关节扭矩传感器输出�?
/// 2. jointExternalTorque_current 表示外界扰动力矩估计或外部直接给定值；
/// 3. 两者物理含义不同，不能默认混用�?
class HicInteractionForceObserver
{
public:
	HicInteractionForceObserver();
	~HicInteractionForceObserver();

	HicStatus initialize(int jointCount, double controlPeriod);
	HicStatus setTorqueSensorConfig(const HicTorqueSensorConfig& config);
	HicStatus getTorqueSensorConfig(HicTorqueSensorConfig& configOut) const;
	HicStatus loadTorqueSensorConfigFromFile(const char* filePath);
	HicStatus updateRawJointTorqueSensor(const double* rawTorqueByHardwareChannel);

	/// @brief 更新关节外力矩估计，单位 N.m�?	/// @note 该值可以来自外部直接输入，也可以来�?coordinator 的电流反推链路：
	/// motorEstimatedTorque_current - dynamicsModelTorque�?
	HicStatus updateJointExternalTorque_current(const double* jointExternalTorque_current);
	HicStatus updateJointExternalTorque_sensor(const double* jointExternalTorque_sensor);
	HicStatus getCalibratedJointTorqueSensor(double* calibratedJointTorque) const;
	HicStatus getFilteredJointTorqueSensor(double* filteredJointTorque) const;

	/// @brief 读取滤波后的关节外力矩估计，单位 N.m�?	/// @note 关节阻抗模式启用 enableExternalTorqueCompensation 时读取该结果�?
	HicStatus getFilteredJointExternalTorque_current(double* jointExternalTorque_current) const;
	HicStatus getFilteredJointExternalTorque_sensor(double* jointExternalTorque_sensor) const;
	HicStatus getTorqueSensorFaultStatus(bool* faultStatus) const;

	void reset();

private:
	HicStatus validateTorqueSensorConfig(const HicTorqueSensorConfig& config) const;
	HicStatus calibrateJointTorqueSensor(const double* rawTorqueByHardwareChannel);
	void applyTorqueSensorFilter();
	void applyExternalTorqueFilter_current();
	void applyExternalTorqueFilter_sensor();

private:
	int jointCount_;
	double controlPeriod_;
	bool initialized_;
	HicTorqueSensorConfig torqueSensorConfig_;
	double rawTorqueByHardwareChannel_[HIC_MAX_JOINTS];
	double calibratedJointTorqueSensor_[HIC_MAX_JOINTS];
	double filteredJointTorqueSensor_[HIC_MAX_JOINTS];
	double jointExternalTorque_current_[HIC_MAX_JOINTS];
	double filteredJointExternalTorque_current_[HIC_MAX_JOINTS];
	double jointExternalTorque_sensor_[HIC_MAX_JOINTS];
	double filteredJointExternalTorque_sensor_[HIC_MAX_JOINTS];
	bool torqueSensorFaultStatus_[HIC_MAX_JOINTS];
};

} // namespace hic
