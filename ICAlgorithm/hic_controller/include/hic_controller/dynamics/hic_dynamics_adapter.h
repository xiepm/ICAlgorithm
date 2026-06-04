#pragma once

#include "hic_controller/interface/hic_c_api.h"

namespace hic
{

/// @brief 动力学适配器。
/// @note 该适配器负责在统一接口与具体机型动力学实现之间做薄封装。
/// @note 后端选择由 robotType 决定，当前已覆盖 elfin、ur、pallet、ds 和 7 自由度模型。
/// @note 除动力学线性参数外，也统一承接 DH/几何参数、重力向量和 payload 等运行时设置入口。
/// @note 当前对外 include 层仅保留 base 与 adapter；
/// 具体机型类保留在 src/dynamics 中，由 adapter 在内部按 robotType 分发。
class HicDynamicsAdapter
{
public:
	HicDynamicsAdapter();
	~HicDynamicsAdapter();

	/// @brief 初始化动力学后端。
	/// @param robotType 机型编号，用于选择不同动力学 backend。
	/// @param jointCount 当前机器人自由度。
	/// @param dynamicParams 运行时动力学参数数组。
	HicStatus initialize(
		int robotType,
		int jointCount,
		const double* dynamicParams);

	/// @brief 在线更新动力学参数。
	HicStatus setDynamicParameters(const double* dynamicParams);

	/// @brief 在线更新基于电流力矩整定得到的动力学参数。
	HicStatus setDynamicParameters_current(const double* dynamicParams);

	/// @brief 在线更新基于扭矩传感器整定得到的动力学参数。
	HicStatus setDynamicParameters_sensor(const double* dynamicParams);

	/// @brief 在线更新机器人几何/DH 参数。
	HicStatus setRobotKinematicParameters(const double* kinematicParams);

	/// @brief 设置重力向量。
	HicStatus setGravityVector(
		double gx,
		double gy,
		double gz);

	/// @brief 设置末端 payload 质量与质心。
	HicStatus setPayloadMassProperties(
		double mass,
		const double centerOfMass[3]);

	HicStatus computeGravityTorque(
		const double* jointPosition,
		double* gravityTorque);

	HicStatus computeCoriolisTorque(
		const double* jointPosition,
		const double* jointVelocity,
		double* coriolisTorque);

	HicStatus computeMassMatrix(
		const double* jointPosition,
		double* massMatrixRowMajor);

	HicStatus computeFrictionTorque(
		const double* jointPosition,
		const double* jointVelocity,
		const double* jointAcceleration,
		double* frictionTorque);

	HicStatus computeModelTorque_current(
		const double* jointPosition,
		const double* jointVelocity,
		const double* jointAcceleration,
		double* modelTorque);

	HicStatus computeModelTorque_sensor(
		const double* jointPosition,
		const double* jointVelocity,
		const double* jointAcceleration,
		double* modelTorque);

	void reset();

private:
	HicStatus setDynamicParametersTo(double* destination, const double* dynamicParams);
	HicStatus computeModelTorqueWithParameters(
		const double* dynamicParams,
		const double* jointPosition,
		const double* jointVelocity,
		const double* jointAcceleration,
		double* modelTorque);

	int robotType_;
	int jointCount_;
	int modelJointCount_;
	bool initialized_;
	void* backend_;
	double dynamicParams_current_[HIC_MAX_DYNAMIC_PARAMS];
	double dynamicParams_sensor_[HIC_MAX_DYNAMIC_PARAMS];
	double payloadMass_;
	double payloadCenterOfMass_[3];
};

} // namespace hic
