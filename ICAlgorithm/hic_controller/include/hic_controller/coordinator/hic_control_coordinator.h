#pragma once

#include <cstdint>
#include <memory>

#include "hic_controller/interface/hic_c_api.h"
#include "hic_controller/control/hic_cartesian_impedance_core.h"
#include "hic_controller/control/hic_joint_impedance_core.h"
#include "hic_controller/dynamics/hic_dynamics_adapter.h"
#include "hic_controller/kinematics/hic_kinematics_adapter.h"
#include "hic_controller/sensing/hic_interaction_force_observer.h"
#include "hic_controller/state/hic_robot_state_observer.h"

namespace hic
{

/// @brief HIC 库内部总协调器。
/// 说明：
/// 1. 该类用于承接算法模块之间的内部协作，不作为正式外部接口；
/// 2. 外部集成方应统一通过 hic_controller/interface/hic_c_api.h 访问库能力；
/// 3. 当前版本重点覆盖“力控总模式”下的零力示教与阻抗子模式。
class HicControlCoordinator
{
public:
	// ============================================================
	// 1. 生命周期与基础初始化
	// ============================================================

	/// @brief 创建并初始化协调器实例。
	/// 该工厂用于产品化场景，确保外部拿到的对象已经完成 initialize；
	/// 如果初始化失败，则返回空指针，避免后续误用半初始化对象。
	static std::shared_ptr<HicControlCoordinator> create(
		const HicControlConfig& config);

	HicControlCoordinator();
	~HicControlCoordinator();

	/// @brief 初始化整个控制协调器。
	/// 说明：
	/// 1. 复制外部基础配置；
	/// 2. 依次初始化运动学、动力学、阻抗核、状态观测器、力观测器；
	/// 3. 清空模式状态和历史缓存，确保后续每条控制链从干净状态启动。
	HicStatus initialize(const HicControlConfig& config);

	// ============================================================
	// 2. 高频状态输入
	// ============================================================

	/// @brief 高频机器人状态输入接口。
	/// @note 该接口适合“外部只提供关节位置和电机电流，速度/加速度由内部差分估计”的场景。
	HicStatus updateRobotState(
		const double* jointPosition,
		const double* motorCurrent,
		double currentTime);

	/// @brief 更新包含关节加速度的关节状态。
	/// 适用于上层已经有独立加速度估计的场景；该接口只进入状态观测器，
	/// 不会隐式修改外力观测器输入。
	HicStatus updateJointState(
		const double* jointPosition,
		const double* jointVelocity,
		const double* jointAcceleration);
	/// @brief 更新外部实测关节力矩。
	/// @note 该量主要进入状态观测/感知链路，本身不直接等价于零力或阻抗主输出。
	HicStatus updateJointMeasuredTorque(const double* jointMeasuredTorque);

	// ============================================================
	// 3. 模式管理
	// ============================================================

	/// @brief 直接设置顶层控制模式。
	/// @warning 这是较底层的模式切换入口，不做专门的进入安全校验；
	/// 若切换到具体模式，优先使用各个 startForceControl... 接口。
	HicStatus setControlMode(HicControlMode mode);

	/// @brief 尝试进入零力示教模式。
	/// @note 进入前会做一组偏保守的安全检查，任何一项失败都拒绝进入。
	HicStatus startForceControlZeroForceMode();
	/// @brief 启动力控模式下的定点位置保持模式。
	HicStatus startForceControlCartesianFixedPositionMode();
	/// @brief 启动力控模式下的定点位姿保持模式。
	HicStatus startForceControlCartesianFixedPoseMode();
	/// @brief 启动力控模式下的轨迹笛卡尔阻抗模式。
	HicStatus startForceControlCartesianTrajectoryMode();
	HicStatus startForceControlJointImpedanceMode();
	/// @brief 请求退出当前力控模式。
	/// @note 零力示教走平滑退出；其余笛卡尔力控子模式当前直接回到 IDLE。
	HicStatus prepareStopForceControlMode();

	// ============================================================
	// 4. 控制参数与目标配置
	// ============================================================

	/// @brief 设置笛卡尔阻抗增益。
	/// @note 该接口只负责把参数转发给阻抗核，不涉及目标位姿。
	HicStatus setCartesianImpedanceGains(const HicImpedanceGains& gains);
	/// @brief 设置零空间控制配置。
	HicStatus setNullspaceConfig(const HicNullspaceControlConfig& config);
	/// @brief 抓取当前关节位置作为零空间参考关节位置。
	HicStatus captureCurrentJointPositionAsNullspaceTarget();
	HicStatus setJointImpedanceConfig(const HicJointImpedanceConfig& config);
	HicStatus captureCurrentJointPositionAsImpedanceTarget();
	/// @brief 设置固定点位置保持目标。
	HicStatus setFixedTargetPosition(const double targetPosition[3]);
	/// @brief 设置固定点阻抗目标位姿。
	HicStatus setFixedTargetPose(const double targetPose[HIC_POSE_DIM]);
	/// @brief 通过当前 q 计算当前末端位置，并捕获为固定点位置保持目标。
	HicStatus captureCurrentPositionAsFixedTarget();
	/// @brief 通过当前 q 计算当前末端位姿，并捕获为固定点阻抗目标。
	HicStatus captureCurrentPoseAsFixedTarget();
	/// @brief 设置轨迹模式下的在线位姿/速度目标。
	HicStatus setOnlineTargetPose(
		const double targetPose[HIC_POSE_DIM],
		const double targetVelocity[HIC_CARTESIAN_DIM]);
	/// @brief 在线更新动力学线性参数。
	HicStatus setDynamicsLinearParameters(const double* dynamicParams);
	/// @brief 在线更新末端负载质量和质心。
	HicStatus setPayloadMassProperties(double mass, const double centerOfMass[3]);
	/// @brief 设置动力学重力向量。
	HicStatus setGravityVector(double gx, double gy, double gz);
	/// @brief 设置电流-力矩换算参数。
	/// @note 这些参数既影响状态观测器里的电机估计力矩，也影响最终输出电流换算。
	HicStatus setMotorTorqueConversionParameters(
		const double* torqueConstant,
		const double* gearRatio,
		const double* transmissionEfficiency);
	/// @brief 设置关节位置安全范围。
	HicStatus setJointPositionLimits(const double* lowerLimits, const double* upperLimits);
	/// @brief 设置关节速度安全范围。
	HicStatus setJointVelocityLimits(const double* lowerLimits, const double* upperLimits);
	/// @brief 设置关节加速度安全范围。
	HicStatus setJointAccelerationLimits(const double* lowerLimits, const double* upperLimits);
	/// @brief 设置关节力矩安全范围。
	HicStatus setJointTorqueLimits(const double* lowerLimits, const double* upperLimits);
	/// @brief 设置电机电流安全范围。
	HicStatus setMotorCurrentLimits(const double* lowerLimits, const double* upperLimits);
	/// @brief 更新外部关节外力矩估计。
	/// @note 当前 ZeroForceMode 第一版没有使用该量，但接口保留供其他链路复用。
	HicStatus updateJointExternalTorque(const double* jointExternalTorque);
	/// @brief 在线设置机器人状态观测器配置。
	HicStatus setRobotStateObserverConfig(const HicRobotStateObserverConfig& config);
	/// @brief 读取当前机器人状态观测器配置。
	HicStatus getRobotStateObserverConfig(HicRobotStateObserverConfig& configOut) const;
	/// @brief 设置扭矩传感器配置。
	HicStatus setTorqueSensorConfig(const HicTorqueSensorConfig& config);
	/// @brief 读取扭矩传感器配置。
	HicStatus getTorqueSensorConfig(HicTorqueSensorConfig& configOut) const;
	/// @brief 从文件加载扭矩传感器配置。
	HicStatus loadTorqueSensorConfigFromFile(const char* filePath);
	/// @brief 更新原始关节扭矩传感器输入。
	HicStatus updateRawJointTorqueSensor(const double* rawTorqueByHardwareChannel);
	/// @brief 获取标定后的关节扭矩传感器值。
	HicStatus getCalibratedJointTorqueSensor(double* calibratedJointTorque) const;
	/// @brief 获取扭矩传感器故障状态。
	HicStatus getTorqueSensorFaultStatus(bool* faultStatus) const;

	// ============================================================
	// 5. 控制命令输出
	// ============================================================

	/// @brief 计算零力示教模式下的电机电流命令。
	/// 计算链路：
	/// 1. 重新检查状态是否满足零力模式运行条件；
	/// 2. 重力补偿 + 速度阻尼；
	/// 3. 软/硬限位保护；
	/// 4. 力矩安全限幅；
	/// 5. 力矩转电流并做电流兜底限幅。
	HicStatus computeZeroForceTorqueCommand(
		double* jointTorqueCommand,
		bool* jointProtectionStatus);

	/// @brief 计算零力示教模式下的电机电流命令。
	/// @note 内部先复用零力示教的关节力矩命令，再统一做电流换算。
	HicStatus computeZeroForceCurrentCommand(
		double* motorCurrentCommand,
		bool* jointProtectionStatus);

	/// @brief 计算笛卡尔阻抗模式下的电机电流命令。
	/// @note 内部先复用力矩输出，再统一做电流换算。
	HicStatus computeCartesianImpedanceCurrentCommand(
		double* motorCurrentCommand,
		bool* jointProtectionStatus);
	/// @brief 输出关节力矩命令。
	/// 用于调试、仿真或直接力矩驱动；内部与电流输出共用同一次阻抗求解结果。
	HicStatus computeCartesianImpedanceTorqueCommand(
		double* jointTorqueCommand,
		bool* jointProtectionStatus);
	HicStatus computeForceControlTorqueCommand(
		double* jointTorqueCommand,
		bool* jointProtectionStatus);
	HicStatus computeForceControlCurrentCommand(
		double* motorCurrentCommand,
		bool* jointProtectionStatus);

	// ============================================================
	// 6. 状态读取与调试
	// ============================================================

	/// @brief 获取当前末端位姿和末端 twist。
	/// 该接口根据当前关节状态和运动学适配器实时计算，不做电流换算。
	HicStatus getCurrentCartesianState(
		double* currentPose,
		double* currentTwist);
	HicControlMode getControlMode() const;
	HicForceControlMode getForceControlMode() const;
	bool isNullspaceEnabled() const;
	int getJointCount() const;
	HicStatus getRobotState(HicRobotState& stateOut) const;
	/// @brief 读取当前基础配置快照。
	HicStatus getConfigSnapshot(HicControlConfig& configOut) const;
	/// @brief 获取最近一次阻抗核输出的末端 wrench 命令。
	HicStatus getLastCartesianWrenchCommand(double wrenchCommand[HIC_CARTESIAN_DIM]) const;
	/// @brief 获取最近一次缓存的关节力矩命令。
	HicStatus getLastJointTorqueCommand(double* jointTorqueCommand) const;
	/// @brief 获取最近一次执行状态码。
	HicStatus getLastStatus() const;

	/// @brief 清空内部运行态并回到默认模式。
	/// @note reset 不会重建对象，也不会重新加载基础配置，只重置运行时状态和缓存。
	HicStatus reset();

private:
	// ============================================================
	// A. ZeroForceMode 内部辅助
	// ============================================================

	/// @brief 零力模式进入/运行前的安全校验。
	/// @param stateOut 返回当前已观测到的关节状态，便于调用方复用，避免重复取状态。
	HicStatus validateZeroForceEntry(HicRobotState* stateOut) const;
	/// @brief 在软限位区域叠加附加阻尼，在硬限位保护区直接报错。
	HicStatus applyZeroForceSoftBoundaryDamping(
		const double* jointPosition,
		const double* jointVelocity,
		double* jointTorqueCommand,
		bool* jointProtectionStatus,
		bool exitDampingActive) const;
	/// @brief 判断零力模式是否已经满足“平滑退出到 IDLE”的速度条件。
	bool isZeroForceExitReady(const double* jointVelocity) const;

	// ============================================================
	// B. 笛卡尔阻抗主求解与通用安全链路
	// ============================================================

	/// @brief 执行一次完整的笛卡尔阻抗求解。
	/// @note 该函数负责状态读取、运动学、动力学、阻抗核以及安全限幅的串联。
	HicStatus runCartesianImpedanceStep(
		double* jointTorqueCommand,
		bool* jointProtectionStatus);
	HicStatus computeJointImpedanceTorqueCommand(
		double* jointTorqueCommand,
		bool* jointProtectionStatus);

	/// @brief 对关节力矩命令做统一安全限幅。
	/// 目前包含：
	/// 1. 关节位置硬限位；
	/// 2. 力矩斜率限制；
	/// 3. 关节力矩幅值限制。
	HicStatus applySafetyLimits(
		const double* jointPosition,
		double* jointTorqueCommand,
		bool* jointProtectionStatus);

	/// @brief 将关节力矩命令换算为电机电流命令。
	/// @note 换算公式依赖 torqueConstant * gearRatio * transmissionEfficiency。
	HicStatus convertTorqueToCurrent(
		const double* jointTorqueCommand,
		double* motorCurrentCommand) const;

	// ============================================================
	// C. 配置同步与通用工具
	// ============================================================

	/// @brief 将最新 config_ 中与状态观测相关的字段同步回状态观测器。
	HicStatus rebuildStateObserverConfig();
	/// @brief 基于当前 coordinator 配置构造默认状态观测器配置。
	void buildDefaultStateObserverConfig(HicRobotStateObserverConfig* configOut) const;

	/// @brief 将电流输出数组清零。
	void clearCurrentCommand(double* motorCurrentCommand) const;
	/// @brief 将力矩输出数组清零。
	void clearTorqueCommand(double* jointTorqueCommand) const;
	/// @brief 将关节保护标志数组清零。
	void clearProtectionStatus(bool* jointProtectionStatus) const;
	/// @brief 检查数组中是否包含 NaN / Inf。
	bool hasNaNOrInf(const double* data, int size) const;
	/// @brief 使“本周期控制命令缓存”失效。
	/// @note 当输入状态、参数或目标发生变化时，应调用它强制下一次重新计算。
	void invalidateCommandCache();

private:
	// ============================================================
	// D. 运行时状态与子模块成员
	// ============================================================

	/// @brief 对象是否已经完成 initialize。
	bool initialized_;
	/// @brief 当前生效的总配置副本。
	HicControlConfig config_;
	/// @brief 当前顶层控制模式。
	HicControlMode controlMode_;
	/// @brief 当前力控模式。
	HicForceControlMode forceControlMode_;
	/// @brief 协调器记录的当前时间戳。
	double currentTime_;

	/// @brief 机器人状态观测器，统一管理关节位置/速度/电流等状态。
	HicRobotStateObserver robotStateObserver_;
	/// @brief 运动学适配器，提供 FK/Jacobian/Twist 等计算。
	HicKinematicsAdapter kinematicsAdapter_;
	/// @brief 动力学适配器，提供重力等模型项计算。
	HicDynamicsAdapter dynamicsAdapter_;
	/// @brief 笛卡尔阻抗控制核心。
	HicCartesianImpedanceCore impedanceCore_;
	HicJointImpedanceCore jointImpedanceCore_;
	/// @brief 交互力/扭矩感知模块。
	HicInteractionForceObserver forceObserver_;
	/// @brief 当前零空间控制配置缓存。
	HicNullspaceControlConfig nullspaceConfig_;
	HicJointImpedanceConfig jointImpedanceConfig_;

	// ============================================================
	// E. 历史命令、缓存与零力模式内部状态
	// ============================================================

	/// @brief 上一周期的关节力矩命令，用于力矩斜率限制。
	double previousJointTorqueCommand_[HIC_MAX_JOINTS];
	/// @brief 上一周期的电机电流命令，当前主要用于调试和后续扩展。
	double previousMotorCurrentCommand_[HIC_MAX_JOINTS];
	/// @brief 最近一次成功计算得到的关节力矩命令缓存。
	double lastJointTorqueCommand_[HIC_MAX_JOINTS];
	/// @brief 最近一次成功计算得到的关节保护状态缓存。
	bool lastJointProtectionStatus_[HIC_MAX_JOINTS];
	/// @brief 零力模式的基础速度阻尼系数。
	double zeroForceDamping_[HIC_MAX_JOINTS];
	/// @brief 零力模式准备退出阶段使用的更强阻尼系数。
	double zeroForceExitDamping_[HIC_MAX_JOINTS];
	/// @brief 最近一次成功计算得到的末端位姿缓存。
	double lastCurrentPose_[HIC_POSE_DIM];
	/// @brief 最近一次成功计算得到的末端 twist 缓存。
	double lastCurrentTwist_[HIC_CARTESIAN_DIM];
	/// @brief 最近一次接口调用的状态码。
	HicStatus lastStatus_;
	/// @brief 命令输入版本号。
	/// @note 只要影响控制结果的输入发生变化，就递增该值。
	std::uint64_t commandInputVersion_;
	/// @brief 最近一次命令缓存对应的输入版本号。
	std::uint64_t lastComputedVersion_;
	/// @brief 当前命令缓存是否有效。
	bool commandCacheValid_;
	/// @brief 零力模式是否已经收到“准备退出”的请求。
	bool zeroForceStopRequested_;
};

} // namespace hic
