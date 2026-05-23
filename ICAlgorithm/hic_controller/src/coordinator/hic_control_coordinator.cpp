#include "hic_controller/coordinator/hic_control_coordinator.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace hic
{
namespace
{
// ============================================================
// 0. 文件内局部工具函数
// 说明：
// 1. 这些函数只在本文件内部使用；
// 2. 主要负责默认阈值、限位区宽度和通用边界整理；
// 3. 放在最前面，便于后续成员函数直接复用。
// ============================================================

/// @brief 根据 robotType 推导该机型期望的关节数。
/// @note 这里做的是“初始化前的快速防呆”，不是完整的机型合法性校验。
int getExpectedJointCountFromRobotType(int robotType)
{
	switch (robotType)
	{
	case 0:
	case 1:
	case 2:
	case 3:
	case 5:
	case 7:
	case 8:
	case 9:
	case 10:
	case 12:
		return 6;
	case 20:
		return 7;
	default:
		// 未知或自定义 robotType 当前不做严格匹配，由各 adapter 自行决定是否可初始化。
		return -1;
	}
}

/// @brief 当某个量没有显式配置上下界时，使用一个极大值作为“近似无限”边界。
double defaultLargeLimit()
{
	return 1.0e12;
}

/// @brief 返回一对上下界中绝对值较大的那个值。
/// @note 主要用于把非对称上下界折算成“对称 maxAbs”表示。
double maxAbsLimit(double lower, double upper)
{
	return std::max(std::fabs(lower), std::fabs(upper));
}

/// @brief 判断一组上下界是否被显式配置过。
/// @note 当前工程约定：上下界同时为 0 时，表示“未配置”而不是“只允许 0”。
bool hasExplicitRange(double lower, double upper)
{
	return lower != 0.0 || upper != 0.0;
}

/// @brief 获取一组上下界中可用于“绝对值阈值判断”的正值幅度。
double configuredPositiveAbsLimit(double lower, double upper)
{
	if (!hasExplicitRange(lower, upper))
	{
		return 0.0;
	}
	return std::max(std::fabs(lower), std::fabs(upper));
}

/// @brief 按“显式上下界优先，其次对称绝对值限幅”的规则对数值进行钳位。
/// @note 该工具函数被力矩限幅、电流限幅等多个场景共用，以保持行为一致。
double clampToConfiguredRange(double value, double lower, double upper, double symmetricAbs)
{
	if (hasExplicitRange(lower, upper))
	{
		if (lower > upper)
		{
			return 0.0;
		}
		return std::max(lower, std::min(upper, value));
	}
	if (symmetricAbs > 0.0)
	{
		return std::max(-symmetricAbs, std::min(symmetricAbs, value));
	}
	return value;
}

/// @brief 将“上下界或对称绝对值”统一展开为 lower / upper / maxAbs 三种表示。
/// @note 状态观测器内部同时维护这三种形式，因此 coordinator 负责在这里做一次归一化。
void fillRangeFromBoundsOrSymmetric(
	double lowerBound,
	double upperBound,
	double symmetricAbs,
	double* lowerOut,
	double* upperOut,
	double* maxAbsOut)
{
	if (!lowerOut || !upperOut || !maxAbsOut)
	{
		return;
	}

	if (hasExplicitRange(lowerBound, upperBound))
	{
		*lowerOut = lowerBound;
		*upperOut = upperBound;
		*maxAbsOut = maxAbsLimit(lowerBound, upperBound);
		return;
	}

	if (symmetricAbs > 0.0)
	{
		*lowerOut = -symmetricAbs;
		*upperOut = symmetricAbs;
		*maxAbsOut = symmetricAbs;
		return;
	}

	*lowerOut = -defaultLargeLimit();
	*upperOut = defaultLargeLimit();
	*maxAbsOut = defaultLargeLimit();
}

/// @brief 给 ZeroForceMode 提供一组保守的默认关节阻尼。
/// @note 这一版以“安全稳定”为目标，不追求手感最优，因此阻尼取值偏保守。
double defaultZeroForceDampingValue(int jointIndex)
{
	static const double kDefaultDamping[7] = { 1.5, 1.5, 1.2, 0.8, 0.5, 0.3, 0.2 };
	if (jointIndex >= 0 && jointIndex < 7)
	{
		return kDefaultDamping[jointIndex];
	}
	return 0.3;
}

/// @brief 计算零力模式允许进入时的关节速度阈值。
/// @note 若上层已经配置速度限幅，则取其一部分作为进入条件；否则使用默认值。
double computeZeroForceEntryVelocityLimit(const HicControlConfig& config, int jointIndex)
{
	const double configuredLimit = configuredPositiveAbsLimit(
		config.lowerJointVelocity[jointIndex],
		config.upperJointVelocity[jointIndex]);
	if (configuredLimit > 0.0)
	{
		return std::max(0.05, std::min(0.3, 0.2 * configuredLimit));
	}
	return 0.2;
}

/// @brief 计算零力模式允许平滑退出到 IDLE 的速度阈值。
/// @note 退出阈值设计得比进入阈值更小，避免还在明显运动时过早切回 IDLE。
double computeZeroForceExitVelocityThreshold(const HicControlConfig& config, int jointIndex)
{
	return std::min(0.05, 0.5 * computeZeroForceEntryVelocityLimit(config, jointIndex));
}

/// @brief 判断关节位置限位是否配置为一个有效窗口。
bool hasValidJointLimitWindow(const HicControlConfig& config, int jointIndex)
{
	return hasExplicitRange(config.lowerJointLimit[jointIndex], config.upperJointLimit[jointIndex]) &&
		config.lowerJointLimit[jointIndex] < config.upperJointLimit[jointIndex];
}

/// @brief 计算硬限位保护区宽度。
/// @note 进入这个区域即视为风险过高，ZeroForceMode 会直接拒绝进入或立刻报错。
double computeHardLimitMargin(const HicControlConfig& config, int jointIndex)
{
	if (!hasValidJointLimitWindow(config, jointIndex))
	{
		return 0.0;
	}
	const double range = config.upperJointLimit[jointIndex] - config.lowerJointLimit[jointIndex];
	return std::max(0.01, std::min(0.03, 0.05 * range));
}

/// @brief 计算软限位阻尼区宽度。
/// @note 软限位区用于“提前加阻尼”，避免关节继续朝硬限位方向逼近。
double computeSoftLimitMargin(const HicControlConfig& config, int jointIndex)
{
	if (!hasValidJointLimitWindow(config, jointIndex))
	{
		return 0.0;
	}
	const double range = config.upperJointLimit[jointIndex] - config.lowerJointLimit[jointIndex];
	return std::max(0.08, std::min(0.2, 0.15 * range));
}

/// @brief 根据距离限位的远近，计算软限位附加阻尼的缩放系数。
/// @return 0 表示不在软限位区；1 表示已经逼近硬限位边缘。
double computeSoftLimitDampingScale(double distanceToLimit, double hardMargin, double softMargin)
{
	if (softMargin <= hardMargin || distanceToLimit >= softMargin)
	{
		return 0.0;
	}
	if (distanceToLimit <= hardMargin)
	{
		return 1.0;
	}
	return (softMargin - distanceToLimit) / (softMargin - hardMargin);
}
}

// ============================================================
// 1. 生命周期与初始化
// ============================================================

std::shared_ptr<HicControlCoordinator> HicControlCoordinator::create(
	const HicControlConfig& config)
{
	// 工厂模式的价值在于：外部永远拿到“可直接用”或“明确失败”的对象，
	// 避免半初始化对象在后续实时链路中被误用。
	std::shared_ptr<HicControlCoordinator> coordinator =
		std::make_shared<HicControlCoordinator>();
	if (!coordinator)
	{
		return nullptr;
	}

	if (coordinator->initialize(config) != HIC_STATUS_OK)
	{
		return nullptr;
	}

	return coordinator;
}

HicControlCoordinator::HicControlCoordinator()
	: initialized_(false),
	  controlMode_(HIC_CONTROL_MODE_IDLE),
	  forceControlMode_(HIC_FORCE_CONTROL_MODE_NONE),
	  currentTime_(0.0),
	  lastStatus_(HIC_STATUS_OK),
	  commandInputVersion_(0),
	  lastComputedVersion_(0),
	  commandCacheValid_(false),
	  zeroForceStopRequested_(false)
{
	// 构造阶段只做“清零和默认值”工作，不做任何依赖外部配置的初始化。
	std::memset(&config_, 0, sizeof(config_));
	std::memset(&nullspaceConfig_, 0, sizeof(nullspaceConfig_));
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
	std::fill(previousJointTorqueCommand_, previousJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousMotorCurrentCommand_, previousMotorCurrentCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointTorqueCommand_, lastJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointProtectionStatus_, lastJointProtectionStatus_ + HIC_MAX_JOINTS, false);
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		// 退出阶段阻尼取得更大，目的是让机械臂更快衰减到静止，便于安全切回 IDLE。
		zeroForceDamping_[i] = defaultZeroForceDampingValue(i);
		zeroForceExitDamping_[i] = 3.0 * zeroForceDamping_[i];
	}
	std::fill(lastCurrentPose_, lastCurrentPose_ + HIC_POSE_DIM, 0.0);
	std::fill(lastCurrentTwist_, lastCurrentTwist_ + HIC_CARTESIAN_DIM, 0.0);
	std::memset(&nullspaceConfig_, 0, sizeof(nullspaceConfig_));
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
}

HicControlCoordinator::~HicControlCoordinator()
{
}

HicStatus HicControlCoordinator::initialize(const HicControlConfig& config)
{
	// 第一层校验只检查“尺寸和控制周期”这类结构性合法性。
	if (config.jointCount <= 0 || config.jointCount > HIC_MAX_JOINTS || config.controlPeriod <= 0.0)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}
	// robotType 表示机器人型号，jointCount 表示该型号实际参与控制的关节数。
	// 初始化阶段检查两者是否匹配，防止 robotType 与 jointCount 配置不一致导致模型计算错误。
	const int expectedJointCount = getExpectedJointCountFromRobotType(config.robotType);
	if (expectedJointCount > 0 && config.jointCount != expectedJointCount)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	config_ = config;
	// 初始化时总是回到一个完全确定的基线状态，避免复用旧实例时残留历史状态。
	currentTime_ = 0.0;
	controlMode_ = HIC_CONTROL_MODE_IDLE;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
	std::fill(previousJointTorqueCommand_, previousJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousMotorCurrentCommand_, previousMotorCurrentCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointTorqueCommand_, lastJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointProtectionStatus_, lastJointProtectionStatus_ + HIC_MAX_JOINTS, false);
	std::fill(lastCurrentPose_, lastCurrentPose_ + HIC_POSE_DIM, 0.0);
	std::fill(lastCurrentTwist_, lastCurrentTwist_ + HIC_CARTESIAN_DIM, 0.0);
	lastStatus_ = HIC_STATUS_OK;
	commandInputVersion_ = 0;
	lastComputedVersion_ = 0;
	commandCacheValid_ = false;
	zeroForceStopRequested_ = false;

	HicStatus status = kinematicsAdapter_.initialize(config.robotType, config.jointCount, config.kinematicParams);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = dynamicsAdapter_.initialize(config.robotType, config.jointCount, config.dynamicParams);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = dynamicsAdapter_.setRobotKinematicParameters(config.kinematicParams);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.initialize(config.jointCount, config.controlPeriod);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = jointImpedanceCore_.initialize(config.jointCount, config.controlPeriod);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}
	jointImpedanceCore_.reset();
	status = jointImpedanceCore_.setConfig(jointImpedanceConfig_);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	HicRobotStateObserverConfig stateConfig = {};
	buildDefaultStateObserverConfig(&stateConfig);
	status = robotStateObserver_.initialize(stateConfig);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = forceObserver_.initialize(config.jointCount, config.controlPeriod);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	initialized_ = true;
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

// ============================================================
// 2. 配置构造与同步
// 说明：
// 1. 将 coordinator 的总配置投影成状态观测器可消费的格式；
// 2. 这部分虽然不直接输出控制命令，但会影响状态有效性与安全检查。
// ============================================================

void HicControlCoordinator::buildDefaultStateObserverConfig(
	HicRobotStateObserverConfig* configOut) const
{
	// 这个函数的职责不是“生成控制参数”，而是把 coordinator 的统一配置
	// 映射成状态观测器能直接消费的配置格式。
	if (!configOut)
	{
		return;
	}

	HicRobotStateObserverConfig& stateConfig = *configOut;
	stateConfig = {};
	stateConfig.jointCount = config_.jointCount;
	stateConfig.controlPeriod = config_.controlPeriod;
	stateConfig.enablePositionFilter = false;
	stateConfig.enableVelocityFilter = false;
	stateConfig.enableAccelerationFilter = false;
	stateConfig.enableMotorCurrentFilter = false;
	stateConfig.enableMeasuredTorqueFilter = false;
	stateConfig.enableCurrentToTorqueEstimate = true;

	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		// 默认关闭滤波，因此 alpha 先都设成 1.0；
		// 如果后续上层打开滤波，这些 alpha 也依然是合法值。
		stateConfig.positionFilterAlpha[i] = 1.0;
		stateConfig.velocityFilterAlpha[i] = 1.0;
		stateConfig.accelerationFilterAlpha[i] = 1.0;
		stateConfig.motorCurrentFilterAlpha[i] = 1.0;
		stateConfig.measuredTorqueFilterAlpha[i] = 1.0;

		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointLimit[i],
			config_.upperJointLimit[i],
			0.0,
			&stateConfig.lowerJointPosition[i],
			&stateConfig.upperJointPosition[i],
			&stateConfig.maxAbsJointPosition[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointVelocity[i],
			config_.upperJointVelocity[i],
			0.0,
			&stateConfig.lowerJointVelocity[i],
			&stateConfig.upperJointVelocity[i],
			&stateConfig.maxAbsJointVelocity[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointAcceleration[i],
			config_.upperJointAcceleration[i],
			0.0,
			&stateConfig.lowerJointAcceleration[i],
			&stateConfig.upperJointAcceleration[i],
			&stateConfig.maxAbsJointAcceleration[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxJointCurrent[i],
			&stateConfig.lowerMotorCurrent[i],
			&stateConfig.upperMotorCurrent[i],
			&stateConfig.maxAbsMotorCurrent[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointTorque[i],
			config_.upperJointTorque[i],
			config_.maxJointTorque[i],
			&stateConfig.lowerMeasuredTorque[i],
			&stateConfig.upperMeasuredTorque[i],
			&stateConfig.maxAbsMeasuredTorque[i]);

		stateConfig.torqueConstant[i] = config_.torqueConstant[i];
		stateConfig.gearRatio[i] = config_.gearRatio[i];
		stateConfig.transmissionEfficiency[i] = config_.transmissionEfficiency[i];
	}
}

HicStatus HicControlCoordinator::updateExternalTorqueEstimateFromRobotState()
{
	// 该函数把“电流反推外力矩”链路接到交互力观测器：
	// 1. robotStateObserver_ 提供滤波后的 q/dq 和由电机电流估算的关节力矩；
	// 2. dynamicsAdapter_ 计算当前状态下的模型力矩；
	// 3. 二者相减得到外界作用在关节上的估计力矩；
	// 4. forceObserver_ 对该估计值做外力矩低通滤波，供关节阻抗等模式读取。
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}

	double q[HIC_MAX_JOINTS] = { 0.0 };
	double dq[HIC_MAX_JOINTS] = { 0.0 };
	double motorEstimatedTorque[HIC_MAX_JOINTS] = { 0.0 };

	// Step 1: 从状态观测器读取内部 SI 单位的滤波状态。
	HicStatus status = robotStateObserver_.getFilteredJointPosition(q);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = robotStateObserver_.getFilteredJointVelocity(dq);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	status = robotStateObserver_.getMotorEstimatedTorque(motorEstimatedTorque);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	// Step 2: 计算需要从电机估计力矩中扣掉的模型力矩。
	// 当前重力项必须可用；科氏/摩擦后端若尚未实现，则保持零向量继续运行。
	double gravityTorque[HIC_MAX_JOINTS] = { 0.0 };
	double coriolisTorque[HIC_MAX_JOINTS] = { 0.0 };
	double frictionTorque[HIC_MAX_JOINTS] = { 0.0 };

	status = dynamicsAdapter_.computeGravityTorque(q, gravityTorque);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = dynamicsAdapter_.computeCoriolisTorque(q, dq, coriolisTorque);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_NOT_IMPLEMENTED)
	{
		return status;
	}

	status = dynamicsAdapter_.computeFrictionTorque(dq, frictionTorque);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_NOT_IMPLEMENTED)
	{
		return status;
	}

	// Step 3: 外力矩估计 = 电机电流反推关节力矩 - 模型补偿力矩。
	double jointExternalTorque[HIC_MAX_JOINTS] = { 0.0 };
	for (int i = 0; i < config_.jointCount; ++i)
	{
		const double modelTorque = gravityTorque[i] + coriolisTorque[i] + frictionTorque[i];
		jointExternalTorque[i] = motorEstimatedTorque[i] - modelTorque;
	}

	// Step 4: 送入交互力观测器，统一使用 externalTorqueFilterAlpha 做滤波。
	return forceObserver_.updateJointExternalTorque(jointExternalTorque);
}

HicStatus HicControlCoordinator::rebuildStateObserverConfig()
{
	// 当外部通过 setXXXLimits / setMotorTorqueConversionParameters 等接口修改配置后，
	// 需要把和观测器相关的那部分重新同步进去，否则状态有效性检查会滞后。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	HicRobotStateObserverConfig stateConfig = {};
	lastStatus_ = robotStateObserver_.getConfig(stateConfig);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}

	stateConfig.jointCount = config_.jointCount;
	stateConfig.controlPeriod = config_.controlPeriod;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointLimit[i],
			config_.upperJointLimit[i],
			0.0,
			&stateConfig.lowerJointPosition[i],
			&stateConfig.upperJointPosition[i],
			&stateConfig.maxAbsJointPosition[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxJointCurrent[i],
			&stateConfig.lowerMotorCurrent[i],
			&stateConfig.upperMotorCurrent[i],
			&stateConfig.maxAbsMotorCurrent[i]);
		fillRangeFromBoundsOrSymmetric(
			config_.lowerJointTorque[i],
			config_.upperJointTorque[i],
			config_.maxJointTorque[i],
			&stateConfig.lowerMeasuredTorque[i],
			&stateConfig.upperMeasuredTorque[i],
			&stateConfig.maxAbsMeasuredTorque[i]);
		stateConfig.torqueConstant[i] = config_.torqueConstant[i];
		stateConfig.gearRatio[i] = config_.gearRatio[i];
		stateConfig.transmissionEfficiency[i] = config_.transmissionEfficiency[i];
	}
	lastStatus_ = robotStateObserver_.setConfig(stateConfig);
	return lastStatus_;
}

// ============================================================
// 3. 高频状态输入
// ============================================================

HicStatus HicControlCoordinator::updateRobotState(
	const double* jointPosition,
	const double* motorCurrent,
	double currentTime)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.updateRobotState(
		jointPosition, motorCurrent, currentTime);
	if (status == HIC_STATUS_OK)
	{
		// 高频状态输入成功后立即刷新外力矩估计。
		// 这样后续关节阻抗模式启用外力矩补偿时，可以直接从 forceObserver_ 读取滤波后的 tau_ext_hat。
		const HicStatus externalTorqueStatus = updateExternalTorqueEstimateFromRobotState();
		if (externalTorqueStatus != HIC_STATUS_OK)
		{
			lastStatus_ = externalTorqueStatus;
			return lastStatus_;
		}

		// 一旦底层状态更新成功，依赖该状态的控制命令缓存就不再可信，必须失效。
		currentTime_ = currentTime;
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::updateJointState(
	const double* jointPosition,
	const double* jointVelocity,
	const double* jointAcceleration)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.updateJointState(
		jointPosition, jointVelocity, jointAcceleration);
	if (status == HIC_STATUS_OK)
	{
		// 这个接口不显式传时间戳，因此 coordinator 用控制周期做单步推进。
		currentTime_ += config_.controlPeriod;
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::updateJointMeasuredTorque(const double* jointMeasuredTorque)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.updateJointMeasuredTorque(jointMeasuredTorque);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

// ============================================================
// 4. 模式管理
// 说明：
// 1. 顶层只区分 FORCE_CONTROL / IDLE；
// 2. 零力示教、定点阻抗、轨迹阻抗都属于 FORCE_CONTROL 内部子模式；
// 3. 其中零力模式进入和退出带有额外的安全语义。
// ============================================================

HicStatus HicControlCoordinator::setControlMode(HicControlMode mode)
{
	// 直接模式切换只维护顶层状态机和缓存，不做专门的安全判断。
	controlMode_ = mode;
	if (mode != HIC_CONTROL_MODE_FORCE_CONTROL)
	{
		forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
		zeroForceStopRequested_ = false;
	}
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlZeroForceMode()
{
	// =========================
	// 零力示教模式入口
	// =========================
	// 这个函数的职责不是“立即产生零力输出”，而是负责把系统安全地切换到
	// HIC_CONTROL_MODE_FORCE_CONTROL + HIC_FORCE_CONTROL_MODE_ZERO_FORCE 状态。
	//
	// 可以把它理解成零力示教的“准入门禁”：
	// 1. coordinator 本身必须已经初始化；
	// 2. 机器人当前状态必须已经更新且有效；
	// 3. 当前速度不能过大；
	// 4. 当前位置不能已经逼近硬限位；
	// 5. 电流-力矩换算参数必须有效。
	//
	// 只有这些条件都满足时，后续 computeZeroForceCurrentCommand() 才有意义。
	// 否则即便强行进入模式，也可能在第一周期就输出不安全或不可解释的命令。
	// ZeroForceMode 的入口尽量保守：
	// 即便上层已经显式调用 start，也必须再次做运行条件校验。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	HicRobotState state = {};
	const HicStatus status = validateZeroForceEntry(&state);
	if (status != HIC_STATUS_OK)
	{
		// 进入失败时显式退回 IDLE，语义上表示：
		// “这次零力示教请求没有被接受，系统仍处于空闲状态”。
		//
			// 这里不保留任何“半进入”痕迹，避免上层误以为当前已经进入零力模式，
			// 但实际上内部条件并不满足。
			// 进入失败时强制回到 IDLE，避免对外留下“似乎已经部分进入零力模式”的歧义状态。
			controlMode_ = HIC_CONTROL_MODE_IDLE;
			forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
			zeroForceStopRequested_ = false;
			lastStatus_ = status;
			return lastStatus_;
		}

	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_ZERO_FORCE;
	zeroForceStopRequested_ = false;
	// 一旦模式切换成功，后续输出语义就变了：
	// 同样的输入状态，此时请求的是“零力示教电流命令”而不是其他模式输出。
	// 因此需要让历史命令缓存全部失效，强制下一次按零力模式重新计算。
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::prepareStopForceControlMode()
{
	// =========================
	// 零力示教模式退出准备
	// =========================
	// 这个函数故意不直接把模式切回 IDLE。
	//
	// 原因是：零力示教退出时如果机械臂还在明显运动，立刻切 IDLE 可能让输出瞬间消失，
	// 从而造成不够平滑的行为。当前第一版采用一个更保守的策略：
	// 1. 先打上 zeroForceStopRequested_ 标记；
	// 2. 后续控制周期继续输出“带更强阻尼的零力命令”；
	// 3. 当所有关节速度都降到阈值以内，再真正切回 IDLE。
	// 这里不立即切 IDLE，而是打标记。
	// 真正退出发生在 computeZeroForceCurrentCommand() 里，
	// 因为只有那里才能结合当前速度判断是否已经足够静止。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL)
	{
		lastStatus_ = HIC_STATUS_OK;
		return lastStatus_;
	}

	if (forceControlMode_ != HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		controlMode_ = HIC_CONTROL_MODE_IDLE;
		forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
		invalidateCommandCache();
		lastStatus_ = HIC_STATUS_OK;
		return lastStatus_;
	}

	zeroForceStopRequested_ = true;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlCartesianFixedPositionMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	lastStatus_ = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}
	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlCartesianFixedPoseMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	lastStatus_ = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}
	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlCartesianTrajectoryMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	lastStatus_ = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY);
	if (lastStatus_ != HIC_STATUS_OK)
	{
		return lastStatus_;
	}
	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

HicStatus HicControlCoordinator::startForceControlJointImpedanceMode()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	controlMode_ = HIC_CONTROL_MODE_FORCE_CONTROL;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE;
	invalidateCommandCache();
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

// ============================================================
// 5. 控制参数与目标配置
// 说明：
// 1. 这部分负责阻抗参数、动力学参数、安全边界和传感器配置；
// 2. 配置变化通常不会立即输出命令，但会影响后续控制求解。
// ============================================================

HicStatus HicControlCoordinator::setCartesianImpedanceGains(const HicImpedanceGains& gains)
{
	const HicStatus status = impedanceCore_.setGains(gains);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setNullspaceConfig(const HicNullspaceControlConfig& config)
{
	nullspaceConfig_ = config;
	const HicStatus status = impedanceCore_.setNullspaceConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentJointPositionAsNullspaceTarget()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	HicRobotState state = {};
	HicStatus status = robotStateObserver_.getRobotState(state);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.captureCurrentJointPositionAsNullspaceTarget(state.jointPosition);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setJointImpedanceConfig(const HicJointImpedanceConfig& config)
{
	jointImpedanceConfig_ = config;
	const HicStatus status = jointImpedanceCore_.setConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentJointPositionAsImpedanceTarget()
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	double jointPosition[HIC_MAX_JOINTS] = { 0.0 };
	HicStatus status = robotStateObserver_.getFilteredJointPosition(jointPosition);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = jointImpedanceCore_.captureTargetPosition(jointPosition);
	if (status == HIC_STATUS_OK)
	{
		for (int i = 0; i < config_.jointCount; ++i)
		{
			jointImpedanceConfig_.targetPosition[i] = jointPosition[i];
		}
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setFixedTargetPosition(const double targetPosition[3])
{
	HicStatus status = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.setFixedTargetPosition(targetPosition);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setFixedTargetPose(const double targetPose[HIC_POSE_DIM])
{
	HicStatus status = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.setFixedTargetPose(targetPose);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentPositionAsFixedTarget()
{
	double currentPose[HIC_POSE_DIM] = { 0.0 };
	double currentTwist[HIC_CARTESIAN_DIM] = { 0.0 };
	HicStatus status = getCurrentCartesianState(currentPose, currentTwist);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = setFixedTargetPosition(currentPose);
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::captureCurrentPoseAsFixedTarget()
{
	double currentPose[HIC_POSE_DIM] = { 0.0 };
	double currentTwist[HIC_CARTESIAN_DIM] = { 0.0 };
	HicStatus status = getCurrentCartesianState(currentPose, currentTwist);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = setFixedTargetPose(currentPose);
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setOnlineTargetPose(
	const double targetPose[HIC_POSE_DIM],
	const double targetVelocity[HIC_CARTESIAN_DIM])
{
	HicStatus status = impedanceCore_.setForceControlMode(HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return status;
	}

	status = impedanceCore_.setOnlineTargetPose(targetPose, targetVelocity);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setDynamicsLinearParameters(const double* dynamicParams)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!dynamicParams)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	std::memcpy(config_.dynamicParams, dynamicParams, sizeof(config_.dynamicParams));
	lastStatus_ = dynamicsAdapter_.setDynamicParameters(config_.dynamicParams);
	if (lastStatus_ == HIC_STATUS_OK)
	{
		// 动力学参数变化会影响重力补偿等模型项，因此必须让命令缓存失效。
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setPayloadMassProperties(
	double mass,
	const double centerOfMass[3])
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!centerOfMass || mass < 0.0)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	lastStatus_ = dynamicsAdapter_.setPayloadMassProperties(mass, centerOfMass);
	if (lastStatus_ == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setGravityVector(double gx, double gy, double gz)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	lastStatus_ = dynamicsAdapter_.setGravityVector(gx, gy, gz);
	if (lastStatus_ == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setMotorTorqueConversionParameters(
	const double* torqueConstant,
	const double* gearRatio,
	const double* transmissionEfficiency)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!torqueConstant || !gearRatio || !transmissionEfficiency)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.torqueConstant[i] = torqueConstant[i];
		config_.gearRatio[i] = gearRatio[i];
		config_.transmissionEfficiency[i] = transmissionEfficiency[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	if (lastStatus_ == HIC_STATUS_OK)
	{
		// 既然换算参数变了，观测器里的估计力矩和控制输出电流都应认为需要重算。
		invalidateCommandCache();
	}
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointPositionLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointLimit[i] = lowerLimits[i];
		config_.upperJointLimit[i] = upperLimits[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointVelocityLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointVelocity[i] = lowerLimits[i];
		config_.upperJointVelocity[i] = upperLimits[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointAccelerationLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointAcceleration[i] = lowerLimits[i];
		config_.upperJointAcceleration[i] = upperLimits[i];
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setJointTorqueLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerJointTorque[i] = lowerLimits[i];
		config_.upperJointTorque[i] = upperLimits[i];
		config_.maxJointTorque[i] = maxAbsLimit(lowerLimits[i], upperLimits[i]);
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::setMotorCurrentLimits(
	const double* lowerLimits,
	const double* upperLimits)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!lowerLimits || !upperLimits)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		config_.lowerMotorCurrent[i] = lowerLimits[i];
		config_.upperMotorCurrent[i] = upperLimits[i];
		config_.maxJointCurrent[i] = maxAbsLimit(lowerLimits[i], upperLimits[i]);
	}
	lastStatus_ = rebuildStateObserverConfig();
	return lastStatus_;
}

HicStatus HicControlCoordinator::updateJointExternalTorque(const double* jointExternalTorque)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = forceObserver_.updateJointExternalTorque(jointExternalTorque);
	if (status == HIC_STATUS_OK)
	{
		// 即便当前 ZeroForceMode 不使用该量，也保持“感知输入改变 => 缓存失效”的统一语义。
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::setRobotStateObserverConfig(const HicRobotStateObserverConfig& config)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = robotStateObserver_.setConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::getRobotStateObserverConfig(HicRobotStateObserverConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return robotStateObserver_.getConfig(configOut);
}

HicStatus HicControlCoordinator::setTorqueSensorConfig(const HicTorqueSensorConfig& config)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = forceObserver_.setTorqueSensorConfig(config);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::getTorqueSensorConfig(HicTorqueSensorConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return forceObserver_.getTorqueSensorConfig(configOut);
}

HicStatus HicControlCoordinator::loadTorqueSensorConfigFromFile(const char* filePath)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	lastStatus_ = forceObserver_.loadTorqueSensorConfigFromFile(filePath);
	return lastStatus_;
}

HicStatus HicControlCoordinator::updateRawJointTorqueSensor(const double* rawTorqueByHardwareChannel)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}

	const HicStatus status = forceObserver_.updateRawJointTorqueSensor(rawTorqueByHardwareChannel);
	if (status == HIC_STATUS_OK)
	{
		invalidateCommandCache();
	}
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::getCalibratedJointTorqueSensor(double* calibratedJointTorque) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return forceObserver_.getCalibratedJointTorqueSensor(calibratedJointTorque);
}

HicStatus HicControlCoordinator::getTorqueSensorFaultStatus(bool* faultStatus) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return forceObserver_.getTorqueSensorFaultStatus(faultStatus);
}

// ============================================================
// 6. ZeroForceMode 主链路
// 说明：
// 1. 先做进入/运行安全校验；
// 2. 再计算重力补偿与速度阻尼；
// 3. 最后叠加软/硬限位保护和统一限幅；
// 4. 若上层请求电流环输出，再做力矩转电流。
// ============================================================

HicStatus HicControlCoordinator::computeZeroForceTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// ==============================
	// 零力示教主控制周期：力矩环版本
	// ==============================
	// 这是零力示教的“核心求解”版本。
	// 它输出的是关节力矩命令，适合：
	// 1. 调试零力控制律本身；
	// 2. 仿真或支持直接力矩驱动的执行器；
	// 3. 供电流环版本复用同一套主逻辑。
	//
	// 当前第一版的控制逻辑刻意保持简单且保守：
	// 1. 状态校验
	// 2. 重力补偿
	// 3. 速度阻尼
	// 4. 软/硬限位保护
	// 5. 力矩安全限幅
	//
	// 它不做：
	// - externalWrench 解释
	// - 末端六维力引导
	// - targetPose 跟踪
	// - 复杂摩擦补偿
	//
	// 因此，这里实现的是“安全优先的关节层零力示教”，
	// 而不是更复杂的交互控制器。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL ||
	    forceControlMode_ != HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		// 只有真正处于零力模式时才允许取零力输出，避免接口语义混乱。
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	HicRobotState state = {};
	HicStatus status = validateZeroForceEntry(&state);
	if (status != HIC_STATUS_OK)
	{
		// 零力模式不仅在 start 时检查一次，
		// 在每个控制周期都会再次检查当前状态是否仍然满足运行条件。
		// 这样可以覆盖“进入后状态突变”的情况，例如速度突然过大、状态失效等。
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	if (zeroForceStopRequested_ && isZeroForceExitReady(state.jointVelocity))
	{
		// 退出条件满足时，不再继续输出渐退命令，而是正式结束零力模式。
		// 当前处理策略是：
		// 1. controlMode_ 切回 IDLE；
		// 2. 清除 stop 标记；
		// 3. 使缓存失效；
		// 4. 当前周期输出保持为零。
		// 只有在退出标记已设置且所有关节速度都足够低时，才真正退回 IDLE。
		controlMode_ = HIC_CONTROL_MODE_IDLE;
		forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
		zeroForceStopRequested_ = false;
		invalidateCommandCache();
		lastStatus_ = HIC_STATUS_OK;
		return lastStatus_;
	}

	if (config_.enableGravityCompensation)
	{
		// 零力示教的第一主项是重力补偿：
		// 目标不是“把关节力矩做成 0”，而是尽量抵消机器人自身重力造成的静态负担，
		// 让人工拖动时不需要额外扛住机械臂重量。
		// 第一版零力示教把“重力补偿 + 阻尼”作为唯一主控制律，
		// 不引入额外外力引导或末端阻抗。
		status = dynamicsAdapter_.computeGravityTorque(state.jointPosition, jointTorqueCommand);
		if (status != HIC_STATUS_OK)
		{
			clearTorqueCommand(jointTorqueCommand);
			lastStatus_ = status;
			return lastStatus_;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		const double damping = zeroForceStopRequested_ ? zeroForceExitDamping_[i] : zeroForceDamping_[i];
		// 第二主项是关节速度阻尼：
		// jointTorqueCommand += -d * dq
		//
		// 它的作用不是“把机器人拉回某个位置”，
		// 而是抑制自由漂移和过快运动，让零力示教更稳定、更安全。
		//
		// 当 prepareStopZeroForceMode() 已经被调用时，
		// 这里会自动切换到更大的退出阻尼，帮助机械臂更快停稳。
		// 阻尼项始终与速度反向，用于吸收运动能量并抑制漂移。
		jointTorqueCommand[i] += -damping * state.jointVelocity[i];
		// TODO(xpm): 如后续零力手感确有需求，再补充简单摩擦补偿；当前版本优先保持可预测的安全行为。
	}

	status = applyZeroForceSoftBoundaryDamping(
		state.jointPosition,
		state.jointVelocity,
		jointTorqueCommand,
		jointProtectionStatus,
		zeroForceStopRequested_);
	if (status == HIC_STATUS_ERROR_JOINT_LIMIT)
	{
		// 一旦已经进入硬限位保护区，就不再尝试继续输出“修正型命令”，
		// 而是直接清零输出并上报关节限位错误。
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	const HicStatus safetyStatus = applySafetyLimits(
		state.jointPosition, jointTorqueCommand, jointProtectionStatus);
	if (safetyStatus != HIC_STATUS_OK && safetyStatus != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		// applySafetyLimits() 负责通用力矩安全链路。
		// 如果这里返回的是更严重的错误，就直接终止本周期输出。
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = safetyStatus;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		lastJointTorqueCommand_[i] = jointTorqueCommand[i];
		lastJointProtectionStatus_[i] = jointProtectionStatus[i];
	}

	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	lastStatus_ = (safetyStatus == HIC_STATUS_OK) ? status : safetyStatus;
	return lastStatus_;
}

HicStatus HicControlCoordinator::computeZeroForceCurrentCommand(
	double* motorCurrentCommand,
	bool* jointProtectionStatus)
{
	// ==============================
	// 零力示教主控制周期：电流环版本
	// ==============================
	// 这里不重新实现一套零力逻辑，而是复用上面的力矩环主链路：
	// 1. 先得到本周期安全限幅后的关节力矩命令；
	// 2. 再把关节力矩换算成电机电流；
	// 3. 最后做一层电流兜底限幅。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!motorCurrentCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearCurrentCommand(motorCurrentCommand);
	clearProtectionStatus(jointProtectionStatus);

	double jointTorqueCommand[HIC_MAX_JOINTS] = { 0.0 };
	HicStatus status = computeZeroForceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	const HicStatus convertStatus = convertTorqueToCurrent(jointTorqueCommand, motorCurrentCommand);
	if (convertStatus != HIC_STATUS_OK)
	{
		// 只要力矩-电流换算失败，就说明执行器参数或输入存在问题，
		// 继续下发电流命令是不安全的，因此本周期清零输出。
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = convertStatus;
		return lastStatus_;
	}

	HicStatus currentStatus = status;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		// 最后一层逐关节电流兜底保护。
		// 这样即便前面某一步未来被改动，最终输出到执行器前仍有一层明确的电流边界。
		// convertTorqueToCurrent() 已经做过一轮电流限幅；
		// 这里再兜底一次，是为了满足“异常时输出绝不越界”的保守策略。
		const double limitedCurrent = clampToConfiguredRange(
			motorCurrentCommand[i],
			config_.lowerMotorCurrent[i],
			config_.upperMotorCurrent[i],
			config_.maxJointCurrent[i]);
		if (limitedCurrent != motorCurrentCommand[i])
		{
			motorCurrentCommand[i] = limitedCurrent;
			jointProtectionStatus[i] = true;
			currentStatus = HIC_STATUS_ERROR_CURRENT_LIMIT;
		}
		previousMotorCurrentCommand_[i] = motorCurrentCommand[i];
	}

	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	// 将本周期结果缓存下来，便于调试读取和与其他输出接口共享。
	lastStatus_ = currentStatus;
	return lastStatus_;
}

HicStatus HicControlCoordinator::validateZeroForceEntry(HicRobotState* stateOut) const
{
	// 这个函数同时服务于“进入时校验”和“运行时持续校验”：
	// 只要状态已经不再满足零力模式的基本安全条件，就直接报错。
	if (!stateOut)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	const HicStatus stateStatus = robotStateObserver_.getRobotState(*stateOut);
	if (stateStatus != HIC_STATUS_OK || !robotStateObserver_.isStateValid())
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}
	if (hasNaNOrInf(stateOut->jointPosition, config_.jointCount) ||
		hasNaNOrInf(stateOut->jointVelocity, config_.jointCount) ||
		hasNaNOrInf(stateOut->motorCurrent, config_.jointCount))
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		// 进入速度阈值偏保守，避免机械臂在明显运动中切入零力模式。
		if (std::fabs(stateOut->jointVelocity[i]) > computeZeroForceEntryVelocityLimit(config_, i))
		{
			return HIC_STATUS_ERROR_ROBOT_STATE;
		}

		if (hasValidJointLimitWindow(config_, i))
		{
			// 靠近硬限位时直接拒绝进入，优先安全，不尝试“贴着边缘工作”。
			const double lower = config_.lowerJointLimit[i];
			const double upper = config_.upperJointLimit[i];
			const double hardMargin = computeHardLimitMargin(config_, i);
			if (stateOut->jointPosition[i] <= lower + hardMargin ||
				stateOut->jointPosition[i] >= upper - hardMargin)
			{
				return HIC_STATUS_ERROR_JOINT_LIMIT;
			}
		}

		const double torqueConstant = config_.torqueConstant[i];
		const double gearRatio = config_.gearRatio[i];
		const double transmissionEfficiency = config_.transmissionEfficiency[i];
		if (!std::isfinite(torqueConstant) || !std::isfinite(gearRatio) ||
			!std::isfinite(transmissionEfficiency) || torqueConstant <= 0.0 ||
			gearRatio <= 0.0 || transmissionEfficiency <= 0.0)
		{
			// 电流换算参数无效意味着后面无法安全地产生执行器输出，因此直接判失败。
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}
	}

	return HIC_STATUS_OK;
}

HicStatus HicControlCoordinator::applyZeroForceSoftBoundaryDamping(
	const double* jointPosition,
	const double* jointVelocity,
	double* jointTorqueCommand,
	bool* jointProtectionStatus,
	bool exitDampingActive) const
{
	if (!jointPosition || !jointVelocity || !jointTorqueCommand || !jointProtectionStatus)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicStatus status = HIC_STATUS_OK;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (!hasValidJointLimitWindow(config_, i))
		{
			continue;
		}

		const double lower = config_.lowerJointLimit[i];
		const double upper = config_.upperJointLimit[i];
		const double hardMargin = computeHardLimitMargin(config_, i);
		const double softMargin = std::max(computeSoftLimitMargin(config_, i), hardMargin);
		const double q = jointPosition[i];
		const double dq = jointVelocity[i];

		if (q <= lower + hardMargin || q >= upper - hardMargin)
		{
			// 一旦进入硬限位保护区，ZeroForceMode 不再尝试“救回来”，而是立即报错停输出。
			jointProtectionStatus[i] = true;
			jointTorqueCommand[i] = 0.0;
			status = HIC_STATUS_ERROR_JOINT_LIMIT;
			continue;
		}

		const double baseDamping = exitDampingActive ? zeroForceExitDamping_[i] : zeroForceDamping_[i];
		const double boundaryDamping = std::max(2.0 * baseDamping, baseDamping + 1.0);
		const double lowerDistance = q - lower;
		const double upperDistance = upper - q;

		if (dq < 0.0)
		{
			// 仅当速度方向继续逼近下限时，才叠加反向阻尼；远离限位时不做多余干预。
			const double scale = computeSoftLimitDampingScale(lowerDistance, hardMargin, softMargin);
			if (scale > 0.0)
			{
				jointTorqueCommand[i] += -boundaryDamping * (1.0 + 3.0 * scale) * dq;
			}
		}
		if (dq > 0.0)
		{
			// 同理，仅对继续逼近上限的运动加阻尼。
			const double scale = computeSoftLimitDampingScale(upperDistance, hardMargin, softMargin);
			if (scale > 0.0)
			{
				jointTorqueCommand[i] += -boundaryDamping * (1.0 + 3.0 * scale) * dq;
			}
		}
	}

	return status;
}

bool HicControlCoordinator::isZeroForceExitReady(const double* jointVelocity) const
{
	// 平滑退出的判据很简单：所有关节速度都降到阈值以下。
	if (!jointVelocity)
	{
		return false;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		if (std::fabs(jointVelocity[i]) > computeZeroForceExitVelocityThreshold(config_, i))
		{
			return false;
		}
	}
	return true;
}

// ============================================================
// 7. 笛卡尔阻抗控制输出链路
// 说明：
// 1. 支持输出关节力矩和电机电流两种形式；
// 2. 共享同一轮阻抗求解结果和缓存机制。
// ============================================================

HicStatus HicControlCoordinator::computeCartesianImpedanceTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL ||
	    (forceControlMode_ != HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION &&
	     forceControlMode_ != HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE &&
	     forceControlMode_ != HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY))
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	if (commandCacheValid_ && lastComputedVersion_ == commandInputVersion_)
	{
		// 若本周期输入没有变化，则复用上一轮已计算结果，避免重复做 FK/Jacobian/动力学求解。
		for (int i = 0; i < config_.jointCount; ++i)
		{
			jointTorqueCommand[i] = lastJointTorqueCommand_[i];
			jointProtectionStatus[i] = lastJointProtectionStatus_[i];
		}
		return lastStatus_;
	}

	const HicStatus status = runCartesianImpedanceStep(jointTorqueCommand, jointProtectionStatus);
	lastStatus_ = status;
	return status;
}

HicStatus HicControlCoordinator::computeCartesianImpedanceCurrentCommand(
	double* motorCurrentCommand,
	bool* jointProtectionStatus)
{
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!motorCurrentCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	double jointTorqueCommand[HIC_MAX_JOINTS] = { 0.0 };
	HicStatus status = computeCartesianImpedanceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = status;
		return status;
	}

	const HicStatus convertStatus = convertTorqueToCurrent(jointTorqueCommand, motorCurrentCommand);
	if (convertStatus != HIC_STATUS_OK)
	{
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = convertStatus;
		return convertStatus;
	}

	lastStatus_ = status;
	return status;
}

// ============================================================
// 8. 状态读取、复位与调试
// ============================================================

HicStatus HicControlCoordinator::computeForceControlTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// 力控模式的统一力矩分发入口。
	// 外部 hic_get_force_control_torque_commands() 只调用这里，具体算法由 forceControlMode_ 决定。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	switch (forceControlMode_)
	{
	case HIC_FORCE_CONTROL_MODE_ZERO_FORCE:
		return computeZeroForceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION:
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE:
	case HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY:
		return computeCartesianImpedanceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	case HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE:
		return computeJointImpedanceTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	case HIC_FORCE_CONTROL_MODE_NONE:
	default:
		clearTorqueCommand(jointTorqueCommand);
		clearProtectionStatus(jointProtectionStatus);
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}
}

HicStatus HicControlCoordinator::computeForceControlCurrentCommand(
	double* motorCurrentCommand,
	bool* jointProtectionStatus)
{
	// 力控模式的统一电流输出入口。
	// 所有力控子模式先走 computeForceControlTorqueCommand() 得到关节力矩，
	// 再统一复用 convertTorqueToCurrent()，避免在电流层为每个模式重复写分支。
	// 对关节阻抗模式来说，本函数的完整路径是：
	// hic_get_force_control_current_commands()
	//   -> computeForceControlCurrentCommand()
	//   -> computeForceControlTorqueCommand()
	//   -> computeJointImpedanceTorqueCommand()
	//   -> convertTorqueToCurrent()
	if (!initialized_)
	{
		// coordinator 尚未完成 initialize()，不能输出任何电流命令。
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!motorCurrentCommand || !jointProtectionStatus)
	{
		// 输出数组由外部提供，空指针时直接报错，避免写非法内存。
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	// 先清空输出，保证后续任何错误返回都不会让调用方误用旧命令。
	clearCurrentCommand(motorCurrentCommand);
	clearProtectionStatus(jointProtectionStatus);

	double jointTorqueCommand[HIC_MAX_JOINTS] = { 0.0 };
	// Step 1: 统一计算当前力控子模式下的关节力矩命令。
	// 具体是零力、笛卡尔阻抗还是关节阻抗，由 forceControlMode_ 在该函数内部继续分发。
	HicStatus status = computeForceControlTorqueCommand(jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		// 除了“命令已被限幅”这种可继续使用的状态，其他错误都不允许继续转电流。
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	// Step 2: 将关节力矩命令转换为电机电流命令。
	// 换算依赖 torqueConstant、gearRatio、transmissionEfficiency。
	const HicStatus convertStatus = convertTorqueToCurrent(jointTorqueCommand, motorCurrentCommand);
	if (convertStatus != HIC_STATUS_OK)
	{
		// 电流换算失败时同样清空输出，避免外部驱动器拿到不可信命令。
		clearCurrentCommand(motorCurrentCommand);
		lastStatus_ = convertStatus;
		return lastStatus_;
	}

	// 保留力矩计算阶段的状态码；若力矩阶段触发限幅，电流数组仍然是限幅后的可用命令。
	lastStatus_ = status;
	return lastStatus_;
}

HicStatus HicControlCoordinator::getCurrentCartesianState(
	double* currentPose,
	double* currentTwist)
{
	// 该接口只做“状态 -> 运动学量”的转换，不参与任何控制输出逻辑。
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	if (!currentPose || !currentTwist)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	HicRobotState state = {};
	HicStatus status = robotStateObserver_.getRobotState(state);
	if (status != HIC_STATUS_OK || !robotStateObserver_.isStateValid())
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	status = kinematicsAdapter_.computeForwardKinematics(state.jointPosition, currentPose);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	return kinematicsAdapter_.computeEndEffectorTwist(
		state.jointPosition, state.jointVelocity, currentTwist);
}

HicControlMode HicControlCoordinator::getControlMode() const
{
	return controlMode_;
}

HicForceControlMode HicControlCoordinator::getForceControlMode() const
{
	return forceControlMode_;
}

bool HicControlCoordinator::isNullspaceEnabled() const
{
	return nullspaceConfig_.enabled;
}

int HicControlCoordinator::getJointCount() const
{
	return config_.jointCount;
}

HicStatus HicControlCoordinator::getRobotState(HicRobotState& stateOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	return robotStateObserver_.getRobotState(stateOut);
}

HicStatus HicControlCoordinator::getConfigSnapshot(HicControlConfig& configOut) const
{
	if (!initialized_)
	{
		return HIC_STATUS_ERROR_INIT;
	}
	configOut = config_;
	return HIC_STATUS_OK;
}

HicStatus HicControlCoordinator::getLastCartesianWrenchCommand(double wrenchCommand[HIC_CARTESIAN_DIM]) const
{
	return impedanceCore_.getLastCartesianWrenchCommand(wrenchCommand);
}

HicStatus HicControlCoordinator::getLastJointTorqueCommand(double* jointTorqueCommand) const
{
	if (!jointTorqueCommand)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] = lastJointTorqueCommand_[i];
	}
	return HIC_STATUS_OK;
}

HicStatus HicControlCoordinator::getLastStatus() const
{
	return lastStatus_;
}

HicStatus HicControlCoordinator::reset()
{
	// reset 的设计目标是“快速回到运行态基线”，而不是“重建整个对象”。
	controlMode_ = HIC_CONTROL_MODE_IDLE;
	forceControlMode_ = HIC_FORCE_CONTROL_MODE_NONE;
	currentTime_ = 0.0;
	std::fill(previousJointTorqueCommand_, previousJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(previousMotorCurrentCommand_, previousMotorCurrentCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointTorqueCommand_, lastJointTorqueCommand_ + HIC_MAX_JOINTS, 0.0);
	std::fill(lastJointProtectionStatus_, lastJointProtectionStatus_ + HIC_MAX_JOINTS, false);
	std::fill(lastCurrentPose_, lastCurrentPose_ + HIC_POSE_DIM, 0.0);
	std::fill(lastCurrentTwist_, lastCurrentTwist_ + HIC_CARTESIAN_DIM, 0.0);
	std::memset(&nullspaceConfig_, 0, sizeof(nullspaceConfig_));
	std::memset(&jointImpedanceConfig_, 0, sizeof(jointImpedanceConfig_));
	robotStateObserver_.reset();
	forceObserver_.reset();
	impedanceCore_.reset();
	jointImpedanceCore_.reset();
	commandInputVersion_ = 0;
	lastComputedVersion_ = 0;
	commandCacheValid_ = false;
	zeroForceStopRequested_ = false;
	lastStatus_ = HIC_STATUS_OK;
	return lastStatus_;
}

// ============================================================
// 9. 笛卡尔阻抗内部求解与通用安全工具
// 说明：
// 1. 这一组函数是控制核心的公共底座；
// 2. ZeroForceMode 和阻抗模式都会复用其中一部分安全能力。
// ============================================================

HicStatus HicControlCoordinator::runCartesianImpedanceStep(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// 该函数是阻抗模式的一次完整控制周期：
	// 取状态 -> 算位姿/Jacobian/twist -> 算模型项 -> 算阻抗 -> 做安全限幅。
	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	double currentPose[HIC_POSE_DIM] = { 0.0 };
	double currentTwist[HIC_CARTESIAN_DIM] = { 0.0 };
	double gravityTorque[HIC_MAX_JOINTS] = { 0.0 };
	double jacobianRowMajor[HIC_MAX_JACOBIAN_SIZE] = { 0.0 };
	HicRobotState state = {};

	HicStatus status = robotStateObserver_.getRobotState(state);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}
	if (!robotStateObserver_.isStateValid())
	{
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	status = kinematicsAdapter_.computeForwardKinematics(state.jointPosition, currentPose);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = kinematicsAdapter_.computeJacobian(state.jointPosition, jacobianRowMajor);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = kinematicsAdapter_.computeEndEffectorTwist(state.jointPosition, state.jointVelocity, currentTwist);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	if (config_.enableGravityCompensation)
	{
		// 当前阻抗模式默认在关节力矩层叠加重力补偿，减轻末端维持姿态时的静态负担。
		status = dynamicsAdapter_.computeGravityTorque(state.jointPosition, gravityTorque);
		if (status != HIC_STATUS_OK)
		{
			return status;
		}
	}

	status = impedanceCore_.computeJointTorque(
		currentPose,
		currentTwist,
		state.jointPosition,
		state.jointVelocity,
		jacobianRowMajor,
		jointTorqueCommand);
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] += gravityTorque[i];
	}

	status = applySafetyLimits(state.jointPosition, jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearTorqueCommand(jointTorqueCommand);
		return status;
	}

	for (int i = 0; i < HIC_POSE_DIM; ++i)
	{
		lastCurrentPose_[i] = currentPose[i];
	}
	for (int i = 0; i < HIC_CARTESIAN_DIM; ++i)
	{
		lastCurrentTwist_[i] = currentTwist[i];
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		lastJointTorqueCommand_[i] = jointTorqueCommand[i];
		lastJointProtectionStatus_[i] = jointProtectionStatus[i];
	}
	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	return status == HIC_STATUS_OK ? HIC_STATUS_OK : status;
}

HicStatus HicControlCoordinator::computeJointImpedanceTorqueCommand(
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	// 关节空间阻抗模式完整链路：
	// 1. 从 robotStateObserver_ 读取滤波后的 q/dq；
	// 2. 按需从 forceObserver_ 读取滤波后的外力矩 tau_ext_hat；
	// 3. jointImpedanceCore_ 只计算关节空间 PD 阻抗力矩；
	// 4. coordinator 叠加重力/科氏等动力学补偿；
	// 5. 最后统一调用 applySafetyLimits()，复用现有安全保护链路。
	if (!initialized_)
	{
		lastStatus_ = HIC_STATUS_ERROR_INIT;
		return lastStatus_;
	}
	if (!jointTorqueCommand || !jointProtectionStatus)
	{
		lastStatus_ = HIC_STATUS_ERROR_NULL_POINTER;
		return lastStatus_;
	}

	clearTorqueCommand(jointTorqueCommand);
	clearProtectionStatus(jointProtectionStatus);

	if (controlMode_ != HIC_CONTROL_MODE_FORCE_CONTROL ||
	    forceControlMode_ != HIC_FORCE_CONTROL_MODE_JOINT_IMPEDANCE)
	{
		lastStatus_ = HIC_STATUS_ERROR_INVALID_PARAM;
		return lastStatus_;
	}

	if (commandCacheValid_ && lastComputedVersion_ == commandInputVersion_)
	{
		for (int i = 0; i < config_.jointCount; ++i)
		{
			jointTorqueCommand[i] = lastJointTorqueCommand_[i];
			jointProtectionStatus[i] = lastJointProtectionStatus_[i];
		}
		return lastStatus_;
	}

	double q[HIC_MAX_JOINTS] = { 0.0 };
	double dq[HIC_MAX_JOINTS] = { 0.0 };
	// Step 1: 读取滤波后的关节位置和速度，单位分别为 rad、rad/s。
	HicStatus status = robotStateObserver_.getFilteredJointPosition(q);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}
	status = robotStateObserver_.getFilteredJointVelocity(dq);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}
	if (!robotStateObserver_.isStateValid())
	{
		lastStatus_ = HIC_STATUS_ERROR_ROBOT_STATE;
		return lastStatus_;
	}

	double tauExt[HIC_MAX_JOINTS] = { 0.0 };
	const double* tauExtPtr = nullptr;
	if (jointImpedanceConfig_.enableExternalTorqueCompensation)
	{
		// Step 2: 外力矩补偿启用时，读取由电流反推链路或外部接口更新的滤波外力矩。
		status = forceObserver_.getFilteredJointExternalTorque(tauExt);
		if (status != HIC_STATUS_OK)
		{
			lastStatus_ = status;
			return lastStatus_;
		}
		tauExtPtr = tauExt;
	}

	double tauImp[HIC_MAX_JOINTS] = { 0.0 };
	// Step 3: 核心类只做关节阻抗数学，不负责动力学补偿、模式状态或安全限幅。
	status = jointImpedanceCore_.computeJointTorque(q, dq, tauExtPtr, tauImp);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}

	double tauGravity[HIC_MAX_JOINTS] = { 0.0 };
	double tauCoriolis[HIC_MAX_JOINTS] = { 0.0 };
	// Step 4: 在 coordinator 层叠加已有动力学补偿，避免阻抗核心类承担模型职责。
	status = dynamicsAdapter_.computeGravityTorque(q, tauGravity);
	if (status != HIC_STATUS_OK)
	{
		lastStatus_ = status;
		return lastStatus_;
	}
	if (config_.enableCoriolisCompensation)
	{
		status = dynamicsAdapter_.computeCoriolisTorque(q, dq, tauCoriolis);
		if (status != HIC_STATUS_OK)
		{
			lastStatus_ = status;
			return lastStatus_;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] = tauImp[i] + tauGravity[i] + tauCoriolis[i];
	}

	// Step 5: 所有关节阻抗输出必须走统一安全链路：硬限位、力矩变化率、力矩幅值限幅。
	status = applySafetyLimits(q, jointTorqueCommand, jointProtectionStatus);
	if (status != HIC_STATUS_OK && status != HIC_STATUS_ERROR_CURRENT_LIMIT)
	{
		clearTorqueCommand(jointTorqueCommand);
		lastStatus_ = status;
		return lastStatus_;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		lastJointTorqueCommand_[i] = jointTorqueCommand[i];
		lastJointProtectionStatus_[i] = jointProtectionStatus[i];
	}
	lastComputedVersion_ = commandInputVersion_;
	commandCacheValid_ = true;
	lastStatus_ = status;
	return lastStatus_;
}

HicStatus HicControlCoordinator::applySafetyLimits(
	const double* jointPosition,
	double* jointTorqueCommand,
	bool* jointProtectionStatus)
{
	if (!jointPosition || !jointTorqueCommand || !jointProtectionStatus)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}
	if (hasNaNOrInf(jointTorqueCommand, config_.jointCount))
	{
		// 控制命令一旦出现 NaN / Inf，最保守的处理就是整组清零并返回状态错误。
		std::fill(jointTorqueCommand, jointTorqueCommand + config_.jointCount, 0.0);
		return HIC_STATUS_ERROR_ROBOT_STATE;
	}

	HicStatus status = HIC_STATUS_OK;
	for (int i = 0; i < config_.jointCount; ++i)
	{
		const bool jointLimitEnabled =
			(config_.upperJointLimit[i] != 0.0 || config_.lowerJointLimit[i] != 0.0);
		if (jointLimitEnabled &&
			(jointPosition[i] > config_.upperJointLimit[i] || jointPosition[i] < config_.lowerJointLimit[i]))
		{
			// 已越过硬限位时，直接把对应关节力矩压成 0，优先保证不继续向危险方向输出。
			jointProtectionStatus[i] = true;
			jointTorqueCommand[i] = 0.0;
			status = HIC_STATUS_ERROR_JOINT_LIMIT;
			continue;
		}

		if (config_.enableTorqueRateLimit && config_.maxTorqueRate[i] > 0.0)
		{
			// 斜率限制针对的是“本周期相对上一周期的变化量”，
			// 它能抑制命令突变，降低驱动器和机械结构受到的冲击。
			const double delta = jointTorqueCommand[i] - previousJointTorqueCommand_[i];
			const double limit = config_.maxTorqueRate[i] * config_.controlPeriod;
			jointTorqueCommand[i] = previousJointTorqueCommand_[i] +
				std::max(-limit, std::min(limit, delta));
		}

		const double clampedTorque = clampToConfiguredRange(
			jointTorqueCommand[i],
			config_.lowerJointTorque[i],
			config_.upperJointTorque[i],
			config_.maxJointTorque[i]);
		if (clampedTorque != jointTorqueCommand[i])
		{
			// 力矩被夹紧并不一定是致命错误，因此返回 current limit 风格的软错误，
			// 同时让上层知道该关节进入了保护状态。
			jointTorqueCommand[i] = clampedTorque;
			jointProtectionStatus[i] = true;
			status = HIC_STATUS_ERROR_CURRENT_LIMIT;
		}
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		previousJointTorqueCommand_[i] = jointTorqueCommand[i];
	}
	return status;
}

HicStatus HicControlCoordinator::convertTorqueToCurrent(
	const double* jointTorqueCommand,
	double* motorCurrentCommand) const
{
	if (!jointTorqueCommand || !motorCurrentCommand)
	{
		return HIC_STATUS_ERROR_NULL_POINTER;
	}

	for (int i = 0; i < config_.jointCount; ++i)
	{
		// 力矩到电流的分母为 kt * N * eta。
		// 只要任何一项非正，说明执行器参数配置存在错误，不能继续下发命令。
		const double denom =
			config_.torqueConstant[i] * config_.gearRatio[i] * config_.transmissionEfficiency[i];
		if (denom <= 0.0)
		{
			return HIC_STATUS_ERROR_INVALID_PARAM;
		}

		double current = jointTorqueCommand[i] / denom;
		if (config_.enableCurrentLimit)
		{
			// 电流限制放在换算后执行，能直接对最终驱动器量做约束。
			current = clampToConfiguredRange(
				current,
				config_.lowerMotorCurrent[i],
				config_.upperMotorCurrent[i],
				config_.maxJointCurrent[i]);
		}
		motorCurrentCommand[i] = current;
	}
	return HIC_STATUS_OK;
}

// ============================================================
// 10. 通用小工具
// ============================================================

void HicControlCoordinator::clearCurrentCommand(double* motorCurrentCommand) const
{
	if (!motorCurrentCommand)
	{
		return;
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		motorCurrentCommand[i] = 0.0;
	}
}

void HicControlCoordinator::clearTorqueCommand(double* jointTorqueCommand) const
{
	if (!jointTorqueCommand)
	{
		return;
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointTorqueCommand[i] = 0.0;
	}
}

void HicControlCoordinator::clearProtectionStatus(bool* jointProtectionStatus) const
{
	if (!jointProtectionStatus)
	{
		return;
	}
	for (int i = 0; i < config_.jointCount; ++i)
	{
		jointProtectionStatus[i] = false;
	}
}

bool HicControlCoordinator::hasNaNOrInf(const double* data, int size) const
{
	for (int i = 0; i < size; ++i)
	{
		if (!std::isfinite(data[i]))
		{
			return true;
		}
	}
	return false;
}

void HicControlCoordinator::invalidateCommandCache()
{
	// 任何影响控制输出的输入变化，都通过版本号递增来表达。
	// 这样 computeCartesianImpedanceTorqueCommand() 就能快速判断缓存是否还能复用。
	++commandInputVersion_;
	commandCacheValid_ = false;
}

} // namespace hic
