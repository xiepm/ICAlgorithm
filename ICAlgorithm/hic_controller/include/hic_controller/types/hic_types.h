#pragma once

/// @brief HIC 库统一状态码。
/// 所有 C API 返回值和核心类接口都统一复用该枚举，便于上层做一致的错误处理。
/// 当前版本新增的状态观测器和扭矩传感器配置接口也复用这一组状态码，
/// 其中：
/// - HIC_STATUS_ERROR_ROBOT_STATE 用于状态无效、NaN/Inf 或范围异常；
/// - HIC_STATUS_ERROR_INVALID_PARAM 用于配置或传感器标定参数非法；
/// - HIC_STATUS_ERROR_NOT_IMPLEMENTED 用于 JSON/JSONC 配置加载等预留接口。
typedef enum HicStatus
{
	/// @brief 执行成功。
	/// @note 表示本次接口调用完成且输出结果有效。
	HIC_STATUS_OK = 0,
	/// @brief 初始化错误。
	/// @note 通常表示对象尚未 initialize，或所依赖子模块还未完成初始化。
	HIC_STATUS_ERROR_INIT = 1,
	/// @brief 输入参数非法。
	/// @note 典型场景包括：关节数不合法、控制周期非正、执行器参数非正、枚举值越界等。
	HIC_STATUS_ERROR_INVALID_PARAM = 2,
	/// @brief 空指针错误。
	/// @note 说明调用方传入了必需但为空的数组或结构体指针。
	HIC_STATUS_ERROR_NULL_POINTER = 3,
	/// @brief 机器人状态错误。
	/// @note 用于描述状态观测结果无效，例如：
	/// 1. jointPosition / jointVelocity / motorCurrent 出现 NaN 或 Inf；
	/// 2. 状态超出配置范围；
	/// 3. 当前状态不足以支撑安全控制输出。
	HIC_STATUS_ERROR_ROBOT_STATE = 4,
	/// @brief 运动学计算错误。
	/// @note 例如正运动学、Jacobian 或末端 twist 计算失败。
	HIC_STATUS_ERROR_KINEMATICS = 5,
	/// @brief 动力学计算错误。
	/// @note 例如重力补偿、质量矩阵、科氏项等模型求解失败。
	HIC_STATUS_ERROR_DYNAMICS = 6,
	/// @brief 奇异位形或数值退化错误。
	/// @note 常用于 Jacobian 退化、伪逆求解不稳定等场景。
	HIC_STATUS_ERROR_SINGULARITY = 7,
	/// @brief 电流或力矩进入限幅保护。
	/// @note 该状态通常不是“系统不可继续运行”的致命错误，
	/// 而是表示本周期输出被安全策略夹紧了。
	HIC_STATUS_ERROR_CURRENT_LIMIT = 8,
	/// @brief 关节位置触发限位保护。
	/// @note 常见于：
	/// 1. 当前关节位置已经超出硬限位；
	/// 2. ZeroForceMode 进入时发现位置过于靠近硬限位保护区。
	HIC_STATUS_ERROR_JOINT_LIMIT = 9,
	/// @brief 预留接口尚未实现。
	/// @note 用于保留 ABI/接口形态，但当前版本还没有接上完整实现的功能。
	HIC_STATUS_ERROR_NOT_IMPLEMENTED = 10
} HicStatus;

/// @brief HIC 顶层控制模式。
/// 说明：
/// 1. 这里描述的是“控制器当前属于哪一大类主模式”；
/// 2. 当前顶层只区分：空闲 / 力控；
/// 3. 零力示教、定点、轨迹笛卡尔阻抗都统一归入力控模式；
/// 4. 更细的差异由 HicForceControlMode 表达。
typedef enum HicControlMode
{
	/// @brief 空闲模式。
	/// @note 当前没有活动控制输出。
	HIC_CONTROL_MODE_IDLE = 0,
	/// @brief 力控模式。
	/// @note 当前工程中的零力示教、定点阻抗、轨迹阻抗都属于这一类。
	HIC_CONTROL_MODE_FORCE_CONTROL = 1
} HicControlMode;

/// @brief 力控模式下的细粒度模式。
/// @note 这是上层状态机最适合直接使用的一组模式值。
/// @note 当前版本把“做什么控制任务”直接体现在这一层：
/// 1. 零力示教
/// 2. 定点位置保持
/// 3. 定点位姿保持
/// 4. 轨迹笛卡尔阻抗
typedef enum HicForceControlMode
{
	/// @brief 当前没有活动控制输出。
	HIC_FORCE_CONTROL_MODE_NONE = 0,
	/// @brief 零力示教模式。
	/// @note 当前版本第一版以重力补偿 + 简单阻尼为主，摩擦补偿预留后续扩展。
	HIC_FORCE_CONTROL_MODE_ZERO_FORCE = 1,
	/// @brief 定点位置保持模式。
	/// @note 只约束末端位置 XYZ，不约束末端姿态；允许在冗余自由度上做零空间控制。
	HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION = 2,
	/// @brief 定点位姿保持模式。
	/// @note 同时约束末端位置和姿态；在冗余机械臂上仍允许叠加零空间控制。
	HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE = 3,
	/// @brief 轨迹笛卡尔阻抗模式。
	/// @note 目标位姿和目标速度可以随时间在线更新。
	HIC_FORCE_CONTROL_MODE_CARTESIAN_TRAJECTORY = 4
} HicForceControlMode;

/// @brief 关节命令输出环路类型。
/// @note 这里表达的是“上层当前想取哪一种执行器侧命令形式”，
/// 而不是内部控制算法属于哪一类。
typedef enum HicCommandLoopType
{
	/// @brief 电流环命令。
	/// @note 典型用于电机驱动器直接接收电流指令的场景。
	HIC_COMMAND_LOOP_CURRENT = 0,
	/// @brief 力矩环命令。
	/// @note 典型用于仿真、调试或执行器支持直接关节力矩输入的场景。
	HIC_COMMAND_LOOP_TORQUE = 1
} HicCommandLoopType;

/// @brief HIC 对外 C ABI 采用固定容量数组以保持接口稳定。
/// @note HIC_MAX_JOINTS 表示关节数组的固定容量上限，不代表当前机器人一定有 16 个关节；
/// @note 当前机器人实际关节数由 HicInitializeConfig::jointCount 或 HicControlConfig::jointCount 决定；
/// @note 例如 7 轴机械臂只使用数组前 7 个元素，其余元素为预留；
/// @note 固定容量数组用于保持 C 接口稳定，并避免实时控制周期中的动态内存分配。
typedef enum HicDimensions
{
	/// @brief 关节相关数组的固定容量上限。
	/// @note 所有 jointPosition / jointVelocity / motorCurrent 等数组都按这个长度预留。
	HIC_MAX_JOINTS = 16,
	/// @brief 笛卡尔速度/力空间维度。
	/// @note 顺序通常为 [x, y, z, rx, ry, rz] 或 [vx, vy, vz, wx, wy, wz]。
	HIC_CARTESIAN_DIM = 6,
	/// @brief 位姿数组维度。
	/// @note 当前约定 pose 格式为 [px, py, pz, qw, qx, qy, qz]。
	HIC_POSE_DIM = 7,
	/// @brief 每个关节对应的动力学参数个数。
	/// @note 用于把线性动力学参数按“关节数 × 每关节参数数”展开存储。
	HIC_DYNAMIC_PARAM_PER_JOINT = 13,
	/// @brief 动力学参数数组最大长度。
	/// @note 等于 HIC_MAX_JOINTS * HIC_DYNAMIC_PARAM_PER_JOINT。
	HIC_MAX_DYNAMIC_PARAMS = HIC_MAX_JOINTS * HIC_DYNAMIC_PARAM_PER_JOINT,
	/// @brief Jacobian 扁平数组最大长度。
	/// @note 当前按 6 x HIC_MAX_JOINTS 的 row-major 形式预留。
	HIC_MAX_JACOBIAN_SIZE = HIC_CARTESIAN_DIM * HIC_MAX_JOINTS
} HicDimensions;

#ifdef __cplusplus
namespace hic
{
/// @brief 在 C++ 命名空间下重新导出 C 枚举类型。
/// @note 这样内部 C++ 代码既可以保持 `hic::` 命名空间风格，
/// 也不会破坏对外 C ABI 的兼容性。
using ::HicStatus;
using ::HicControlMode;
using ::HicForceControlMode;
using ::HicCommandLoopType;
using ::HicDimensions;
}
#endif
