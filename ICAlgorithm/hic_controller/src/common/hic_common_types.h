#ifndef HIC_COMMON_TYPES_H_
#define HIC_COMMON_TYPES_H_

#include <string>
#include <vector>
#include "hic_frames.hpp"
#include <Eigen/Eigen>

#pragma warning( disable : 4290 )
#define  Max_ActualAxisCnt 6

#define			        NUMOFJOINTS3	                    3  // 5轴机械臂
#define			        NUMOFJOINTS5	                    5  // 5轴机械臂#define			        NUMOFJOINTS5	                    5  // 5轴机械臂
#define			        NUMOFJOINTS6	                    6  // 6轴机械臂
#define					NUMOFJOINTS7						7  // 7轴机械臂
#define					NUMOFSOLVED8				        8  // 8组臂型解 
#define					NUMOFSOLVED4				        4  // 4组臂型解（针对5DOF机械臂）
#define			        DIMOFTASKSPACE6	                    6  // 6维任务空间
#define			        DIMOFTASKSPACE3                     3  // 3维任务空间

typedef bool                    EcBoolean;
typedef double					EcReal;

///  unsigned integer type used to represent the sizes of containers
typedef std::size_t                       EcSizeT;
typedef unsigned int					  EcU32;

///  similar to EcSizeT but for strings
typedef std::string::size_type            EcStringSizeT;

/// a general floating-point vector
typedef std::vector<EcReal>               EcRealVector;

typedef std::vector<EcU32>				  EcU32Vector;

/// a general boolean vector
typedef std::vector<EcBoolean>            EcBooleanVector;

/// a general 2D boolean vector
typedef std::vector<EcBooleanVector>      EcBooleanVectorVector;

/// a general floating-point array
typedef std::vector<EcRealVector>         EcRealVectorVector;

/// a general floating-point 3D array
typedef std::vector<EcRealVectorVector>   EcRealVector3D;

typedef	KDL::Frame						  EcFrame;
typedef KDL::Vector						  EcVector;
typedef KDL::Rotation					  EcRotation;
typedef std::vector<KDL::Frame>			  EcFrameVector;

typedef Eigen::MatrixXd					  EcRealMatrixX;
typedef Eigen::VectorXd					  EcRealVectorX;
typedef Eigen::Quaternionf				  EcQuaternion;

typedef std::vector<EcVector>			  EcVectorVector;

typedef enum EN_inverseKineState
{
	ikState_normal = 0,
	ikState_outofLimit,			// immediately stop motio；
	ikState_noSolution,			// need to switch to emergency stop plan;
	ikState_notContinuous,		// need to switch to emergency stop plan;
}ENInverseKineState;

typedef struct FullDHParams {
	EcReal theta[10];
	EcReal k[10];
	EcReal d[10];
	EcReal a[10];
	EcReal alpha[10];
	EcReal beta[10];
}DHParams;

typedef struct EN_PcsElem
{
	/// @brief 笛卡尔位置与欧拉角。
	/// @note 该旧兼容类型通常面向外部工程/老接口使用，历史上常按
	/// x/y/z 使用 mm，rx/ry/rz 使用 deg 传递。
	/// @note 若与新的 hic_c_api 交互，建议在边界层转换为：
	/// 位置 m，角度 rad。
	EcReal x, y, z, rx, ry, rz;
	EN_PcsElem()
	{
		x = 0;
		y = 0;
		z = 0;
		rx = 0;
		ry = 0;
		rz = 0;
	}

	EN_PcsElem& operator=(const EN_PcsElem& info)
	{
		copy(this, info);
		return *this;
	}

private:
	void copy(EN_PcsElem* pThis, const EN_PcsElem& info)
	{
		pThis->x = info.x;
		pThis->y = info.y;
		pThis->z = info.z;
		pThis->rx = info.rx;
		pThis->ry = info.ry;
		pThis->rz = info.rz;
	}
}EN_PcsElem;

typedef struct EN_AcsElem{
	/// @brief 关节角。
	/// @note 该旧兼容类型在外部老接口里通常按 deg 使用；
	/// 若送入新的 hic_c_api，应先转换为 rad。
	EcReal j1, j2, j3, j4, j5, j6;
}EN_AcsElem;

typedef enum EN_FrameType
{
	enFrameType_Global,
	enFrameType_Local
}ENFrameType;

typedef enum EN_TrayMode
{
	enTrayMode_Line=0,
	enTrayMode_Square,
	enTrayMode_Box,
	enTrayMode_List
}EN_TrayMode;

typedef struct EN_RobotDH{
	EcReal theta[10];
	EcReal d[10];
	EcReal a[10];
	EcReal alpha[10];
}RobotDH;

// 运动状态：用于判断目标运动是否已经开始、正在进行、成功或失败。
typedef enum EN_motionState
{
	enMotionState_NotStarted = 0,
	enMotionState_InProgress,
	enMotionState_Succeeded,
	enMotionState_Failed,
}ENmotionState;

typedef enum EN_MoveException
{
	enMoveException_NO_EXCEPTION,
	enMoveException_COLLISION,
	enMoveException_JOINT_LIMIT,
	enMoveException_SINGULARITY,
	enMoveException_GENERAL_STOPPING_CRITERION
}ENMoveException;

// 轨迹速度规划类型：S 曲线或梯形规划。
typedef enum EN_profileType
{
	enProfileType_Scurve = 0,
	enProfileType_Trape
}ENProfileType;

/// @brief 阻抗模式运行状态。
/// 该枚举同时被算法主类和对外 C 接口复用，避免状态定义散落在不同模块中。
typedef enum EN_impedanceState
{
	impedance_normal = 0,
	impedance_errorInitParams,
	impedance_errorRobotState,
	impedance_overJointsRangeLimit,
	impedance_overJointsCurrent,
	impedance_singularity,
	impedance_targetJumpTooLarge,
	impedance_finishedReadyToClose
}ENImpedanceState;

/// @brief 阻抗控制目标生成模式。
/// fixed: 固定点阻抗；
/// trajectory: 在线轨迹点跟踪。
typedef enum EN_impedanceTargetMode
{
	impedance_target_fixed = 0,
	impedance_target_trajectory = 1
}ENImpedanceTargetMode;

typedef std::vector<EN_PcsElem>			  EcPcsVector;

#define EC_FALSE            0
#define EC_TRUE             1
#define EC_NULL             0

#endif
