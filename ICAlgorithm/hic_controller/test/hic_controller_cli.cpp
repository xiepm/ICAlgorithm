#include "hic_dual_process_shared.h"

#include <cerrno>
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

// 这个文件是“控制器侧”的命令行测试程序。
//
// 它在整套双进程测试里的角色是：
// 1. 作为客户端连接 hic_state_generator；
// 2. 不断接收“外部机器人状态”；
// 3. 调用 hic_update_state_estimates 把状态送进控制器；
// 4. 根据用户输入，切换到零力示教或定点阻抗模式；
// 5. 打印当前控制器输出的电流命令、内部状态、末端状态。
//
// 如果把整个测试想象成真实系统：
// - hic_state_generator 像“机器人驱动/底层状态源”
// - hic_controller_cli 像“上层调试终端 + 控制算法测试入口”
//
// 新手第一次读这个文件时，可以抓住一条主线：
// “先收状态 -> 再 update_state -> 再启动模式 -> 再看输出”

// 统一打印 double 数组。
//
// 这个小工具本身很简单，但它承担了大部分可视化输出工作。
// 只要你在终端里看到一串类似：
// jointPosition: 0.1000, 0.2000, ...
// motorCurrentCommand: 0.3000, -0.1000, ...
// 基本都是通过这个函数打印出来的。
//
// 参数说明：
// - name: 这一行数据的名字
// - data: 要打印的数组首地址
// - size: 数组里有多少个元素需要打印
void printVector(const char* name, const double* data, int size)
{
	std::cout << name << ": ";
	for (int i = 0; i < size; ++i)
	{
		std::cout << std::fixed << std::setprecision(4) << data[i]
			  << (i + 1 == size ? "" : ", ");
	}
	std::cout << std::endl;
}

// 统一打印布尔数组。
//
// 在这个 demo 里，bool 数组主要用来表示“每个关节是否触发保护”。
// 为了让输出更紧凑，这里把 true / false 打印成 1 / 0。
//
// 例如：
// jointProtectionStatus: 0, 0, 1, 0, 0, 0
// 表示第 3 个关节当前处于保护状态。
void printBoolVector(const char* name, const bool* data, int size)
{
	std::cout << name << ": ";
	for (int i = 0; i < size; ++i)
	{
		std::cout << (data[i] ? 1 : 0) << (i + 1 == size ? "" : ", ");
	}
	std::cout << std::endl;
}

// 初始化控制器。
//
// 对小白来说，可以把这里理解成“控制器上电后的基础配置步骤”。
// 程序一启动，不会立刻进入零力或阻抗模式，而是先做最基本的准备。
//
// 这里依次完成三件事：
// 1. 创建控制器；
// 2. 下发关节力矩传感器参数；
// 3. 下发阻抗增益参数。
//
// 这三步为什么都放在这里：
// - hic_initialize_control：相当于创建并初始化控制器内部对象
// - hic_set_joint_torque_sensor_parameters：让控制器知道如何理解关节力矩相关参数
// - hic_set_cartesian_impedance_gains：给阻抗模式设置默认增益
//
// 只要这个函数失败，后面的 zero / fixed 基本都不该继续测了。
// 因为那意味着控制器还没有准备好。
int initializeController()
{
	using namespace hic_test;
	const HicInitializeConfig initConfig = makeDefaultInitializeConfig();
	const HicControlConfig runtimeConfig = makeDefaultRuntimeConfig(initConfig);
	int status = hic_initialize_control(&initConfig);
	std::cout << "init status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	status = applyDefaultRuntimeConfig(runtimeConfig);
	std::cout << "apply runtime config status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	HicTorqueSensorConfig torqueConfig = makeTorqueSensorConfig(initConfig);
	status = hic_set_joint_torque_sensor_parameters(&torqueConfig);
	std::cout << "set torque sensor config status: " << status << std::endl;
	if (status != HIC_STATUS_OK)
	{
		return status;
	}

	HicImpedanceGains gains = makeDefaultGains();
	status = hic_set_cartesian_impedance_gains(&gains);
	std::cout << "set impedance gains status: " << status << std::endl;
	return status;
}

// 从状态发生器读取“最新一帧”状态。
//
// 这个函数是“从外部世界拿状态”的入口。
// 注意：控制器自己并不知道机器人现在在哪、速度多大、电流多少，
// 这些都要靠状态发生器通过 socket 发过来。
//
// 状态发生器会不断往 socket 里发数据，如果 CLI 很久没读，
// 缓冲区里可能已经积累了很多旧帧。
//
// 这里采用的策略是：
// 1. 先非阻塞地尽量把可读数据全部读出来；
// 2. 只保留最后一帧，也就是最新状态；
// 3. 如果当前一帧都没有缓存，再阻塞等下一帧。
//
// 为什么不直接拿第一帧就返回？
// 因为第一帧可能已经“过时”了。
// 假设状态发生器每 10ms 发送一次，而 CLI 100ms 才读一次，
// 那缓冲区里就可能堆了 10 帧数据。如果只取最早那一帧，
// 控制器看到的就不是“现在”的状态，而是“100ms 前”的状态。
//
// 所以后续 update_state 时，应该尽量使用最新状态。
//
// 返回值：
// - true: 成功拿到一帧完整状态
// - false: 没拿到有效数据，或者连接异常
bool receiveLatestStatePacket(int socketFd, hic_test::StatePacket* packetOut)
{
	if (!packetOut)
	{
		return false;
	}

	hic_test::StatePacket packet = {};
	bool received = false;
	for (;;)
	{
		const ssize_t bytes = recv(socketFd, &packet, sizeof(packet), MSG_DONTWAIT);
		if (bytes == static_cast<ssize_t>(sizeof(packet)))
		{
			*packetOut = packet;
			received = true;
			continue;
		}
		if (bytes == 0)
		{
			return received;
		}
		if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
		{
			break;
		}
		if (bytes < 0)
		{
			std::perror("recv");
		}
		break;
	}

	if (received)
	{
		return true;
	}

	const ssize_t bytes = recv(socketFd, &packet, sizeof(packet), 0);
	if (bytes != static_cast<ssize_t>(sizeof(packet)))
	{
		if (bytes < 0)
		{
			std::perror("recv");
		}
		return false;
	}
	*packetOut = packet;
	return true;
}

// 向状态发生器发送一条控制命令。
//
// 这个 CLI 除了接收状态，也可以反过来给状态发生器发少量命令。
// 目前只做了两个最基本的控制动作：
//
// 当前 demo 支持两类命令：
// 1. 切换场景，例如 idle / drag / limit / fast
// 2. 请求状态发生器 shutdown
//
// 场景切换的意义是：
// - idle: 基本静止，适合先把流程跑通
// - drag: 缓慢运动，适合看零力示教输出
// - limit: 靠近限位，适合看保护逻辑
// - fast: 速度较大，适合看零力模式拒绝进入
bool sendControlCommand(int socketFd, int type, int scenario)
{
	hic_test::ControlPacket command = {};
	command.type = type;
	command.scenario = scenario;
	const ssize_t bytes = send(socketFd, &command, sizeof(command), MSG_NOSIGNAL);
	return bytes == static_cast<ssize_t>(sizeof(command));
}

// 把收到的状态包转换成 HIC 控制器需要的输入格式，
// 然后调用 hic_update_state_estimates。
//
// 这是整个 demo 的关键动作之一：
// 外部状态只有先通过这里送进控制器，
// 后面的零力示教、定点阻抗才有“当前机器人状态”可用。
//
// 可以把这个过程理解成“喂状态”：
// - packet 是 socket 收到的原始外部状态
// - input 是 HIC C API 规定的输入结构体
// - hic_update_state_estimates 是把这些数据送进控制器内部观察器
//
// 调用成功后，控制器内部就能基于这些输入估算：
// - 关节速度
// - 关节加速度
// - 电机估计力矩
// - 当前笛卡尔状态
int syncControllerState(const hic_test::StatePacket& packet)
{
	HicStateEstimateInput input = {};
	input.jointCount = hic_test::kDemoJointCount;
	for (int i = 0; i < HIC_MAX_JOINTS; ++i)
	{
		input.jointPosition[i] = packet.jointPosition[i] * kRadToDeg;
		input.motorCurrent[i] = packet.motorCurrent[i];
	}
	input.currentTime = packet.currentTime;
	return hic_update_state_estimates(&input);
}

// 打印“控制器内部看到的机器人状态”。
//
// 这里不是原始 socket 数据，而是经过 hic_update_state_estimates
// 之后，控制器内部整理出的状态。
//
// 当你怀疑状态没更新进去，优先看这里。
//
// 常见观察点：
// - jointPosition：当前位置是否合理
// - jointVelocity：速度是不是和场景一致
// - jointAcceleration：有没有被正常估算出来
// - motorCurrent：是否接收到了外部电流
// - motorEstimatedTorque：控制器内部估算的关节力矩
//
// 此外，这里还会尝试打印当前力控模式下“这一拍可获取的目标关节力矩命令”。
// 这个值不是状态发生器发来的，而是控制器按当前模式计算出来、建议外界使用的命令。
// 如果当前不在力控模式里，这里就不会有有效力矩输出。
void printObservedState()
{
	HicRobotState robotState = {};
	const int robotStateStatus = hic_get_robot_state(&robotState);
	std::cout << "robot state status: " << robotStateStatus << std::endl;
	if (robotStateStatus == HIC_STATUS_OK)
	{
		printVector("jointPosition", robotState.jointPosition, hic_test::kDemoJointCount);
		printVector("jointVelocity", robotState.jointVelocity, hic_test::kDemoJointCount);
		printVector("jointAcceleration", robotState.jointAcceleration, hic_test::kDemoJointCount);
		printVector("motorCurrent", robotState.motorCurrent, hic_test::kDemoJointCount);
		printVector("motorEstimatedTorque", robotState.motorEstimatedTorque, hic_test::kDemoJointCount);
	}

	HicActiveControlState activeState = {};
	const int activeStateStatus = hic_get_active_control_state(&activeState);
	if (activeStateStatus == HIC_STATUS_OK &&
	    activeState.forceControlMode != HIC_FORCE_CONTROL_MODE_NONE)
	{
		double jointTorque[HIC_MAX_JOINTS] = { 0.0 };
		bool protection[HIC_MAX_JOINTS] = { false };
		const int torqueStatus = hic_get_force_control_torque_commands(jointTorque, protection);
		std::cout << "force control torque status: " << torqueStatus << std::endl;
		if (torqueStatus == HIC_STATUS_OK || torqueStatus == HIC_STATUS_ERROR_CURRENT_LIMIT)
		{
			printVector("jointTorqueCommand", jointTorque, hic_test::kDemoJointCount);
			printBoolVector("jointTorqueProtectionStatus", protection, hic_test::kDemoJointCount);
		}
	}
	else
	{
		std::cout << "force control torque status: no active force control mode" << std::endl;
	}
}

// 打印最近一次从状态发生器收到的原始状态包。
//
// 建议把这个输出和 printObservedState() 对照着看：
// - packet 是外部输入
// - observed state 是控制器内部感知结果
//
// 这个函数更像“原始输入监视器”。
// 如果你想知道状态发生器到底发了什么，先看这里。
void printPacket(const hic_test::StatePacket& packet)
{
	std::cout << "packet sequence: " << packet.sequence
		  << ", scenario: " << hic_test::scenarioName(packet.scenario)
		  << ", time: " << std::fixed << std::setprecision(3) << packet.currentTime
		  << std::endl;
	printVector("packet jointPosition", packet.jointPosition, hic_test::kDemoJointCount);
	printVector("packet motorCurrent", packet.motorCurrent, hic_test::kDemoJointCount);
}

// 打印当前控制模式下的控制输出。
//
// 这是最核心的观测函数，主要回答三个问题：
// 1. 当前控制器是不是已经进入某个模式；
// 2. 当前模式下输出了什么电流命令；
// 3. 当前机器人状态、力矩状态、末端状态分别是什么。
//
// 这个函数内部会先看当前 control mode：
// - 如果当前 forceControlMode 是 ZERO_FORCE，就说明正在做零力示教
// - 如果当前 forceControlMode 是其它力控模式，就说明正在做阻抗类控制
// - 如果是 IDLE，就说明当前没有活动控制输出
//
// 这里打印的内容很多，但可以按这个顺序理解：
// 1. 当前模式和最近状态码
// 2. 这一拍算出来的电流命令
// 3. 哪些关节触发了保护
// 4. 控制器内部机器人状态
// 5. 当前末端位姿和末端速度
//
// 也就是说，这一个函数基本就是“这一拍控制结果的总览”。
void printModeOutput()
{
	const int mode = hic_get_force_control_mode();
	std::cout << "force control mode: " << mode << ", last status: " << hic_get_last_status() << std::endl;

	double motorCurrentCmd[HIC_MAX_JOINTS] = { 0.0 };
	bool protection[HIC_MAX_JOINTS] = { false };
	HicActiveControlState activeState = {};
	const int activeStateStatus = hic_get_active_control_state(&activeState);

	if (activeStateStatus == HIC_STATUS_OK &&
	    mode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE &&
	    activeState.forceControlMode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
	{
		const int status = hic_get_force_control_current_commands(motorCurrentCmd, protection);
		std::cout << "zero force current status: " << status << std::endl;
		printVector("motorCurrentCommand", motorCurrentCmd, hic_test::kDemoJointCount);
		printBoolVector("jointProtectionStatus", protection, hic_test::kDemoJointCount);
	}
	else if (activeStateStatus == HIC_STATUS_OK &&
		 mode != HIC_FORCE_CONTROL_MODE_NONE &&
		 activeState.forceControlMode != HIC_FORCE_CONTROL_MODE_NONE)
	{
		const int status = hic_get_force_control_current_commands(motorCurrentCmd, protection);
		std::cout << "impedance current status: " << status << std::endl;
		printVector("motorCurrentCommand", motorCurrentCmd, hic_test::kDemoJointCount);
		printBoolVector("jointProtectionStatus", protection, hic_test::kDemoJointCount);
	}
	else
	{
		std::cout << "controller is idle, no active output command" << std::endl;
	}

	printObservedState();

	HicCartesianState cartesianState = {};
	const int cartesianStatus = hic_get_current_cartesian_state(&cartesianState);
	std::cout << "cartesian state status: " << cartesianStatus << std::endl;
	if (cartesianStatus == HIC_STATUS_OK)
	{
		printVector("currentPose", cartesianState.pose, HIC_POSE_DIM);
		printVector("currentTwist", cartesianState.twist, HIC_CARTESIAN_DIM);
	}
}

// 命令行帮助。
//
// 这里不只列命令，还尽量写清楚“什么时候用、预期看到什么”。
// 新手第一次运行时，直接照着底部“推荐测试顺序”操作即可。
//
// 这份帮助对应的典型测试思路是：
// 1. 先确认状态通路正常（state / sync）
// 2. 再测零力示教（zero / run / stop）
// 3. 再测定点阻抗（fixed / run）
// 4. 最后切到 limit / fast 等特殊场景看保护行为
void printHelp()
{
	std::cout
		<< "commands:\n"
		<< "  help                     显示帮助\n"
		<< "  scenario idle|drag|limit|fast\n"
		<< "                           切换状态发生器场景：\n"
		<< "                           idle  = 基本静止，适合首次测试\n"
		<< "                           drag  = 缓慢拖动，适合看零力示教响应\n"
		<< "                           limit = 靠近限位，适合看限位保护\n"
		<< "                           fast  = 速度较大，适合看零力进入拒绝\n"
		<< "  sync                     接收一帧最新状态，并调用 hic_update_state_estimates\n"
		<< "  zero                     先 sync，再尝试进入零力示教模式\n"
		<< "  fixed_pose               先 sync，抓取当前位姿，再进入定点位姿保持模式\n"
		<< "  fixed_pos                先 sync，抓取当前位置，再进入定点位置保持模式\n"
		<< "  fixed                    等价于 fixed_pose，保留作简短命令\n"
		<< "  step                     再同步一帧状态，并打印当前控制输出\n"
		<< "  run N                    连续执行 N 次 step，例如 run 10\n"
		<< "  stop                     请求当前模式平滑退出\n"
		<< "  state                    打印最新原始状态包 + 控制器内部机器人状态\n"
		<< "  reset                    调用 hic_reset_control，回到干净状态\n"
		<< "  shutdown                 请求状态发生器退出\n"
		<< "  quit                     退出当前 CLI\n"
		<< "\n"
		<< "推荐测试顺序：\n"
		<< "  1. state\n"
		<< "  2. sync\n"
		<< "  3. zero\n"
		<< "  4. run 10\n"
		<< "  5. stop\n"
		<< "  6. run 10\n"
		<< "  7. reset\n"
		<< "  8. fixed_pose\n"
		<< "  9. run 10\n";
}
}

int main()
{
	using namespace hic_test;

	// 先作为 socket client 连接状态发生器。
	//
	// 这个 CLI 自己不产生机器人状态，它只负责：
	// 1. 接收状态发生器发送过来的关节状态；
	// 2. 调用 HIC C API 更新控制器内部状态；
	// 3. 根据用户命令进入零力示教或定点阻抗；
	// 4. 打印电流、力矩、状态信息。
	//
	// 所以使用顺序必须是：
	// 1. 先启动 hic_state_generator
	// 2. 再启动 hic_controller_cli
	//
	// 如果这一步 connect 失败，通常就是：
	// - 状态发生器还没启动
	// - 端口号不一致
	// - 本机 socket 环境有问题
	const int socketFd = socket(AF_INET, SOCK_STREAM, 0);
	if (socketFd < 0)
	{
		std::perror("socket");
		return 1;
	}

	sockaddr_in address = {};
	address.sin_family = AF_INET;
	address.sin_port = htons(kStateSocketPort);
	if (inet_pton(AF_INET, kStateSocketHost, &address.sin_addr) != 1)
	{
		std::cerr << "invalid socket host: " << kStateSocketHost << std::endl;
		close(socketFd);
		return 1;
	}
	if (connect(socketFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
	{
		std::cerr << "failed to connect " << kStateSocketHost << ":" << kStateSocketPort
			  << ", please start hic_state_generator first: " << std::strerror(errno) << std::endl;
		close(socketFd);
		return 1;
	}

	// 初始化控制器。
	//
	// 到这里为止，socket 已经连通了，但控制器本身还没准备好。
	// initializeController() 成功之后，才算“通信和控制器两边都就绪”。
	const int initStatus = initializeController();
	if (initStatus != HIC_STATUS_OK)
	{
		close(socketFd);
		return initStatus;
	}

	// 一进入 CLI 就先打印帮助，方便第一次使用的人直接照着命令测试。
	printHelp();

	// lastPacket 保存最近一次收到的原始状态包。
	// 可以把它理解成“CLI 当前手里最新的一份外部机器人状态快照”。
	//
	// hasPacket 用来标记“我们是否至少收到过一帧状态”。
	// 这样执行 state 命令时，如果还没收到任何数据，就不会误打印空包。
	StatePacket lastPacket = {};
	bool hasPacket = false;

	std::string line;
	while (true)
	{
		// 主命令循环。
		//
		// 整个 CLI 的使用方式很朴素：
		// - 用户在终端输入一行命令
		// - 程序解析这行文字
		// - 执行对应动作
		// - 打印结果
		//
		// 所以你可以把下面这一大串 if 分支理解成：
		// “一个手工写的命令解释器”。
		std::cout << "\nhic-cli> " << std::flush;
		if (!std::getline(std::cin, line))
		{
			break;
		}

		std::istringstream iss(line);
		std::string command;
		iss >> command;
		if (command.empty())
		{
			continue;
		}
		if (command == "help")
		{
			printHelp();
			continue;
		}
		if (command == "scenario")
		{
			// 切换状态发生器场景。
			//
			// 这一步不会直接改变控制模式，
			// 它只是告诉“外部状态源”改成另一种运动/边界场景。
			//
			// 也就是说，scenario 改的是“机器人状态如何变化”，
			// 而不是“控制器如何控制机器人”。
			// 两者要区分开：
			// - scenario 是外部输入场景
			// - zero / fixed 是控制模式
			std::string name;
			iss >> name;
			const int scenario = scenarioFromString(name);
			if (scenario < 0)
			{
				std::cout << "unknown scenario: " << name << std::endl;
				continue;
			}
			if (!sendControlCommand(socketFd, CONTROL_COMMAND_SET_SCENARIO, scenario))
			{
				std::cout << "failed to send scenario command" << std::endl;
				break;
			}
			std::cout << "scenario switch requested: " << scenarioName(scenario) << std::endl;
			continue;
		}

		// 这些命令都依赖“最新状态”：
		// - sync: 更新控制器状态
		// - zero/fixed: 模式启动前必须基于最新状态
		// - step/run: 计算输出前必须先更新状态
		// - state: 需要打印最新原始包
		//
		// 所以这里把“先收一帧状态”的逻辑统一前置了，避免每个分支重复写。
		if ((command == "sync" || command == "zero" || command == "fixed" ||
		     command == "step" || command == "run" || command == "state"))
		{
			if (!receiveLatestStatePacket(socketFd, &lastPacket))
			{
				std::cout << "failed to receive state packet" << std::endl;
				break;
			}
			hasPacket = true;
		}

		if (command == "sync")
		{
			// 最基础的动作：把最近一帧外部状态喂给控制器。
			//
			// 如果这个步骤都失败，那么 zero / fixed / step 的结果通常也不可信。
			//
			// 对新手来说，这个命令可以理解成：
			// “只更新状态，不做模式切换，先确认控制器已经看到机器人了。”
			const int status = syncControllerState(lastPacket);
			std::cout << "sync status: " << status << std::endl;
			continue;
		}
		if (command == "zero")
		{
			// 零力示教标准入口：
			// 1. 先同步状态；
			// 2. 再调用 hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_ZERO_FORCE)。
			//
			// 如果进入失败，通常是安全检查没通过，比如：
			// - 当前速度过大
			// - 状态未准备好
			// - 靠近硬限位
			//
			// 如果进入成功，真正的电流命令并不是在这里长期循环输出，
			// 而是在后面的 step / run 里，每次再调用
			// hic_get_force_control_current_commands() 计算出来。
			int status = syncControllerState(lastPacket);
			std::cout << "sync status: " << status << std::endl;
			if (status == HIC_STATUS_OK)
			{
				status = hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_ZERO_FORCE);
				std::cout << "start zero force status: " << status << std::endl;
			}
			printModeOutput();
			continue;
		}
		if (command == "fixed" || command == "fixed_pose")
		{
			// 定点阻抗标准入口：
			// 1. 先同步状态；
			// 2. 抓取当前位姿作为“固定目标”；
			// 3. 启动笛卡尔阻抗模式。
			//
			// 这也是当前 demo 里“定点模式”的主要测试入口。
			//
			// 为什么要先 capture 当前位姿？
			// 因为“定点阻抗”的意思是：以某个目标位姿为中心产生恢复力。
			// 如果没有先记录这个目标位姿，控制器就不知道“要定在哪儿”。
			int status = syncControllerState(lastPacket);
			std::cout << "sync status: " << status << std::endl;
			if (status == HIC_STATUS_OK)
			{
				status = hic_capture_current_pose_as_fixed_target();
				std::cout << "capture fixed pose target status: " << status << std::endl;
			}
			if (status == HIC_STATUS_OK)
			{
				status = hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSE);
				std::cout << "start fixed pose mode status: " << status << std::endl;
			}
			printModeOutput();
			continue;
		}
		if (command == "fixed_pos")
		{
			int status = syncControllerState(lastPacket);
			std::cout << "sync status: " << status << std::endl;
			if (status == HIC_STATUS_OK)
			{
				status = hic_capture_current_position_as_fixed_target();
				std::cout << "capture fixed position target status: " << status << std::endl;
			}
			if (status == HIC_STATUS_OK)
			{
				status = hic_start_force_control_mode(HIC_FORCE_CONTROL_MODE_CARTESIAN_FIXED_POSITION);
				std::cout << "start fixed position mode status: " << status << std::endl;
			}
			printModeOutput();
			continue;
		}
		if (command == "step")
		{
			// 单步推进一次：
			// - 先同步一帧最新状态
			// - 再打印当前模式下的输出
			//
			// 当你想慢慢看每一步发生了什么，step 比 run 更适合。
			// 它很适合排查“这一拍为什么输出了这个电流值”。
			const int status = syncControllerState(lastPacket);
			std::cout << "sync status: " << status << std::endl;
			printModeOutput();
			continue;
		}
		if (command == "run")
		{
			// 连续推进多步。
			//
			// 典型用途：
			// - 观察零力示教在连续几个周期内的输出变化
			// - 观察 stop 之后是否平滑退出
			// - 观察定点阻抗模式是否稳定维持
			//
			// 这里每一步都会：
			// 1. 再收一帧最新状态
			// 2. 再 update_state
			// 3. 再打印这一步的控制输出
			//
			// 所以 run 10 本质上就是把 step 连做 10 次。
			int steps = 1;
			iss >> steps;
			if (steps <= 0)
			{
				steps = 1;
			}
			for (int i = 0; i < steps; ++i)
			{
				if (!receiveLatestStatePacket(socketFd, &lastPacket))
				{
					std::cout << "failed to receive state packet" << std::endl;
					close(socketFd);
					return 1;
				}
				hasPacket = true;
				const int status = syncControllerState(lastPacket);
				std::cout << "\nrun step " << i << " sync status: " << status << std::endl;
				printModeOutput();
			}
			continue;
		}
		if (command == "stop")
		{
			// 请求当前模式退出。
			//
			// 注意：
			// - 零力模式是“准备退出”，不一定 stop 一下就立刻 idle；
			// - 常见做法是 stop 之后再 run 几步，观察是否完成平滑退出。
			//
			// 这和很多实时控制系统的思路一致：
			// “先发退出请求，再让控制器在后续几个周期里平稳收尾”。
			const int mode = hic_get_force_control_mode();
			HicActiveControlState activeState = {};
			const int activeStateStatus = hic_get_active_control_state(&activeState);
			if (activeStateStatus == HIC_STATUS_OK &&
			    mode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE &&
			    activeState.forceControlMode == HIC_FORCE_CONTROL_MODE_ZERO_FORCE)
			{
				std::cout << "prepare stop zero force status: "
					  << hic_prepare_stop_force_control_mode() << std::endl;
			}
			else if (activeStateStatus == HIC_STATUS_OK &&
				 mode != HIC_FORCE_CONTROL_MODE_NONE &&
				 activeState.forceControlMode != HIC_FORCE_CONTROL_MODE_NONE)
			{
				std::cout << "prepare stop impedance status: "
					  << hic_prepare_stop_force_control_mode() << std::endl;
			}
			else
			{
				std::cout << "controller already idle" << std::endl;
			}
			continue;
		}
		if (command == "state")
		{
			// 打印最近原始输入 + 控制器内部状态。
			//
			// 这个命令最适合排查：
			// - 状态发生器有没有正常发数据
			// - hic_update_state_estimates 之后内部状态是否合理
			//
			// 如果 packet 正常、但 robot state 不正常，
			// 问题大概率出在 update_state 链路或内部状态观察器。
			if (hasPacket)
			{
				printPacket(lastPacket);
			}
			printObservedState();
			continue;
		}
		if (command == "reset")
		{
			// 重置控制器状态。
			//
			// 如果前面的测试流程已经切过很多模式，或者你想从干净状态重来，
			// 先 reset 再继续是最稳妥的。
			//
			// 一个很常见的使用习惯是：
			// “测完 zero 之后 reset，再开始 fixed 的测试”。
			std::cout << "reset status: " << hic_reset_control() << std::endl;
			continue;
		}
		if (command == "shutdown")
		{
			// 请求状态发生器进程退出。
			//
			// 这个命令通常放在整套测试结束时再执行。
			// 执行后，对端的 hic_state_generator 会收到退出请求。
			if (!sendControlCommand(socketFd, CONTROL_COMMAND_SHUTDOWN, 0))
			{
				std::cout << "failed to send shutdown command" << std::endl;
			}
			else
			{
				std::cout << "generator shutdown requested" << std::endl;
			}
			continue;
		}
		if (command == "quit" || command == "exit")
		{
			// 只退出 CLI 本身，不强制关闭状态发生器。
			// 如果希望对端也结束，请先执行 shutdown。
			break;
		}

		// 如果命令没匹配上任何已知分支，就走到这里。
		// 这是一个很普通的“未知命令”提示。
		std::cout << "unknown command: " << command << std::endl;
	}

	// 离开主循环后，关闭 socket，正常退出。
	close(socketFd);
	return 0;
}
