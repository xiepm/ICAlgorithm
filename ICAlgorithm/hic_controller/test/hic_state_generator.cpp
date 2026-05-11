#include "hic_dual_process_shared.h"

#include <cerrno>
#include <csignal>
#include <arpa/inet.h>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/socket.h>

namespace
{
volatile std::sig_atomic_t g_keepRunning = 1;

void handleSignal(int)
{
	g_keepRunning = 0;
}
}

int main(int argc, char** argv)
{
	using namespace hic_test;

	int scenario = SIM_SCENARIO_IDLE;
	if (argc >= 2)
	{
		const int parsedScenario = scenarioFromString(argv[1]);
		if (parsedScenario >= 0)
		{
			scenario = parsedScenario;
		}
	}

	std::signal(SIGINT, handleSignal);
	std::signal(SIGTERM, handleSignal);

	const int serverFd = socket(AF_INET, SOCK_STREAM, 0);
	if (serverFd < 0)
	{
		std::perror("socket");
		return 1;
	}

	int reuseAddr = 1;
	setsockopt(serverFd, SOL_SOCKET, SO_REUSEADDR, &reuseAddr, sizeof(reuseAddr));

	sockaddr_in address = {};
	address.sin_family = AF_INET;
	address.sin_port = htons(kStateSocketPort);
	address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

	if (bind(serverFd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
	{
		std::perror("bind");
		close(serverFd);
		return 1;
	}

	if (listen(serverFd, 1) != 0)
	{
		std::perror("listen");
		close(serverFd);
		return 1;
	}

	std::cout << "state generator started, socket="
		  << kStateSocketHost << ":" << kStateSocketPort
		  << ", scenario=" << scenarioName(scenario) << std::endl;

	std::uint64_t sequence = 0;
	double timeSec = 0.0;

	while (g_keepRunning)
	{
		const int clientFd = accept(serverFd, nullptr, nullptr);
		if (clientFd < 0)
		{
			if (errno == EINTR)
			{
				continue;
			}
			std::perror("accept");
			break;
		}

		std::cout << "controller connected" << std::endl;
		while (g_keepRunning)
		{
			ControlPacket command = {};
			const ssize_t commandBytes = recv(clientFd, &command, sizeof(command), MSG_DONTWAIT);
			if (commandBytes == static_cast<ssize_t>(sizeof(command)))
			{
				if (command.type == CONTROL_COMMAND_SET_SCENARIO)
				{
					scenario = command.scenario;
					std::cout << "scenario switched to " << scenarioName(scenario) << std::endl;
				}
				else if (command.type == CONTROL_COMMAND_SHUTDOWN)
				{
					g_keepRunning = 0;
					break;
				}
			}
			else if (commandBytes == 0)
			{
				std::cout << "controller disconnected" << std::endl;
				break;
			}
			else if (commandBytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
			{
				std::perror("recv");
				break;
			}

			StatePacket packet = {};
			packet.sequence = sequence++;
			packet.scenario = scenario;
			packet.currentTime = timeSec;
			generateScenarioState(scenario, timeSec, packet.jointPosition, packet.motorCurrent);

			const ssize_t sentBytes = send(clientFd, &packet, sizeof(packet), MSG_NOSIGNAL);
			if (sentBytes != static_cast<ssize_t>(sizeof(packet)))
			{
				if (sentBytes < 0)
				{
					std::perror("send");
				}
				std::cout << "controller disconnected" << std::endl;
				break;
			}

			timeSec += kControlPeriodSec;
			usleep(static_cast<useconds_t>(kControlPeriodSec * 1.0e6));
		}

		close(clientFd);
	}

	close(serverFd);
	std::cout << "state generator stopped" << std::endl;
	return 0;
}
