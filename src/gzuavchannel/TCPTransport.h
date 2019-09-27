#ifndef TCPTRANSPORT_H
#define TCPTRANSPORT_H

#include "Transport.h"

#include <map>
#include <string>
#include <vector>

class TCPTransport : public Transport
{
	public:
		// server mode
		TCPTransport(int listenPort, const std::vector<std::string> &uavNames);

		// client mode
		TCPTransport(const char *connectTarget, const std::vector<std::string> &uavNames);

		~TCPTransport() override;

		int fd() const override;
		void runOnce() override;

		void setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb) override;
		void sendPacket(int uav_num, const void *data, size_t len) override;

	private:
		class TCPConnection : public IO::Pollable
		{
			public:
				TCPConnection(int fd, const std::vector<int> &local2global);
				~TCPConnection() override;

				int fd() const override;
				void runOnce() override;

				void setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb);
				void sendPacket(int uav_num, const void *data, size_t len);

			private:
				int m_fd;
				std::function<void(int uav_num, const void *data, size_t len)> m_recvHandler;

				// UAV ID mapping
				std::vector<int> m_local2global;
				std::map<int, int> m_global2local;

				// Number of messages to wait for before sending a TCP packet (we
				// assume that all UAVs' packets are sent at the same time in both
				// directions -- as is always the case with this framework)
				size_t m_pendingMessagesCountdown;
		};

		IO::PollGroup m_pollGrp;
		std::map<int, TCPConnection*> m_global2conn; // uav_num -> connection
		std::vector<TCPConnection*> m_connections;
};

#endif // TCPTRANSPORT_H
