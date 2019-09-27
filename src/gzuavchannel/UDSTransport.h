#ifndef UDSTRANSPORT_H
#define UDSTRANSPORT_H

#include "Transport.h"

#include <string>
#include <vector>

class UDSTransport : public Transport
{
	public:
		UDSTransport(const std::string &path, const std::vector<std::string> &localNames);
		~UDSTransport() override;

		int fd() const override;
		void runOnce() override;

		void setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb) override;
		void sendPacket(int uav_num, const void *data, size_t len) override;

	private:
		IO::PollGroup m_pollGrp;
		std::function<void(int uav_num, const void *data, size_t len)> m_recvHandler;
		std::map<int, int> m_num2fd; // uav_num -> fd
};

#endif // UDSTRANSPORT_H
