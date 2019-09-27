#include "UDSTransport.h"

#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <algorithm>
#include <set>

UDSTransport::UDSTransport(const std::string &path, const std::vector<std::string> &localNames)
{
	int serv_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);

	struct sockaddr_un addr;
	memset(&addr, 0, sizeof(addr));
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

	if (bind(serv_fd, (struct sockaddr*)&addr, sizeof(addr)) == -1)
		err(EXIT_FAILURE, "UDS: bind(%s) failed", addr.sun_path);

	listen(serv_fd, localNames.size());

	std::set<std::string> notYetConnectedNames(localNames.begin(), localNames.end());

	// block until all UAVs are connected
	while (!notYetConnectedNames.empty())
	{
		int uav_fd = accept(serv_fd, nullptr, nullptr);

		char name[256];
		int name_len = recv(uav_fd, name, sizeof(name), 0);

		if (name_len <= 0 || name_len >= sizeof(name))
			errx(EXIT_FAILURE, "failed to receive UAV name");

		name[name_len] = '\0';

		if (notYetConnectedNames.erase(name) != 1)
			errx(EXIT_FAILURE, "UDS: received unexpected UAV name: %s", name);

		int idx = std::find(localNames.begin(), localNames.end(), name) - localNames.begin();
		warnx("UDS: UAV #%d is connected %s on socket %d", idx, name, uav_fd);

		m_num2fd.emplace(idx, uav_fd);
		m_pollGrp.add(uav_fd, [this, idx, uav_fd]()
		{
			char data[65536];
			int r = recv(uav_fd, data, sizeof(data), 0);

			if (r <= 0)
				err(EXIT_FAILURE, "UDS: UAV #%d read error", idx);

			if (m_recvHandler)
				m_recvHandler(idx, data, r);
		});
	}

	warnx("UDS: all UAVs are connected");
	close(serv_fd);
}

UDSTransport::~UDSTransport()
{
	// TODO: close connections
}

int UDSTransport::fd() const
{
	return m_pollGrp.fd();
}

void UDSTransport::runOnce()
{
	m_pollGrp.runOnce();
}

void UDSTransport::setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb)
{
	m_recvHandler = cb;
}

void UDSTransport::sendPacket(int uav_num, const void *data, size_t len)
{
	send(m_num2fd.at(uav_num), data, len, 0);
}
