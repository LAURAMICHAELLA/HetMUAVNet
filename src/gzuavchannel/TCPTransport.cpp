#include "TCPTransport.h"

#include <arpa/inet.h>
#include <err.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>

#include <algorithm>
#include <set>

static bool isValidPort(int n)
{
	if (n < 1 || n > 65535)
		return false;
	else
		return true;
}

static void recvAll(int fd, void *buf, size_t len)
{
	if (recv(fd, buf, len, MSG_WAITALL) != len)
		err(EXIT_FAILURE, "TCP: stream ended unexpectedly");
}

static void sendAll(int fd, const void *buf, size_t len, bool haveMoreToSend = false)
{
	while (len != 0)
	{
		int r = send(fd, buf, len, haveMoreToSend ? MSG_MORE : 0);
		if (r <= 0)
			err(EXIT_FAILURE, "TCP: send failed");

		buf = r + (const char*)buf;
		len -= r;
	}
}

static uint16_t recv16(int fd)
{
	uint16_t tmp;
	recvAll(fd, &tmp, 2);
	return ntohs(tmp);
}

static void send16(int fd, uint16_t val, bool haveMoreToSend = false)
{
	uint16_t tmp = htons(val);
	sendAll(fd, &tmp, 2, haveMoreToSend);
}

TCPTransport::TCPTransport(int listenPort, const std::vector<std::string> &globalUavNames)
{
	if (!isValidPort(listenPort))
		errx(EXIT_FAILURE, "TCP: port number has invalid format or value");

	int serv_fd = socket(AF_INET, SOCK_STREAM, 0);

	int optval = 1;
	setsockopt(serv_fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(listenPort);

	if (bind(serv_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
		err(EXIT_FAILURE, "TCP: bind(%d) failed", listenPort);

	listen(serv_fd, globalUavNames.size());

	std::set<std::string> notYetConnectedNames(globalUavNames.begin(), globalUavNames.end());

	puts("GZUAVCHANNEL:TCP-LISTENING");

	// block until all UAVs are connected
	while (!notYetConnectedNames.empty())
	{
		int client_fd = accept(serv_fd, nullptr, nullptr);

		// Receive UAV list
		std::vector<std::string> localUavNames;
		std::vector<int> local2global;
		size_t remainingUavs = recv16(client_fd);
		while (remainingUavs-- != 0)
		{
			size_t name_len = recv16(client_fd);
			char name[name_len + 1];
			recvAll(client_fd, name, name_len);
			name[name_len] = '\0';

			if (notYetConnectedNames.erase(name) != 1)
				errx(EXIT_FAILURE, "TCP: received unexpected UAV name: %s", name);

			int idx = std::find(globalUavNames.begin(), globalUavNames.end(), (std::string)name) - globalUavNames.begin();
			warnx("TCP: UAV #%d is connected %s on socket %d", idx, name, client_fd);

			localUavNames.push_back(name);
			local2global.push_back(idx);
		}

		// Create TCPConnection
		TCPConnection *c = new TCPConnection(client_fd, local2global);
		m_connections.push_back(c);
		m_pollGrp.add(c);

		for (int g : local2global)
			m_global2conn.emplace(g, c);
	}

	warnx("TCP: all UAVs are connected");
	close(serv_fd);
}

TCPTransport::TCPTransport(const char *connectTarget, const std::vector<std::string> &globalUavNames)
{
	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;

	const char *colon = strrchr(connectTarget, ':');
	if (colon == nullptr || colon == connectTarget
		|| !isValidPort(atoi(colon + 1))
		|| inet_pton(AF_INET, std::string(connectTarget, colon).c_str(), &addr.sin_addr) != 1)
	{
		errx(EXIT_FAILURE, "TCP: invalid target address or port number");
	}

	addr.sin_port = htons(atoi(colon + 1));

	int fd = socket(AF_INET, SOCK_STREAM, 0);

	if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
		err(EXIT_FAILURE, "TCP: connect(%s) failed", connectTarget);

	std::vector<int> local2global;

	send16(fd, globalUavNames.size());
	for (size_t i = 0; i < globalUavNames.size(); i++)
	{
		send16(fd, globalUavNames[i].length(), true);
		sendAll(fd, globalUavNames[i].c_str(), globalUavNames[i].length());
		local2global.push_back((int)i);
	}

	// Create TCPConnection
	TCPConnection *c = new TCPConnection(fd, local2global);
	m_connections.push_back(c);
	m_pollGrp.add(c);

	for (int g : local2global)
		m_global2conn.emplace(g, c);
}

TCPTransport::~TCPTransport()
{
	for (TCPConnection *c : m_connections)
		delete c;
}

int TCPTransport::fd() const
{
	return m_pollGrp.fd();
}

void TCPTransport::runOnce()
{
	m_pollGrp.runOnce();
}

void TCPTransport::setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb)
{
	for (TCPConnection *c : m_connections)
		c->setReceivedPacketHandler(cb);
}

void TCPTransport::sendPacket(int uav_num, const void *data, size_t len)
{
	m_global2conn.at(uav_num)->sendPacket(uav_num, data, len);
}

TCPTransport::TCPConnection::TCPConnection(int fd, const std::vector<int> &local2global)
: m_fd(fd), m_local2global(local2global), m_pendingMessagesCountdown(local2global.size())
{
	// Create reverse mapping
	for (size_t i = 0; i < m_local2global.size(); i++)
		m_global2local.emplace(m_local2global[i], (int)i);

	// Disable TCP outgoing data buffering
	int optval = 1;
	setsockopt(m_fd, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));

	optval = 0;
	setsockopt(m_fd, IPPROTO_TCP, TCP_CORK, &optval, sizeof(optval));
}

TCPTransport::TCPConnection::~TCPConnection()
{
	close(m_fd);
}

int TCPTransport::TCPConnection::fd() const
{
	return m_fd;
}

void TCPTransport::TCPConnection::runOnce()
{
	uint16_t uav_id = m_local2global.at(recv16(m_fd));
	uint16_t len = recv16(m_fd);

	uint8_t buf[len];
	recvAll(m_fd, buf, len);

	if (m_recvHandler)
		m_recvHandler(uav_id, buf, len);
}

void TCPTransport::TCPConnection::setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb)
{
	m_recvHandler = cb;
}

void TCPTransport::TCPConnection::sendPacket(int uav_num, const void *data, size_t len)
{
	send16(m_fd, m_global2local.at(uav_num), true);
	send16(m_fd, len, true);

	if (--m_pendingMessagesCountdown == 0)
	{
		sendAll(m_fd, data, len, false);
		m_pendingMessagesCountdown = m_local2global.size();
	}
	else
	{
		sendAll(m_fd, data, len, true);
	}
}
