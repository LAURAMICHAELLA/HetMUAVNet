#include "ExternalSyncServer.h"

#include <arpa/inet.h>
#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>

static bool isValidPort(int n)
{
	if (n < 1 || n > 65535)
		return false;
	else
		return true;
}

static bool sendAll(int fd, const void *buf, size_t len)
{
	while (len != 0)
	{
		int r = send(fd, buf, len, 0);
		if (r <= 0)
			return false;

		buf = r + (const char*)buf;
		len -= r;
	}

	return true;
}

// send (buf, len) to all file descriptors and remove those that fail
static void sendOrRemove(std::vector<int> &fds, const void *buf, size_t len)
{
	std::vector<int>::iterator it = fds.begin();
	while (it != fds.end())
	{
		if (sendAll(*it, buf, len))
		{
			++it;
		}
		else
		{
			warnx("ExternalSyncServer: failed to send BEGIN-TICK, removing client");

			close(*it);
			it = fds.erase(it);
		}
	}
}

// receive one byte from each file descriptor and remove those that fail
static void recvAckOrRemove(std::vector<int> &fds)
{
	std::vector<int>::iterator it = fds.begin();
	while (it != fds.end())
	{
		char dummy;
		if (recv(*it, &dummy, 1, 0) == 1)
		{
			++it;
		}
		else
		{
			warnx("ExternalSyncServer: failed to receive END-TICK, removing client");

			close(*it);
			it = fds.erase(it);
		}
	}
}

ExternalSyncServer::ExternalSyncServer(int listenPort)
{
	if (!isValidPort(listenPort))
		errx(EXIT_FAILURE, "ExternalSyncServer: port number has invalid format or value");

	m_serv = socket(AF_INET, SOCK_STREAM, 0);

	int optval = 1;
	setsockopt(m_serv, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(listenPort);

	if (bind(m_serv, (struct sockaddr*)&addr, sizeof(addr)) < 0)
		err(EXIT_FAILURE, "ExternalSyncServer: bind(%d) failed", listenPort);

	listen(m_serv, 1000);

	// Make accept() non-blocking
	int flags = fcntl(m_serv, F_GETFL, 0);
	fcntl(m_serv, F_SETFL, flags | O_NONBLOCK);
}

ExternalSyncServer::~ExternalSyncServer()
{
	for (int fd : m_clientsPhase0)
		close(fd);

	for (int fd : m_clientsPhase1)
		close(fd);

	close(m_serv);
}

void ExternalSyncServer::beginPhase0(double ts)
{
	// Send ts to all clients in m_clientsPhase0
	sendOrRemove(m_clientsPhase0, &ts, sizeof(double));
	m_currentTimestamp = ts;
}

void ExternalSyncServer::endPhase0()
{
	// Wait for all clients in m_clientsPhase0 to complete
	recvAckOrRemove(m_clientsPhase0);
}

void ExternalSyncServer::doPhase1AndMainteinance()
{
	// Send ts and positions to all clients in m_clientsPhase1
	if (m_clientsPhase1.empty() == false)
	{
		std::vector<uint8_t> pkt = buildStatePacket();
		sendOrRemove(m_clientsPhase1, pkt.data(), pkt.size());
		recvAckOrRemove(m_clientsPhase1);
	}

	// Accept new connections
	while (true)
	{
		// Try and accept a new incoming connection
		int fd = accept(m_serv, nullptr, nullptr);
		if (fd < 0)
		{
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				break;
			else
				err(EXIT_FAILURE, "ExternalSyncServer: accept failed");
		}

		// Make the new connection blocking
		int flags = fcntl(fd, F_GETFL, 0);
		fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

		// Disable TCP outgoing data buffering
		int optval = 1;
		setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));

		optval = 0;
		setsockopt(fd, IPPROTO_TCP, TCP_CORK, &optval, sizeof(optval));

		// Identify the phase this client is interested in
		char subscribePhase;
		int r = recv(fd, &subscribePhase, 1, 0);

		// Add it to the proper list
		if (r == 1 && subscribePhase == 0)
		{
			m_clientsPhase0.push_back(fd);
		}
		else if (r == 1 && subscribePhase == 1)
		{
			m_clientsPhase1.push_back(fd);
		}
		else
		{
			warnx("ExternalSyncServer: invalid phase subscription data from incoming connection");
			close(fd);

			continue;
		}

		warnx("ExternalSyncServer: new connection subscribed to phase %d", subscribePhase);
	}
}

void ExternalSyncServer::setUavPosition(int uavId, double x, double y, double z)
{
	m_positions[uavId] = { x, y, z };
}

std::vector<uint8_t> ExternalSyncServer::buildStatePacket() const
{
	std::vector<uint8_t> buff;
	buff.reserve(
		sizeof(double) + sizeof(uint32_t) +
		( sizeof(uint32_t) + 3*sizeof(double) ) * m_positions.size());

	// helper function that appends data to buff
	auto appendToBuff = [&buff](const void *data, size_t len)
	{
		buff.insert(buff.end(), (const uint8_t*)data, (const uint8_t*)data + len);
	};

	// timestamp (double)
	appendToBuff(&m_currentTimestamp, sizeof(double));

	// num_of_position_records (uint32_t)
	uint32_t numPositions = (uint32_t)m_positions.size();
	appendToBuff(&numPositions, sizeof(uint32_t));

	// for each UAV
	for (auto it : m_positions)
	{
		// uav_id (uint32_t)
		appendToBuff(&it.first, sizeof(uint32_t));

		// x, y, z (double)
		appendToBuff(&it.second, 3 * sizeof(double));
	}

	return buff;
}
