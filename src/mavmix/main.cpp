#include "IO/Poll.h"
#include "MAVLink/MAVLink.h"

#include <arpa/inet.h>
#include <err.h>
#include <linux/sockios.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>

#include <set>

#define PRINT_MESSAGES false

class TCPMavlinkConnection : public IO::Pollable
{
	public:
		TCPMavlinkConnection(int fd)
		: m_fd(fd)
		{
			memset(&m_rxmsg, 0, sizeof(m_rxmsg));
			memset(&m_status, 0, sizeof(m_status));
		}

		~TCPMavlinkConnection() override
		{
			close(m_fd);
		}

		void setMessageHandler(const std::function<void(const mavlink_message_t *msg)> &cb)
		{
			m_recvMsg = cb;
		}

		void setConnectionLostHandler(const std::function<void()> &cb)
		{
			m_connLost = cb;
		}

		int fd() const override
		{
			return m_fd;
		}

		void runOnce()
		{
			uint8_t data[65536];
			int r = recv(m_fd, data, sizeof(data), 0);

			if (r <= 0)
			{
				if (m_connLost)
					m_connLost();
				return;
			}

			for (int i = 0; i < r; i++)
			{
				mavlink_status_t mavlink_status;
				bool msg_available = mavlink_frame_char_buffer(&m_rxmsg, &m_status, data[i], &m_nextMessage, &mavlink_status);

				if (msg_available)
				{
					if (m_recvMsg)
						m_recvMsg(&m_nextMessage);
				}
			}
		}

		bool safeSend(const mavlink_message_t *msg)
		{
			uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
			int length = mavlink_msg_to_send_buffer(buffer, msg);

			int value;
			ioctl(m_fd, SIOCOUTQ, &value);

			// send only if there are less than 4K bytes already enqueued
			if (value < 4096)
				send(m_fd, buffer, length, 0);
		}

	private:
		int m_fd;

		mavlink_message_t m_rxmsg, m_nextMessage;
		mavlink_status_t m_status;

		std::function<void(const mavlink_message_t *msg)> m_recvMsg;
		std::function<void()> m_connLost;
};

static IO::PollGroup pg;
static TCPMavlinkConnection *gcs;
static std::set<TCPMavlinkConnection*> uavs;

static void newConnectionGCS(int new_sk)
{
	if (gcs != nullptr)
	{
		pg.remove(gcs);
		delete gcs;
	}

	gcs = new TCPMavlinkConnection(new_sk);

	gcs->setMessageHandler([](const mavlink_message_t *msg)
	{
		if (PRINT_MESSAGES)
		{
			printf("ToUAV %u:%u:%u ", msg->sysid, msg->compid, msg->seq);
			MAVLink::print_message(msg);
		}

		// send to all UAVs
		for (TCPMavlinkConnection *uav : uavs)
			uav->safeSend(msg);
	});

	gcs->setConnectionLostHandler([]()
	{
		pg.remove(gcs);
		delete gcs;
		gcs = nullptr;
	});

	pg.add(gcs);
}

static void newConnectionUAV(int new_sk)
{
	TCPMavlinkConnection *uav = new TCPMavlinkConnection(new_sk);

	uav->setMessageHandler([](const mavlink_message_t *msg)
	{
		if (PRINT_MESSAGES)
		{
			printf("ToGCS %u:%u:%u ", msg->sysid, msg->compid, msg->seq);
			MAVLink::print_message(msg);
		}

		// send to GCS
		if (gcs)
			gcs->safeSend(msg);
	});

	uav->setConnectionLostHandler([uav]()
	{
		pg.remove(uav);
		uavs.erase(uav);
		delete uav;
	});

	pg.add(uav);
	uavs.insert(uav);
}

static int makeServer(int port)
{
	int fd = socket(AF_INET, SOCK_STREAM, 0);

	int optval = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = htonl(INADDR_ANY);
	addr.sin_port = htons(port);
	if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
		err(EXIT_FAILURE, "bind(%d) failed", port);

	listen(fd, 1000);

	return fd;
}

int main(int argc, const char *argv[])
{
	if (argc != 3)
	{
		fprintf(stderr, "Usage: %s GCSport UAVport\n", argv[0]);
		return EXIT_FAILURE;
	}

	int portGCS = atoi(argv[1]);
	int portUAV = atoi(argv[2]);

	int serv_gcs = makeServer(portGCS);
	int serv_uav = makeServer(portUAV);

	pg.add(serv_gcs, [=]() { newConnectionGCS(accept(serv_gcs, nullptr, nullptr)); });
	pg.add(serv_uav, [=]() { newConnectionUAV(accept(serv_uav, nullptr, nullptr)); });

	while (true)
		pg.runOnce();

	return EXIT_SUCCESS;
}
