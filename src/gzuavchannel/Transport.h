#ifndef TRANSPORT_H
#define TRANSPORT_H

#include "IO/Poll.h"

#include <stdint.h>

class Transport : public IO::Pollable
{
	public:
		virtual ~Transport() override;

		virtual void setReceivedPacketHandler(const std::function<void(int uav_num, const void *data, size_t len)> &cb) = 0;
		virtual void sendPacket(int uav_num, const void *data, size_t len) = 0;
};

#endif // TRANSPORT_H
