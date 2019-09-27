#include "CommandLineParser.h"
#include "ExternalSyncServer.h"
#include "TCPTransport.h"
#include "UDSTransport.h"

#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// packet sent by gazebo plugin
struct fdmPacket
{
	double timestamp;
	double positionXYZ[3];
	// extra fields that are not used by gzuavchannel follow
};

static Transport *makeTransport(const char *spec, const std::vector<std::string> &uavNames)
{
	if (strncasecmp(spec, "tcpc:", 5) == 0)
	{
		return new TCPTransport(spec + 5, uavNames);
	}
	else if (strncasecmp(spec, "tcpl:", 5) == 0)
	{
		return new TCPTransport(atoi(spec + 5), uavNames);
	}
	else if (strncasecmp(spec, "uds:", 4) == 0 && strlen(spec) > 4)
	{
		return new UDSTransport(spec + 4, uavNames);
	}
	else
	{
		errx(EXIT_FAILURE, "Unrecognized transport type: %s", spec);
	}
}

int main(int argc, char *argv[])
{
	CommandLineParser cl(argc, argv);

	// Ensure output is not fully buffered, so that status updates can be
	// received without delays
	setlinebuf(stdout);

	// Initialize server that will provide external synchonization to its clients
	ExternalSyncServer *syncsrv = (cl.externalSyncServerPort == 0) ?
		nullptr : new ExternalSyncServer(cl.externalSyncServerPort);

	// Initialize transport channels
	Transport *upstreamTransport, *downstreamTransport;
	if (cl.invertInitializationOrder == false)
	{
		puts("GZUAVCHANNEL:STARTING");
		upstreamTransport = makeTransport(cl.upstreamSpec, cl.upstreamUavNames);
		puts("GZUAVCHANNEL:HALF");
		downstreamTransport = makeTransport(cl.downstreamSpec, cl.downstreamUavNames);
		puts("GZUAVCHANNEL:GO");
	}
	else
	{
		puts("GZUAVCHANNEL:STARTING");
		downstreamTransport = makeTransport(cl.downstreamSpec, cl.downstreamUavNames);
		puts("GZUAVCHANNEL:HALF");
		upstreamTransport = makeTransport(cl.upstreamSpec, cl.upstreamUavNames);
		puts("GZUAVCHANNEL:GO");
	}

	// Setup callbacks
	size_t forwardedPackets;
	upstreamTransport->setReceivedPacketHandler([&](int uav_num, const void *data, size_t len)
	{
		const fdmPacket *pkt = (const fdmPacket*)data;

		downstreamTransport->sendPacket(uav_num, data, len);

		if (forwardedPackets++ == 0)
		{
			if (syncsrv != nullptr)
				syncsrv->beginPhase0(pkt->timestamp);
		}

		if (syncsrv != nullptr)
			syncsrv->setUavPosition(uav_num, pkt->positionXYZ[0], pkt->positionXYZ[1], pkt->positionXYZ[2]);
	});
	downstreamTransport->setReceivedPacketHandler([&](int uav_num, const void *data, size_t len)
	{
		upstreamTransport->sendPacket(uav_num, data, len);
		forwardedPackets++;
	});

	// Main loop
	while (true)
	{
		if (syncsrv != nullptr)
			syncsrv->endPhase0();

		// End phase 0: Downstream -> Upstream
		forwardedPackets = 0;
		while (forwardedPackets != cl.uavCount)
			downstreamTransport->runOnce();

		// Phase 1
		if (syncsrv != nullptr)
			syncsrv->doPhase1AndMainteinance();

		// Start phase 0: Upstream -> Downstream
		forwardedPackets = 0;
		while (forwardedPackets != cl.uavCount)
			upstreamTransport->runOnce();
	}

	delete syncsrv;
	return EXIT_SUCCESS;
}
