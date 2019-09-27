#include "CommandLineParser.h"

#include <getopt.h>
#include <err.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum
{
	OPT_DOWNSTREAM = 256,
	OPT_UPSTREAM,
	OPT_INVERT_ORDER,
	OPT_EXTERNAL_SYNC_SERVER,
};

static option long_options[] =
{
	{ "help", no_argument, nullptr, 'h' },
	{ "downstream", required_argument, nullptr, OPT_DOWNSTREAM },
	{ "upstream", required_argument, nullptr, OPT_UPSTREAM },
	{ "invert-order", no_argument, nullptr, OPT_INVERT_ORDER },
	{ "external-sync-server", required_argument, nullptr, OPT_EXTERNAL_SYNC_SERVER },
	{ nullptr, 0, nullptr, 0 }
};

static void showHelp()
{
	fprintf(stderr, "Usage: %s --upstream transport_specUS --downstream transport_specDS uav_names...\n", program_invocation_name);
	fprintf(stderr, "\n");
	fprintf(stderr, "This program channels packets between the simulator and ArduCopter.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "\"transport_specUS\" and \"transport_specDS\" can have one of the follwing values:\n");
	fprintf(stderr, " - tcpl:PORT / tcpc:IP:PORT\n");
	fprintf(stderr, "   Connect a set of UAVs through a TCP connection. Use \"tcpl\" on the server\n");
	fprintf(stderr, "   and \"tcpc\" on the client\n");
	fprintf(stderr, " - uds:/path/to/socket\n");
	fprintf(stderr, "   Wait for one SEQPACKET connection from each UAV on the specified Unix Domain\n");
	fprintf(stderr, "   socket path. The first received packet must contain the uav_name.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "\"uav_names...\" is a comma-separated list of UAV names:\n");
	fprintf(stderr, "   If a name does not contain a colon, the same name is used on both sides.\n");
	fprintf(stderr, "   If a name does contain a colon (i.e. \"nameUS:nameDS\"), then nameUS is used by\n");
	fprintf(stderr, "   the upstream transport driver and nameDS is used by the downstream transport\n");
	fprintf(stderr, "   driver.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "Other options:\n");
	fprintf(stderr, " --invert-order: Normally, the upstream transport is initialized first. This\n");
	fprintf(stderr, "                 option makes the downstream transport be initialized first.\n");
	fprintf(stderr, "\n");

	exit(EXIT_FAILURE);
}

CommandLineParser::CommandLineParser(int argc, char *argv[])
{
	downstreamSpec = nullptr;
	upstreamSpec = nullptr;
	invertInitializationOrder = false;
	externalSyncServerPort = 0;

	bool help_requested = false;

	while (true)
	{
		int c, option_index = 0;
		c = getopt_long(argc, argv, "+h", long_options, &option_index);

		if (c == -1) // end of options
			break;

		switch (c)
		{
			case '?':
			case ':':
				// getopt() has already printed an error message
				exit(EXIT_FAILURE);
			case 'h':
				help_requested = true;
				break;
			case OPT_DOWNSTREAM:
				if (downstreamSpec != nullptr)
					errx(EXIT_FAILURE, "option '%s' cannot be specified more than once", long_options[option_index].name);
				downstreamSpec = optarg;
				break;
			case OPT_UPSTREAM:
				if (upstreamSpec != nullptr)
					errx(EXIT_FAILURE, "option '%s' cannot be specified more than once", long_options[option_index].name);
				upstreamSpec = optarg;
				break;
			case OPT_INVERT_ORDER:
				if (invertInitializationOrder)
					errx(EXIT_FAILURE, "option '%s' cannot be specified more than once", long_options[option_index].name);
				invertInitializationOrder = true;
				break;
			case OPT_EXTERNAL_SYNC_SERVER:
				if (externalSyncServerPort != 0)
					errx(EXIT_FAILURE, "option '%s' cannot be specified more than once", long_options[option_index].name);
				externalSyncServerPort = atoi(optarg);
				if (externalSyncServerPort == 0)
					errx(EXIT_FAILURE, "option '%s' has invalid format", long_options[option_index].name);
				break;
		}
	}

	if (help_requested)
		showHelp();

	if (downstreamSpec == nullptr)
		errx(EXIT_FAILURE, "option 'downstream' is required");

	if (upstreamSpec == nullptr)
		errx(EXIT_FAILURE, "option 'upstream' is required");

	if (optind == argc)
		errx(EXIT_FAILURE, "at least one UAV name is required");

	for (int i = optind; i < argc; i++)
	{
		const char *text = argv[i];
		const char *colon = strchr(text, ':');

		if (colon == nullptr)
		{
			// same name on both sides
			upstreamUavNames.emplace_back(text);
			downstreamUavNames.emplace_back(text);
		}
		else
		{
			// different names
			upstreamUavNames.emplace_back(text, colon);
			downstreamUavNames.emplace_back(colon + 1);
		}
	}

	uavCount = upstreamUavNames.size();
}
