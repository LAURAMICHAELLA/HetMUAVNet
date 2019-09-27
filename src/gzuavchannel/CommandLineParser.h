#ifndef COMMANDLINEPARSER_H
#define COMMANDLINEPARSER_H

#include <string>
#include <vector>

struct CommandLineParser
{
	CommandLineParser(int argc, char *argv[]);

	const char *downstreamSpec;
	const char *upstreamSpec;
	bool invertInitializationOrder;

	int externalSyncServerPort; // 0 = no server

	size_t uavCount;
	std::vector<std::string> upstreamUavNames;
	std::vector<std::string> downstreamUavNames;
};

#endif // COMMANDLINEPARSER_H
