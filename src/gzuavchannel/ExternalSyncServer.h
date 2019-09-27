#ifndef EXTERNALSYNCSERVER_H
#define EXTERNALSYNCSERVER_H

#include <map>
#include <vector>

class ExternalSyncServer
{
	public:
		ExternalSyncServer(int listenPort);
		~ExternalSyncServer();

		void beginPhase0(double ts);
		void endPhase0();

		void doPhase1AndMainteinance();

		void setUavPosition(int uavId, double x, double y, double z);

	private:
		std::vector<uint8_t> buildStatePacket() const;

		// Server socket that accepts new connections
		int m_serv;

		// Connections that synchronize with Phase 0
		std::vector<int> m_clientsPhase0;

		// Connections that synchronize with Phase 1
		std::vector<int> m_clientsPhase1;

		// Simulation status
		struct Position { double x, y, z; };
		std::map<uint32_t, Position> m_positions; // uavId -> position
		double m_currentTimestamp;
};

#endif // EXTERNALSYNCSERVER_H
