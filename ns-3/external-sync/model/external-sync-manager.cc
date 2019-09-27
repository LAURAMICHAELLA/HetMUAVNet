/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Fabio D'Urso, Federico Fausto Santoro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Fabio D'Urso <durso@dmi.unict.it>
 *         Federico Fausto Santoro <federico.santoro@unict.it>
 */

#define DEFAULT_NODECONTROLLER_SERVER_PORT 9998
#define DEFAULT_SIMSYNC_PORT 9999

#include "ns3/node-list.h"
#include "ns3/mobility-module.h"
#include "ns3/object.h"
#include "ns3/config.h"
#include "ns3/global-value.h"
#include "external-sync-manager.h"

#include <errno.h>
#include <err.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <vector>
#include <map>

using namespace std;

namespace ns3 {

// Pending list of nodes.
static map<uint32_t, Callback<void, Ptr<Node>, const void*, size_t> > g_registeredPendings;

// List of connected sockets with nodes and callbacks.
static map<Ptr<Node>, int> g_registeredSockets;
//reverse mapping of g_registeredSockets
static map<int, Ptr<Node> > g_registeredNodeSockets;
static map<int, Callback<void, Ptr<Node>, const void*, size_t> > g_registeredCallbacks;

static int g_simsync_socket;
static in_addr_t g_simsync_ip = htonl(INADDR_LOOPBACK);
static int g_simsync_port = DEFAULT_SIMSYNC_PORT;
static int g_nodecontroller_port = DEFAULT_NODECONTROLLER_SERVER_PORT;

void
ExternalSyncManager::SetSimulatorController(const char *ip, int port)
{
  g_simsync_ip = inet_addr(ip);
  g_simsync_port = port;
}

void
ExternalSyncManager::SetNodeControllerServerPort(int port)
{
  g_nodecontroller_port = port;
}

void
ExternalSyncManager::RegisterNode(Ptr<Node> node, Callback<void, Ptr<Node>, const void*, size_t> cb)
{
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(node);
  g_registeredPendings.emplace(node->GetId(), cb);
}

void
ExternalSyncManager::SendMessage(Ptr<Node> n, const void *payload, size_t size)
{
  int socket = g_registeredSockets[n];
  uint32_t size32 = size;

  if (!SendAll(socket, &size32, sizeof(uint32_t)) || !SendAll(socket, payload, size))
    {
      errx(EXIT_FAILURE, "Failed to send data to Node Controller");
    }

  char buff;
  if (recv(socket, &buff, 1, 0) != 1 || buff != '!')
    {
      errx(EXIT_FAILURE, "Failed to receive ack from Node Controller");
    }
}

void
ExternalSyncManager::InitSimSyncConnection()
{
  // Connect to Simulation Controller (TCP)
  g_simsync_socket = socket(AF_INET, SOCK_STREAM, 0);

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = g_simsync_ip;
  serv_addr.sin_port = htons(g_simsync_port);

  warnx("Connecting to the Simulation Controller");

  if (connect(g_simsync_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
    err(EXIT_FAILURE, "Cannot connect to the Simulation Controller");

  warnx("Connected to the Simulation Controller");

  DisableTcpDelays(g_simsync_socket);

  // Phase Subscription TODO: must be a callback
  send(g_simsync_socket, "\001", 1, 0);
}

void
ExternalSyncManager::WaitForNodesConnections()
{
  struct sockaddr_in serverAddress;
  int server_socket = socket(AF_INET, SOCK_STREAM, 0);

  int optval = 1;
  setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));

  memset(&serverAddress, 0, sizeof(serverAddress));
  serverAddress.sin_family = AF_INET;
  serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
  serverAddress.sin_port = htons(g_nodecontroller_port);

  if (bind(server_socket, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) < 0)
    err(EXIT_FAILURE, "Failed to bind to port %d", g_nodecontroller_port);

  listen(server_socket, g_registeredPendings.size());

  warnx("Waiting for Node Controller connections");

  while (g_registeredPendings.empty() == false)
    {
      int s = accept(server_socket, nullptr, nullptr);

      DisableTcpDelays(s);

      uint32_t id;
      if (recv(s, &id, sizeof(uint32_t), MSG_WAITALL) != sizeof(uint32_t))
        errx(EXIT_FAILURE, "A Node Controller connection was terminated unexpectedly");

      if (g_registeredPendings.find(id) == g_registeredPendings.end())
        errx(EXIT_FAILURE, "A Node Controller connection attempted to register an unexpected or already taken ID");

      warnx("Node Controller #%u connected", id);

      RegisterExternalSocket(NodeList::GetNode(id), s, g_registeredPendings[id]);
      g_registeredPendings.erase(id);
    }

  warnx("All Node Controllers are connected");
  close(server_socket);
}

void
ExternalSyncManager::InitExternalConnections()
{
  InitSimSyncConnection();
  WaitForNodesConnections();
}

void
ExternalSyncManager::RegisterExternalSocket(Ptr<Node> node, int fd, Callback<void,Ptr<Node>, const void*, size_t> cb)
{
  g_registeredSockets.emplace(node, fd);
  g_registeredNodeSockets.emplace(fd, node);
  g_registeredCallbacks.emplace(fd, cb);
}

double
ExternalSyncManager::WaitForBeginTick()
{
  while (true)
    {
      fd_set rfds;
      FD_ZERO(&rfds);
      FD_SET(g_simsync_socket, &rfds);
      int maxfd = g_simsync_socket;

      for (auto kv : g_registeredCallbacks)
        {
          FD_SET(kv.first, &rfds);
          if (maxfd < kv.first)
            maxfd = kv.first;
        }

      if (select(maxfd+1, &rfds, nullptr, nullptr, nullptr) < 0)
        errx(EXIT_FAILURE, "Select failed");

      for (auto kv : g_registeredCallbacks)
        {
          if (FD_ISSET(kv.first, &rfds))
            {
              if (!ProcessMessage(kv.first))
                return -1;
            }
        }

      if (FD_ISSET(g_simsync_socket, &rfds))
        {
          double timestamp = ProcessBeginTick();
          if (timestamp < 0)
            warnx("The Simulation Controller connection was terminated");

          return timestamp;
        }
    }
}

double
ExternalSyncManager::ProcessBeginTick()
{
  double buf;
  if (recv(g_simsync_socket, &buf, sizeof(double), MSG_WAITALL) != sizeof(double))
    {
      return -1;
    }

  uint32_t num_positions;
  if (recv(g_simsync_socket, &num_positions, sizeof(uint32_t), MSG_WAITALL) == sizeof(uint32_t))
    {
      // The Simulator Controller sent some positions
      for (uint32_t i = 0; i < num_positions; ++i)
        {
          uint32_t nodeid;
          double pos [3];
          if (recv(g_simsync_socket, &nodeid, sizeof(uint32_t), MSG_WAITALL) != sizeof(uint32_t) || recv(g_simsync_socket, pos, sizeof(double)*3, MSG_WAITALL) != sizeof(double)*3)
            {
              return -1;
            }

          SetPosition(nodeid, pos[0], pos[1], pos[2]);
        }
    }
  else
    {
      return -1;
    }

  return buf * 1e9; // seconds -> nanoseconds
}

bool
ExternalSyncManager::SendEndTick()
{
  return send(g_simsync_socket, "!", 1, MSG_NOSIGNAL) == 1;
}

bool
ExternalSyncManager::SendAll(int s, const void *payload, size_t size)
{
  size_t sent = 0;

  while (sent < size)
    {
      int r = send(s, (const char*) payload + sent, size - sent, 0);

      if (r <= 0)
        return false;

      sent += r;
    }

  return true;
}

bool
ExternalSyncManager::ProcessMessage(int fd)
{
  uint32_t payloadLength;
  int received = recv(fd, &payloadLength, sizeof(uint32_t), MSG_WAITALL);

  if (received == 0)
    {
      warnx("A Node Controller connection was terminated");
      return false;
    }
  else if (received != sizeof(uint32_t))
    {
      errx(EXIT_FAILURE, "Failed to receive next packet's length from a Node Controller");
    }

  void* buffer = malloc(payloadLength);
  if (recv(fd, buffer, payloadLength, MSG_WAITALL) != payloadLength)
    errx(EXIT_FAILURE, "Failed to receive next packet's payload from a Node Controller");

  g_registeredCallbacks[fd](g_registeredNodeSockets[fd], buffer, payloadLength);
  free(buffer);

  if (send(fd, "!", 1, MSG_NOSIGNAL) != 1)
    errx(EXIT_FAILURE, "Failed to send ACK to a Node Controller");

  return true;
}

void
ExternalSyncManager::SetPosition(uint32_t id, double x, double y, double z)
{
  Ptr<Node> node = NodeList::GetNode(id);
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
  Vector pos;
  pos.x = x;
  pos.y = y;
  pos.z = z;
  mobility->SetPosition(pos);
}

void
ExternalSyncManager::DisableTcpDelays(int fd)
{
  int optval = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));

  optval = 0;
  setsockopt(fd, IPPROTO_TCP, TCP_CORK, &optval, sizeof(optval));
}

}
