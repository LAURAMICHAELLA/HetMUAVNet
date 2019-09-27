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

#ifndef EXTERNAL_SYNC_MANAGER_H
#define EXTERNAL_SYNC_MANAGER_H


#include "ns3/core-module.h"
#include "ns3/external-sync-simulator-impl.h"
#include "ns3/ptr.h"
#include "ns3/callback.h"
#include "ns3/node.h"

namespace ns3 {

class ExternalSyncManager
{
  friend class ExternalSyncSimulatorImpl;

public:
  static void SetSimulatorController(const char *ip, int port);
  static void SetNodeControllerServerPort(int port);
  static void RegisterNode(Ptr<Node> node, Callback<void, Ptr<Node>, const void*, size_t> cb);
  static void SendMessage(Ptr<Node> n, const void *payload, size_t size);

private:
  /* Called by ExternalSimulatorImpl::Run. */
  static void InitExternalConnections();
  static double WaitForBeginTick();
  static bool SendEndTick();
  static bool SendAll(int s, const void *payload, size_t size);

  static void InitSimSyncConnection();
  static void WaitForNodesConnections();
  static void RegisterExternalSocket(Ptr<Node> node, int fd, Callback<void, Ptr<Node>, const void*, size_t> cb);
  static bool ProcessMessage(int fd);
  static void SetPosition(uint32_t id, double x, double y, double z);
  static void DisableTcpDelays(int fd);
  static double ProcessBeginTick();
};

}

#endif /* EXTERNAL_SYNC_MANAGER_H */
