/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/csma-module.h"
#include "ns3/external-sync-manager.h"


#include "ns3/mobility-module.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <string>
#include <vector>

#define SIM_DST_PORT 12345

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("LanScriptExample");

std::map<Ipv4Address, Ptr<Node>> ip_node_list;

Ipv4Address
GetAddressOfNode(Ptr<Node> node)
{
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
  Ipv4Address addri = iaddr.GetLocal();
  return addri;
}

void
ForwardMessage(uint32_t sender, int32_t receiver, const std::vector<uint8_t> &payload)
{
  Ptr<Packet> p = Create<Packet> (payload.data(), payload.size());
  Ptr<Node> nodeSender = NodeList::GetNode(sender);
  Ptr<Socket> sock = nodeSender->GetObject<Socket>();
  if (receiver < 0)
    {
      sock->SendTo(p, 0, InetSocketAddress (Ipv4Address ("255.255.255.255"), SIM_DST_PORT));
    }
  else
    {
      Ptr<Node> nodeReceiver = NodeList::GetNode(receiver);
      Ipv4Address dstaddr = GetAddressOfNode(nodeReceiver);
      sock->SendTo(p, 0, InetSocketAddress (dstaddr, SIM_DST_PORT));
    }
}

void
ProcessMessage(Ptr<Node> sender, const void* buffer, size_t size)
{
  /*
  Message protocol struct

                  BUFFER
  -------------------------------------
  |             ID_NODE (4)           |  S
  -------------------------------------  I
  |                                   |  Z
  |      PAYLOAD (LENGTH_MESSAGE)     |  E
  |                                   |
  -------------------------------------
  */

  int32_t nodeid;

  memcpy(&nodeid, (((char*)buffer)), sizeof(nodeid));
  std::vector<uint8_t> payload((const char*)buffer + 4, (const char*)buffer + size);

  Simulator::Schedule(MilliSeconds(1), &ForwardMessage, sender->GetId(), nodeid, payload);
}

void
SocketReceive (Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> packet = socket->RecvFrom (from);
  packet->RemoveAllPacketTags ();
  packet->RemoveAllByteTags ();

  int32_t idfrom = ip_node_list[InetSocketAddress::ConvertFrom(from).GetIpv4()]->GetId();
  uint8_t buffer [packet->GetSize()+sizeof(idfrom)];
  memcpy(buffer,&idfrom,sizeof(idfrom));
  packet->CopyData(buffer+sizeof(idfrom),packet->GetSize());

  ExternalSyncManager::SendMessage(socket->GetNode(), buffer, packet->GetSize()+sizeof(idfrom));
}

int
main (int argc, char *argv[])
{
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::ExternalSyncSimulatorImpl"));
  CommandLine cmd;

  uint32_t numExternalNodes = 2;
  cmd.AddValue("num-ext-nodes","Number of external nodes processes", numExternalNodes);

  cmd.Parse(argc, argv);

  ExternalSyncManager::SetSimulatorController("127.0.0.1", 7833);
  ExternalSyncManager::SetNodeControllerServerPort(9998);

  Time::SetResolution (Time::NS);
  LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);

  NodeContainer nodes;
  nodes.Create(numExternalNodes);

  InternetStackHelper stack;
  stack.Install(nodes);

  for (uint32_t i = 0; i < numExternalNodes; ++i)
    {
      ExternalSyncManager::RegisterNode(nodes.Get(i), MakeCallback(&ProcessMessage));
    }

  CsmaHelper csma;
  csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
  csma.SetChannelAttribute("Delay", TimeValue (MilliSeconds(2)));
  csma.SetDeviceAttribute("Mtu", UintegerValue(1400));
  NetDeviceContainer devices = csma.Install(nodes);

  Ipv4AddressHelper address;
  address.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = address.Assign (devices);

  for (uint32_t i = 0; i < numExternalNodes; ++i)
    {
      Ptr<Socket> srcSocket = Socket::CreateSocket (nodes.Get(i), TypeId::LookupByName ("ns3::UdpSocketFactory"));
      srcSocket->Bind(InetSocketAddress (Ipv4Address::GetAny (), SIM_DST_PORT));
      srcSocket->SetRecvCallback (MakeCallback (&SocketReceive));
      srcSocket->SetAllowBroadcast(true);
      nodes.Get(i)->AggregateObject(srcSocket);
      ip_node_list.emplace(interfaces.GetAddress(i), nodes.Get(i));
      std::cerr << "IP of Node #" << i << " is " << interfaces.GetAddress(i) << std::endl;
    }

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
