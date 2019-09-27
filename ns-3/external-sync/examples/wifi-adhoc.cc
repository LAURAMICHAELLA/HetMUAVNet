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
#include "ns3/external-sync-manager.h"
#include "ns3/mobility-module.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/timer.h"
#include "ns3/nstime.h"
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/config.h"
#include "ns3/global-value.h"

#include <vector>
#include <string>
#include <unistd.h>
#include <sys/time.h>

#define SIM_DST_PORT 12345

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE("WifiAdHocScriptExample");

std::map<Ipv4Address, Ptr<Node>> ip_node_list;

Ipv4Address
GetAddressOfNode(Ptr<Node> node)
{
  Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
  Ipv4InterfaceAddress iaddr = ipv4->GetAddress(1, 0);
  Ipv4Address addri = iaddr.GetLocal();
  return addri;
}

void ForwardMessage(uint32_t sender, int32_t receiver, const std::vector<uint8_t> &payload)
{
  Ptr<Packet> p = Create<Packet>(payload.data(), payload.size());
  Ptr<Node> nodeSender = NodeList::GetNode(sender);
  Ptr<Socket> sock = nodeSender->GetObject<Socket>();
  if (receiver < 0)
  {
    NodeList::GetNode(sender)->GetDevice(0)->Send(p, NodeList::GetNode(sender)->GetDevice(0)->GetBroadcast(), 0);
    //sock->SendTo(p, 0, InetSocketAddress(Ipv4Address("255.255.255.255"), SIM_DST_PORT));
  }
  else
  {
    Ptr<Node> nodeReceiver = NodeList::GetNode(receiver);
    Ipv4Address dstaddr = GetAddressOfNode(nodeReceiver);
    sock->SendTo(p, 0, InetSocketAddress(dstaddr, SIM_DST_PORT));
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

void SocketReceive(Ptr<Socket> socket)
{
  Address from;
  Ptr<Packet> packet = socket->RecvFrom(from);
  packet->RemoveAllPacketTags();
  packet->RemoveAllByteTags();

  int32_t idfrom = ip_node_list[InetSocketAddress::ConvertFrom(from).GetIpv4()]->GetId();
  uint8_t buffer[packet->GetSize() + sizeof(idfrom)];
  memcpy(buffer, &idfrom, sizeof(idfrom));
  packet->CopyData(buffer + sizeof(idfrom), packet->GetSize());

  ExternalSyncManager::SendMessage(socket->GetNode(), buffer, packet->GetSize() + sizeof(idfrom));
}

int main(int argc, char *argv[])
{

  GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::ExternalSyncSimulatorImpl"));
  CommandLine cmd;

  std::string phyMode("DsssRate1Mbps");
  uint32_t numExternalNodes = 2;
  cmd.AddValue("num-ext-nodes", "Number of external uavs processes", numExternalNodes);
  cmd.Parse(argc, argv);

  ExternalSyncManager::SetSimulatorController("127.0.0.1", 7833);
  ExternalSyncManager::SetNodeControllerServerPort(9998);

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  //Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

  Time::SetResolution(Time::NS);

  NodeContainer nodes;
  nodes.Create(numExternalNodes);

  CsmaHelper csma;
  csma.SetChannelAttribute ("DataRate", StringValue ("100Mbps"));
  csma.SetChannelAttribute ("Delay", StringValue ("2ms"));
  csma.Install(nodes);

  WifiMacHelper mac;
  WifiHelper wifi;
  wifi.SetStandard(WIFI_PHY_STANDARD_80211n_2_4GHZ);

  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode), "ControlMode", StringValue(phyMode));
  mac.SetType("ns3::AdhocWifiMac");

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());

  NetDeviceContainer wifiDevice = wifi.Install(phy, mac, nodes);

  // mobility.
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  /* Internet stack*/
  InternetStackHelper stack;
  stack.Install(nodes);

  Ipv4AddressHelper address;
  address.SetBase("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer wifiNodeInterface;
  wifiNodeInterface = address.Assign(wifiDevice);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  for (uint32_t i = 0; i < numExternalNodes; ++i)
  {
    ExternalSyncManager::RegisterNode(nodes.Get(i), MakeCallback(&ProcessMessage));
    Ptr<Socket> srcSocket = Socket::CreateSocket(nodes.Get(i), TypeId::LookupByName("ns3::UdpSocketFactory"));
    srcSocket->Bind(InetSocketAddress(Ipv4Address::GetAny(), SIM_DST_PORT));
    srcSocket->SetRecvCallback(MakeCallback(&SocketReceive));
    srcSocket->SetAllowBroadcast(true);
    //srcSocket->BindToNetDevice(nodes.Get(i)->GetDevice(1));
    nodes.Get(i)->AggregateObject(srcSocket);
    ip_node_list.emplace(wifiNodeInterface.GetAddress(i), nodes.Get(i));
    std::cerr << "IP of NODE #" << i << " is " << wifiNodeInterface.GetAddress(i) << std::endl;
  }

  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
