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
#include <ns3/log.h>
#include <ns3/core-module.h>
#include <ns3/lr-wpan-module.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/propagation-delay-model.h>
#include <ns3/simulator.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/packet.h>
#include "ns3/external-sync-manager.h"
#include "ns3/mobility-module.h"
#include <ns3/lr-wpan-error-model.h>
#include <ns3/propagation-loss-model.h>
#include <ns3/lr-wpan-net-device.h>
#include <ns3/spectrum-value.h>
#include <ns3/lr-wpan-spectrum-value-helper.h>
#include <ns3/lr-wpan-mac.h>
#include <ns3/node.h>
#include <ns3/node-list.h>
#include <ns3/net-device.h>
#include <ns3/single-model-spectrum-channel.h>
#include <ns3/multi-model-spectrum-channel.h>
#include <ns3/mac16-address.h>
#include <ns3/constant-position-mobility-model.h>
#include <ns3/uinteger.h>
#include <ns3/nstime.h>
#include <ns3/abort.h>
#include <ns3/command-line.h>
#include "ns3/global-value.h"

#include <iostream>

using namespace ns3;
using namespace std;

map<Mac16Address, Ptr<Node>> mac_node_list;
map<uint32_t, Mac16Address> node_mac_list;
map<uint32_t, Ptr<LrWpanNetDevice>> node_device_list;
bool verbose = false;

void
ForwardMessage(uint32_t sender, int32_t receiver, const std::vector<uint8_t> &payload)
{
  Ptr<Packet> packet = Create<Packet>(payload.data(), payload.size());
  Ptr<Node> nodeSender = NodeList::GetNode(sender);
  Ptr<LrWpanNetDevice> device = node_device_list[sender];
  McpsDataRequestParams params;
  params.m_srcAddrMode = SHORT_ADDR;
  params.m_dstAddrMode = SHORT_ADDR;
  params.m_dstPanId = 0;
  params.m_msduHandle = 0;
  //params.m_txOptions = TX_OPTION_ACK;
  params.m_txOptions = 0;
  
  Address sendto;
  if (receiver < 0)
    {
      params.m_dstAddr = Mac16Address ("ff:ff");
      sendto = Mac16Address ("ff:ff");
    }
  else
    {
      params.m_dstAddr = node_mac_list[receiver];
      sendto = node_mac_list[receiver];
    }
  
  if (verbose)
    cout << "At:" << Simulator::Now().GetSeconds() << " Node[" << device->GetNode()->GetId() << "] sent a packet [" << payload.data() << ", " << payload.size() << " bytes] to Node[" << params.m_dstAddr << "]" << std::endl;
  
  device->GetMac()->McpsDataRequest(params,packet);
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
  |          PAYLOAD (SIZE)           |  E
  |                                   |
  -------------------------------------
  */

  int32_t nodeid;
  
  memcpy(&nodeid, (((char*)buffer)), sizeof(nodeid));
  std::vector<uint8_t> payload((const char*)buffer + 4, (const char*)buffer + (size));

  if (verbose)
    cout << "Node[" << sender->GetId() << "] wants send a packet [" << ((const char*)buffer + 4) << ", " << (size - 4) << "] to Node[" << nodeid << "] of length " << (size - 4) << "bytes" << std::endl;

  Simulator::ScheduleNow(&ForwardMessage, sender->GetId(), nodeid, payload);
}

static bool
PacketReceived(Ptr<NetDevice> device, Ptr<const Packet> packet, short unsigned int flags, const ns3::Address& from){

  Ptr<Packet> p = packet->Copy();
  p->RemoveAllPacketTags();
  p->RemoveAllByteTags();

  int32_t idfrom = mac_node_list[Mac16Address::ConvertFrom(from)]->GetId();
  uint8_t buffer[p->GetSize() + sizeof(idfrom)];
  memcpy(buffer, &idfrom, sizeof(idfrom));
  p->CopyData(buffer + sizeof(idfrom), p->GetSize());

  if (verbose)
    cout << "At:" << Simulator::Now().GetSeconds() << " Node[" << device->GetNode()->GetId() << "] received a packet from Node[" << idfrom << "] with message: " << ((const char*)buffer + 4) << std::endl;
  
  ExternalSyncManager::SendMessage(device->GetNode(), buffer, p->GetSize() + sizeof(idfrom));
  return true;
}

int main (int argc, char *argv[])
{

  GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::ExternalSyncSimulatorImpl"));

  ExternalSyncManager::SetSimulatorController("127.0.0.1", 7833);
  ExternalSyncManager::SetNodeControllerServerPort(9998);
  Packet::EnablePrinting ();
  Packet::EnableChecking ();
  Time::SetResolution (Time::NS);

  int packetSize = 20;
  // a default transmit power of -30 dBm (12 meters).
  double txPower = -30;
  uint32_t channelNumber = 11;
  uint32_t numExternalNodes = 2;
  

  CommandLine cmd;
  cmd.AddValue("num-ext-nodes","Number of external nodes processes", numExternalNodes);
  cmd.AddValue("tx-power","Power of trasmission device", txPower);
  cmd.AddValue("channel","Channel of network", channelNumber);
  cmd.AddValue("verbose","If you want to see the packets logs", verbose);

  cmd.Parse (argc, argv);

  NodeContainer nodes;
  nodes.Create(numExternalNodes);  

  // Each device must be attached to the same channel
  Ptr<MultiModelSpectrumChannel> channel = CreateObject<MultiModelSpectrumChannel>();
  Ptr<LogDistancePropagationLossModel> propModel = CreateObject<LogDistancePropagationLossModel>();
  //Ptr<ConstantSpeedPropagationDelayModel> delayModel = CreateObject<ConstantSpeedPropagationDelayModel> ();
  Ptr<RandomPropagationDelayModel> delayModel = CreateObject<RandomPropagationDelayModel>();
  channel->AddPropagationLossModel(propModel);
  channel->SetPropagationDelayModel(delayModel);

  for (uint32_t i = 0; i < numExternalNodes; ++i)
    {
      ExternalSyncManager::RegisterNode(nodes.Get(i), MakeCallback(&ProcessMessage));
      Ptr<Packet> p = Create<Packet>(packetSize);
      Ptr<LrWpanNetDevice> device = CreateObject<LrWpanNetDevice>();

      Mac16Address macaddress = Mac16Address();
      uint8_t shortaddr[2] = { (uint8_t)(i >> 8), (uint8_t)(i & 0xff) };
      macaddress.CopyFrom(shortaddr);
      device->SetAddress(macaddress);

      device->SetChannel(channel);

      nodes.Get(i)->AddDevice(device);

      LrWpanSpectrumValueHelper svh;
      Ptr<SpectrumValue> psd = svh.CreateTxPowerSpectralDensity (txPower, channelNumber);
      device->GetPhy()->SetTxPowerSpectralDensity(psd);

      device->SetReceiveCallback(MakeCallback(&PacketReceived));

      std::cerr << "MAC of NODE #" << i << " is " << device->GetAddress() << std::endl;
      mac_node_list.emplace(macaddress, nodes.Get(i));
      node_mac_list.emplace(i, macaddress);
      node_device_list.emplace(i, device);
    }

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;
}
