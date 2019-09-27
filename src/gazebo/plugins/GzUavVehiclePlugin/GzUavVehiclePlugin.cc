/*
 * Copyright (C) 2018 Fabio D'Urso <durso@dmi.unict.it>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "GzUavVehiclePlugin/GzUavVehiclePlugin.hh"

#include "GzUavPhaseGenerator.hh"

#include <sys/socket.h>
#include <sys/un.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(GzUav::GzUavVehiclePlugin)

GzUav::GzUavVehiclePlugin::GzUavVehiclePlugin()
{
}

GzUav::GzUavVehiclePlugin::~GzUavVehiclePlugin()
{
  for (std::pair<Rotor*, int> it : rotors)
  {
    Rotor *rotor = it.first;
    delete rotor;
  }
}

void GzUav::GzUavVehiclePlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "[GzUavVehiclePlugin] model pointer is null");
  GZ_ASSERT(_sdf, "[GzUavVehiclePlugin] sdf pointer is null");

  if (getenv("GZUAV_UDS") == nullptr)
    gzthrow("[GzUavVehiclePlugin] Environment variable GZUAV_UDS must be set");

  // Store pointer to the model
  this->model = _model;

  // Initialize internal components and rotors
  this->poseSampler.Load(_model, _sdf);
  this->gimbal = nullptr;
  sdf::ElementPtr elemSDF = _sdf->GetFirstElement();

  while (elemSDF)
  {
    if (elemSDF->GetName() == "rotor")
    {
      int channel;
      if (elemSDF->HasAttribute("channel"))
        elemSDF->GetAttribute("channel")->Get(channel);
      else
        gzthrow("[GzUavVehiclePlugin] <rotor channel=\"nn\">...</rotor> must be set");

      Rotor *rotor = new Rotor(this->model, elemSDF);
      this->rotors.emplace(rotor, channel);
    }
    else if (elemSDF->GetName() == "gimbal")
    {
      if (this->gimbal != nullptr)
        gzthrow("[GzUavVehiclePlugin] <gimbal> block cannot appear twice");

      this->gimbal = new Gimbal(this->model, elemSDF);
    }

    elemSDF = elemSDF->GetNextElement();
  }

  // Connect to gzuavchannel
  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  strncpy(addr.sun_path, getenv("GZUAV_UDS"), sizeof(addr.sun_path) - 1);

  this->sock = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (connect(this->sock, (struct sockaddr*)&addr, sizeof(addr)) == -1)
    gzthrow("[GzUavVehiclePlugin] connect failed");

  // Send IDENTIFY-UAV message
  std::string uav_name = _model->GetName();
  send(this->sock, uav_name.c_str(), uav_name.length(), 0);

  // Request OnUpdate callbacks
  this->updateConnection1 = GzUavPhaseGenerator::instance()->updateBegin1.Connect(
    std::bind(&GzUavVehiclePlugin::OnUpdateBegin1, this));
  this->updateConnection3 = GzUavPhaseGenerator::instance()->updateBegin3.Connect(
    std::bind(&GzUavVehiclePlugin::OnUpdateBegin3, this));
}

void GzUav::GzUavVehiclePlugin::OnUpdateBegin1()
{
  ssize_t r;

  // Receive END-TICK-AC packet
  struct packetEndTickAC endTickPkt;

  r = recv(this->sock,
           reinterpret_cast<void *>(&endTickPkt),
           sizeof(endTickPkt),
           0);

  if (r != sizeof(endTickPkt))
    gzthrow("[GzUavVehiclePlugin] recv failed");

  for (std::pair<Rotor*, int> it : rotors)
  {
    Rotor *rotor = it.first;
    int channel = it.second;

    rotor->ApplyMotorCommand(endTickPkt.motorCommands[channel]);
  }

  if (this->gimbal != nullptr)
  {
    this->gimbalSample = this->gimbal->Update(
        endTickPkt.gimbalRPY[0],
        endTickPkt.gimbalRPY[1],
        endTickPkt.gimbalRPY[2]
    );
  }
}

void GzUav::GzUavVehiclePlugin::OnUpdateBegin3()
{
  ssize_t r;

  // Send BEGIN-TICK-AC packet
  struct packetBeginTickAC beginTickPkt;

  beginTickPkt.timestamp = this->model->GetWorld()->SimTime().Double();
  beginTickPkt.vehiclePose = this->poseSampler.Sample();
  beginTickPkt.gimbalOrientation = this->gimbalSample;

  r = send(this->sock,
           reinterpret_cast<const void *>(&beginTickPkt),
           sizeof(beginTickPkt),
           0);

  if (r != sizeof(beginTickPkt))
    gzthrow("[GzUavVehiclePlugin] send failed");
}
