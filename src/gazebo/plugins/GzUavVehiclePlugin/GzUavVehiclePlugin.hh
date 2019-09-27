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
#ifndef GZUAV_GZUAVVEHICLEPLUGIN_HH_
#define GZUAV_GZUAVVEHICLEPLUGIN_HH_

#include "GzUavVehiclePlugin/common.hh"
#include "GzUavVehiclePlugin/Gimbal.hh"
#include "GzUavVehiclePlugin/PoseSampler.hh"
#include "GzUavVehiclePlugin/Rotor.hh"

namespace gazebo
{
namespace GzUav
{
  class GAZEBO_VISIBLE GzUavVehiclePlugin : public ModelPlugin
  {
    /// \brief Constructor.
    public: GzUavVehiclePlugin();

    /// \brief Destructor.
    public: ~GzUavVehiclePlugin();

    // Documentation Inherited.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Simulation step callback method (event 1).
    private: void OnUpdateBegin1();

    /// \brief Simulation step callback method (event 3).
    private: void OnUpdateBegin3();

    /// \brief Pointer to the model
    private: physics::ModelPtr model;

    /// \brief Unix domain socket connected to gzuavchannel.
    private: int sock;

    /// \brief Pointers to the update event connections.
    private: event::ConnectionPtr updateConnection1, updateConnection3;

    /// \brief PoseSampler component.
    private: PoseSampler poseSampler;

    /// \brief Propellers with their channel indices.
    private: std::map<Rotor*, int> rotors;

    /// \brief Gimbal (if present).
    private: Gimbal *gimbal;

    /// \brief Gimbal orientation.
    private: gimbalOrientationSample gimbalSample;
  };
}
}
#endif
