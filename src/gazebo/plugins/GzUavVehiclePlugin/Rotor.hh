/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GZUAV_ROTOR_HH_
#define GZUAV_ROTOR_HH_

#include "GzUavVehiclePlugin/common.hh"

#include <gazebo/physics/physics.hh>

namespace gazebo
{
namespace GzUav
{
  class Rotor
  {
    /// \brief Constructor.
    public: Rotor(physics::ModelPtr _model, sdf::ElementPtr rotorSDF);

    /// \brief Destructor.
    public: ~Rotor();

    /// \brief Apply motor command.
    public: void ApplyMotorCommand(float command);

    /// \brief Max rotor propeller RPM.
    private: double maxRpm;

    /// \brief Velocity PID for motor control.
    private: common::PID pid;

    /// \brief Control propeller joint.
    private: physics::JointPtr joint;

    /// \brief Direction multiplier for this rotor.
    private: double multiplier;

    /// \brief Simulation time of last ApplyMotorCommand call.
    private: common::Time prevTime;
  };
}
}
#endif
