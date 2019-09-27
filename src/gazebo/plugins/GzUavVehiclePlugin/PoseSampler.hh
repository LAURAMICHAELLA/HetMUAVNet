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
#ifndef GZUAV_POSESAMPLER_HH_
#define GZUAV_POSESAMPLER_HH_

#include "GzUavVehiclePlugin/common.hh"

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
namespace GzUav
{
  class PoseSampler
  {
    /// \brief Constructor.
    public: PoseSampler();

    /// \brief Destructor.
    public: ~PoseSampler();

    /// \brief Initialize this sampler.
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Produce one pose sample according to the current vehicle state.
    public: struct poseSample Sample();

    /// \brief Pointer to the model;
    private: physics::ModelPtr model;

    /// \brief Pointer to an IMU sensor
    private: sensors::ImuSensorPtr imuSensor;
  };
}
}
#endif
