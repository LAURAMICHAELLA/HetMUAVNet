/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
 * Copyright (C) 2018 Fabio D'Urso <durso@dmi.unict.it>
 *
 * Based on "A basic gimbal controller" (PX4/sitl_gazebo) by John Hsu
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
#ifndef GZUAV_GIMBAL_HH_
#define GZUAV_GIMBAL_HH_

#include "GzUavVehiclePlugin/common.hh"

#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

namespace gazebo
{
namespace GzUav
{
  class Gimbal
  {
    /// \brief Constructor.
    public: Gimbal(physics::ModelPtr _model, sdf::ElementPtr gimbalSDF);

    /// \brief Destructor.
    public: ~Gimbal();

    /// \brief TODO
    public: struct gimbalOrientationSample Update(double target_roll,
        double target_pitch, double target_yaw);

    private: common::PID rollPid;
    private: common::PID pitchPid;
    private: common::PID yawPid;

    private: physics::JointPtr rollJoint;
    private: physics::JointPtr pitchJoint;
    private: physics::JointPtr yawJoint;

    /// \brief Pointer to an IMU sensor
    private: sensors::ImuSensorPtr imuSensor;

    /// \brief Simulation time of last OnUpdate call.
    private: common::Time prevTime;
  };
}
}
#endif
