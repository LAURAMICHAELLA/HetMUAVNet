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
#ifndef GZUAV_COMMON_HH_
#define GZUAV_COMMON_HH_

#include <sdf/sdf.hh>
#include <gazebo/common/common.hh>

#define MAX_MOTORS 16

namespace gazebo
{
namespace GzUav
{
  /// \brief Obtains a parameter from sdf.
  /// \param[in] _sdf Pointer to the sdf object.
  /// \param[in] _name Name of the parameter.
  /// \param[out] _param Param Variable to write the parameter to.
  /// \param[in] _default_value Default value, if the parameter not available.
  /// \param[in] _verbose If true, gzerror if the parameter is not available.
  /// \return True if the parameter was found in _sdf, false otherwise.
  template<class T>
  bool getSdfParam(sdf::ElementPtr _sdf, const std::string &_name,
    T &_param, const T &_defaultValue, const bool &_verbose = false)
  {
    if (_sdf->HasElement(_name))
    {
      _param = _sdf->GetElement(_name)->Get<T>();
      return true;
    }

    _param = _defaultValue;

    if (_verbose)
    {
      gzerr << "[GzUavVehiclePlugin] Please specify a value for parameter ["
        << _name << "].\n";
    }
    return false;
  }

  /// \brief Reads all PID parameters from sdf.
  void getSdfPidParams(sdf::ElementPtr _sdf, const std::string &_prefix,
    common::PID &_pid);

  /// \brief A struct containing pose and other dynamics info.
  struct poseSample
  {
    /// \brief Model position in world frame
    double positionXYZ_world[3];

    /// \brief IMU angular velocity
    double imuAngularVelocityRPY[3];

    /// \brief IMU linear acceleration
    double imuLinearAccelerationXYZ[3];

    /// \brief IMU quaternion orientation
    double imuOrientationQuat[4];

    /// \brief Model velocity in NED frame
    double velocityXYZ[3];

    /// \brief Model position in NED frame
    double positionXYZ[3];
  };

  /// \brief A struct containing current gimbal angles.
  struct gimbalOrientationSample
  {
    /// \brief Gimbal orientation (yaw=0 is north)
    double gimbalRPY[3];
  };

  /// \brief An outgoing packet (sent at the beginning of each simulation step).
  struct packetBeginTickAC
  {
    /// \brief Packet timestamp
    double timestamp;

    /// \brief Vehicle pose and other dynamics info
    poseSample vehiclePose;

    /// \brief Gimbal orientation.
    gimbalOrientationSample gimbalOrientation;
  };

  /// \brief An incoming packet (received at the end of each simulation step).
  struct packetEndTickAC
  {
    /// \brief Motor commands sent by ArduCopter.
    float motorCommands[MAX_MOTORS];

    /// \brief Gimbal setpoint (yaw=0 is north)
    float gimbalRPY[3];
  };
}
}
#endif
