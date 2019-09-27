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
#include "GzUavVehiclePlugin/common.hh"

using namespace gazebo;

void GzUav::getSdfPidParams(sdf::ElementPtr _sdf, const std::string &_prefix,
  common::PID &_pid)
{
  // Overload the PID parameters if they are available.
  double param;
  getSdfParam<double>(_sdf, _prefix + "_p_gain", param, _pid.GetPGain());
  _pid.SetPGain(param);

  getSdfParam<double>(_sdf, _prefix + "_i_gain", param, _pid.GetIGain());
  _pid.SetIGain(param);

  getSdfParam<double>(_sdf, _prefix + "_d_gain", param,  _pid.GetDGain());
  _pid.SetDGain(param);

  getSdfParam<double>(_sdf, _prefix + "_i_max", param, _pid.GetIMax());
  _pid.SetIMax(param);

  getSdfParam<double>(_sdf, _prefix + "_i_min", param, _pid.GetIMin());
  _pid.SetIMin(param);

  getSdfParam<double>(_sdf, _prefix + "_cmd_max", param, _pid.GetCmdMax());
  _pid.SetCmdMax(param);

  getSdfParam<double>(_sdf, _prefix + "_cmd_min", param, _pid.GetCmdMin());
  _pid.SetCmdMin(param);

  // Set PID initial command
  _pid.SetCmd(0.0);
}
