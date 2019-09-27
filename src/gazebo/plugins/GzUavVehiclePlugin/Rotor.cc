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
#include "GzUavVehiclePlugin/Rotor.hh"

using namespace gazebo;

GzUav::Rotor::Rotor(physics::ModelPtr _model, sdf::ElementPtr rotorSDF)
{
  this->maxRpm = 838.0;

  // Get pointer to the joint
  std::string jointName;
  getSdfParam<std::string>(rotorSDF, "joint_name", jointName, "<missing joint_name>", true);
  this->joint = _model->GetJoint(jointName);
  if (!this->joint)
    gzthrow("[GzUavVehiclePlugin] Joint not found");

  // Get turning direction
  std::string turningDirection;
  getSdfParam<std::string>(rotorSDF, "turning_direction", turningDirection, "<missing turning_direction>", true);
  if (turningDirection == "cw")
    this->multiplier = -1;
  else if (turningDirection == "ccw")
    this->multiplier = 1;
  else
    gzthrow("[GzUavVehiclePlugin] Invalid turning direction");

  // Initialize PID with default values
  this->pid.Init(0.1, 0, 0, 0, 0, 1.0, -1.0);
  getSdfPidParams(rotorSDF, "vel", this->pid);
}

GzUav::Rotor::~Rotor()
{
}

void GzUav::Rotor::ApplyMotorCommand(float command)
{
  common::Time curTime = this->joint->GetWorld()->SimTime();

  if (prevTime != common::Time())
  {
    double velTarget = this->multiplier * this->maxRpm * command;
    double vel = this->joint->GetVelocity(0);
    double error = vel - velTarget;
    double force = this->pid.Update(error, (curTime - prevTime).Double());

    this->joint->SetForce(0, force);
  }

  prevTime = curTime;
}
