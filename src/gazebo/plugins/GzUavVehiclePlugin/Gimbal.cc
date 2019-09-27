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
#include "GzUavVehiclePlugin/Gimbal.hh"

using namespace gazebo;

static void loadAxis(physics::ModelPtr model, sdf::ElementPtr axisSDF,
  common::PID &pid, physics::JointPtr &joint)
{
  GzUav::getSdfPidParams(axisSDF, "vel", pid);

  // Get pointer to the joint
  std::string jointName;
  GzUav::getSdfParam<std::string>(axisSDF, "joint_name", jointName, "<missing joint_name>", true);
  joint = model->GetJoint(jointName);
  if (!joint)
    gzthrow("[GzUavVehiclePlugin] Gimbal joint not found");
}

static double normalizeAbout(double _angle, double reference)
{
  double diff = _angle - reference;
  // normalize diff about (-pi, pi], then add reference
  while (diff <= -M_PI)
  {
    diff += 2.0*M_PI;
  }
  while (diff > M_PI)
  {
    diff -= 2.0*M_PI;
  }
  return diff + reference;
}

static double shortestAngularDistance(double _from, double _to)
{
  return normalizeAbout(_to, _from) - _from;
}

static ignition::math::Vector3d threeAxisRot(
  double r11, double r12, double r21, double r31, double r32)
{
  return ignition::math::Vector3d(
    atan2( r31, r32 ),
    asin ( r21 ),
    atan2( r11, r12 ));
}

static ignition::math::Vector3d QtoZXY(const ignition::math::Quaterniond &_q)
{
  // taken from
  // http://bediyap.com/programming/convert-quaternion-to-euler-rotations/
  // case zxy:
  ignition::math::Vector3d result = threeAxisRot(
    -2*(_q.X()*_q.Y() - _q.W()*_q.Z()),
    _q.W()*_q.W() - _q.X()*_q.X() + _q.Y()*_q.Y() - _q.Z()*_q.Z(),
    2*(_q.Y()*_q.Z() + _q.W()*_q.X()),
    -2*(_q.X()*_q.Z() - _q.W()*_q.Y()),
    _q.W()*_q.W() - _q.X()*_q.X() - _q.Y()*_q.Y() + _q.Z()*_q.Z());
  return result;
}

GzUav::Gimbal::Gimbal(physics::ModelPtr _model, sdf::ElementPtr gimbalSDF)
{
  // Locate IMU sensor
  std::string imuName;
  getSdfParam<std::string>(gimbalSDF, "imu_name", imuName, "gimbal_imu_sensor");
  std::string imuScopedName = _model->GetWorld()->Name() + "::" +
                              _model->GetScopedName() + "::" + imuName;
  this->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
    (sensors::SensorManager::Instance()->GetSensor(imuScopedName));

  if (!this->imuSensor)
    gzthrow("[GzUavVehiclePlugin] Gimbal IMU sensor not found");

  // Set the IMU's reference X axis to point North
  this->imuSensor->SetWorldToReferenceOrientation(ignition::math::Quaterniond(
      0,
      0,
      M_PI + _model->GetWorld()->SphericalCoords()->HeadingOffset().Radian()
    ));

  // Configure axes
  this->rollPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  if (gimbalSDF->HasElement("roll"))
    loadAxis(_model, gimbalSDF->GetElement("roll"), this->rollPid, this->rollJoint);
  else
    gzthrow("[GzUavVehiclePlugin] Gimbal <roll> block is missing");

  this->pitchPid.Init(5, 0, 0, 0, 0, 0.3, -0.3);
  if (gimbalSDF->HasElement("pitch"))
    loadAxis(_model, gimbalSDF->GetElement("pitch"), this->pitchPid, this->pitchJoint);
  else
    gzthrow("[GzUavVehiclePlugin] Gimbal <pitch> block is missing");

  this->yawPid.Init(1, 2, 0, 0, 0, 1.0, -1.0);
  if (gimbalSDF->HasElement("yaw"))
    loadAxis(_model, gimbalSDF->GetElement("yaw"), this->yawPid, this->yawJoint);
  else
    gzthrow("[GzUavVehiclePlugin] Gimbal <yaw> block is missing");
}

GzUav::Gimbal::~Gimbal()
{
}

struct GzUav::gimbalOrientationSample GzUav::Gimbal::Update(double target_roll,
        double target_pitch, double target_yaw)
{
  common::Time curTime = this->rollJoint->GetWorld()->SimTime();

  this->imuSensor->Update(true);

  // Change sign of roll and pitch target value
  target_roll = -target_roll;
  target_yaw = -target_yaw;

  /// Get current joint angles (in sensor frame):

  /// currentAngleYPRVariable is defined in roll-pitch-yaw-fixed-axis
  /// and gimbal is constructed using yaw-roll-pitch-variable-axis
  ignition::math::Vector3d currentAngleYPRVariable(
    this->imuSensor->Orientation().Euler());
  ignition::math::Vector3d currentAnglePRYVariable(
    QtoZXY(ignition::math::Quaterniond(currentAngleYPRVariable)));

  if (prevTime != common::Time())
  {
    double dt = (curTime - prevTime).Double();

    // anything to do with gazebo joint has
    // hardcoded negative joint axis for pitch and roll
    // TODO: make joint direction a parameter
    const double rDir = -1;
    const double pDir = -1;
    const double yDir = 1;

    // truncate command inside joint angle limits
    double rollLimited = ignition::math::clamp(target_roll,
      rDir*this->rollJoint->UpperLimit(0),
	  rDir*this->rollJoint->LowerLimit(0));
    double pitchLimited = ignition::math::clamp(target_pitch,
      pDir*this->pitchJoint->UpperLimit(0),
      pDir*this->pitchJoint->LowerLimit(0));
    double yawLimited = ignition::math::clamp(target_yaw,
      yDir*this->yawJoint->LowerLimit(0),
	  yDir*this->yawJoint->UpperLimit(0));

    ignition::math::Quaterniond commandRPY(
      rollLimited, pitchLimited, yawLimited);

    /// get joint limits (in sensor frame)
    /// TODO: move to Load() if limits do not change
    ignition::math::Vector3d lowerLimitsPRY
      (pDir*this->pitchJoint->LowerLimit(0),
       rDir*this->rollJoint->LowerLimit(0),
       yDir*this->yawJoint->LowerLimit(0));
    ignition::math::Vector3d upperLimitsPRY
      (pDir*this->pitchJoint->UpperLimit(0),
       rDir*this->rollJoint->UpperLimit(0),
       yDir*this->yawJoint->UpperLimit(0));

    // normalize errors
    double pitchError = shortestAngularDistance(
      pitchLimited, currentAnglePRYVariable.X());
    double rollError = shortestAngularDistance(
      rollLimited, currentAnglePRYVariable.Y());
    double yawError = shortestAngularDistance(
      yawLimited, currentAnglePRYVariable.Z());

    // Clamp errors based on current angle and estimated errors from rotations:
    // given error = current - target, then
    // if target (current angle - error) is outside joint limit, truncate error
    // so that current angle - error is within joint limit, i.e.:
    // lower limit < current angle - error < upper limit
    // or
    // current angle - lower limit > error > current angle - upper limit
    // re-expressed as clamps:
    // hardcoded negative joint axis for pitch and roll
    if (lowerLimitsPRY.X() < upperLimitsPRY.X())
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - upperLimitsPRY.X(),
        currentAnglePRYVariable.X() - lowerLimitsPRY.X());
    }
    else
    {
      pitchError = ignition::math::clamp(pitchError,
        currentAnglePRYVariable.X() - lowerLimitsPRY.X(),
        currentAnglePRYVariable.X() - upperLimitsPRY.X());
    }
    if (lowerLimitsPRY.Y() < upperLimitsPRY.Y())
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y());
    }
    else
    {
      rollError = ignition::math::clamp(rollError,
        currentAnglePRYVariable.Y() - lowerLimitsPRY.Y(),
        currentAnglePRYVariable.Y() - upperLimitsPRY.Y());
    }
    if (lowerLimitsPRY.Z() < upperLimitsPRY.Z())
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z());
    }
    else
    {
      yawError = ignition::math::clamp(yawError,
        currentAnglePRYVariable.Z() - lowerLimitsPRY.Z(),
        currentAnglePRYVariable.Z() - upperLimitsPRY.Z());
    }

    // apply forces to move gimbal
    double pitchForce = this->pitchPid.Update(pitchError, dt);
    this->pitchJoint->SetForce(0, pDir*pitchForce);

    double rollForce = this->rollPid.Update(rollError, dt);
    this->rollJoint->SetForce(0, rDir*rollForce);

    double yawForce = this->yawPid.Update(yawError, dt);
    this->yawJoint->SetForce(0, yDir*yawForce);
  }

  prevTime = curTime;

  // Change sign of roll and pitch readings
  gimbalOrientationSample result;
  result.gimbalRPY[0] = -currentAnglePRYVariable.Y();
  result.gimbalRPY[1] = currentAnglePRYVariable.X();
  result.gimbalRPY[2] = -currentAnglePRYVariable.Z();
  return result;
}
