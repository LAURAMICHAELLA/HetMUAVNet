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
#include "GzUavVehiclePlugin/PoseSampler.hh"

using namespace gazebo;

GzUav::PoseSampler::PoseSampler()
{
}

GzUav::PoseSampler::~PoseSampler()
{
}

void GzUav::PoseSampler::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Store pointer to the model
  this->model = _model;

  // Locate IMU sensor
  std::string imuName;
  getSdfParam<std::string>(_sdf, "imu_name", imuName, "imu_sensor");
  std::string imuScopedName = this->model->GetWorld()->Name() + "::" +
                              this->model->GetScopedName() + "::" + imuName;
  this->imuSensor = std::dynamic_pointer_cast<sensors::ImuSensor>
    (sensors::SensorManager::Instance()->GetSensor(imuScopedName));

  if (!imuSensor)
    gzthrow("[GzUavVehiclePlugin] IMU sensor not found");
}

struct GzUav::poseSample GzUav::PoseSampler::Sample()
{
  struct GzUav::poseSample pose;

  this->imuSensor->Update(true);

  // asssumed that the imu orientation is:
  //   x forward
  //   y right
  //   z down

  // get linear acceleration in body frame
  ignition::math::Vector3d linearAccel = this->imuSensor->LinearAcceleration();
  // gzerr << "lin accel [" << linearAccel << "]\n";

  // copy to pose
  pose.imuLinearAccelerationXYZ[0] = linearAccel.X();
  pose.imuLinearAccelerationXYZ[1] = linearAccel.Y();
  pose.imuLinearAccelerationXYZ[2] = linearAccel.Z();

  // get angular velocity in body frame
  ignition::math::Vector3d angularVel = this->imuSensor->AngularVelocity();

  // copy to pose
  pose.imuAngularVelocityRPY[0] = angularVel.X();
  pose.imuAngularVelocityRPY[1] = angularVel.Y();
  pose.imuAngularVelocityRPY[2] = angularVel.Z();

  // get inertial pose and velocity
  // position of the quadrotor in world frame
  // this position is used to calcualte bearing and distance
  // from starting location, then use that to update gps position.
  // The algorithm looks something like below (from ardupilot helper
  // libraries):
  //   bearing = to_degrees(atan2(position.y, position.x));
  //   distance = math.sqrt(self.position.x**2 + self.position.y**2)
  //   (self.latitude, self.longitude) = util.gps_newpos(
  //    self.home_latitude, self.home_longitude, bearing, distance)
  // where xyz is in the NED directions.
  // Gazebo world xyz is assumed to be N, -E, -D, so flip some stuff
  // around.
  // orientation of the quadrotor in world NED frame -
  // assuming the world NED frame has xyz mapped to NED,
  // imuLink is NED - z down

  // If Gazebo world is rotated (i.e. X axis is not North), this transform
  // fixes it so that the above comment still applies
  ignition::math::Pose3d worldYaw(0, 0, 0, 0, 0,
    this->model->GetWorld()->SphericalCoords()->HeadingOffset().Radian());

  // gazeboToNED brings us from gazebo model: x-forward, y-right, z-down
  // to the aerospace convention: x-forward, y-left, z-up
  ignition::math::Pose3d gazeboToNED(0, 0, 0, IGN_PI, 0, 0);

  // model world pose brings us to model, x-forward, y-left, z-up
  // adding gazeboToNED gets us to the x-forward, y-right, z-down
  ignition::math::Pose3d worldPose = this->model->GetLink()->WorldPose();
  ignition::math::Pose3d worldToModel = gazeboToNED + worldPose;

  // get transform from world NED to Model frame
  ignition::math::Pose3d NEDToModel = worldToModel - gazeboToNED + worldYaw;

  // positionXYZ (NED)
  pose.positionXYZ[0] = NEDToModel.Pos().X();
  pose.positionXYZ[1] = NEDToModel.Pos().Y();
  pose.positionXYZ[2] = NEDToModel.Pos().Z();

  // imuOrientationQuat is the rotation from world NED frame
  // to the quadrotor frame.
  pose.imuOrientationQuat[0] = NEDToModel.Rot().W();
  pose.imuOrientationQuat[1] = NEDToModel.Rot().X();
  pose.imuOrientationQuat[2] = NEDToModel.Rot().Y();
  pose.imuOrientationQuat[3] = NEDToModel.Rot().Z();

  // gzdbg << "imu [" << worldToModel.rot.GetAsEuler() << "]\n";
  // gzdbg << "ned [" << gazeboToNED.rot.GetAsEuler() << "]\n";
  // gzdbg << "rot [" << NEDToModel.rot.GetAsEuler() << "]\n";

  // Get NED velocity in body frame *
  // or...
  // Get model velocity in NED frame
  ignition::math::Vector3d velGazeboWorldFrame = this->model->GetLink()->WorldLinearVel();
  ignition::math::Vector3d velNEDFrame =
    worldYaw.Rot().RotateVector(gazeboToNED.Rot().RotateVectorReverse(velGazeboWorldFrame));
  pose.velocityXYZ[0] = velNEDFrame.X();
  pose.velocityXYZ[1] = velNEDFrame.Y();
  pose.velocityXYZ[2] = velNEDFrame.Z();

  // positionXYZ_world
  pose.positionXYZ_world[0] = worldPose.Pos().X();
  pose.positionXYZ_world[1] = worldPose.Pos().Y();
  pose.positionXYZ_world[2] = worldPose.Pos().Z();

  return pose;
}
