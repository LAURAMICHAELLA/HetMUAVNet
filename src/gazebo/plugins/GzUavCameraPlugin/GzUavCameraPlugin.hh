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
#ifndef GZUAV_GZUAVCAMERAPLUGIN_HH_
#define GZUAV_GZUAVCAMERAPLUGIN_HH_

#include "GzUavCameraPlugin/FrameServer.hh"

#include <gazebo/plugins/CameraPlugin.hh>

namespace gazebo
{
namespace GzUav
{
  class GAZEBO_VISIBLE GzUavCameraPlugin : public CameraPlugin
  {
    /// \brief Constructor.
    public: GzUavCameraPlugin();

    // Documentation Inherited.
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    /// \brief Simulation begin step callback method.
    private: void OnUpdateBegin(const common::UpdateInfo &info);

    /// \brief Simulation end step callback method.
    private: void OnUpdateEnd();

    /// \brief Pointers to the update event connections.
    private: event::ConnectionPtr updateConnectionBegin, updateConnectionEnd;

    /// \brief Current and previous simulation update times.
    private: common::Time currentTime, previousTime;

    /// \brief Update rate set by the user and last simulation time that tiggered the camera.
    private: common::Time period, previousTriggerTime;

    /// Mutex and condition variable to synchronise physics and rendering threads.
    private: std::mutex frameMutex;
    private: std::condition_variable frameCondition;

    /// \brief Output server.
    private: std::unique_ptr<FrameServer> frameServer;

    /// \brief Next frame to be emitted.
    private: std::unique_ptr<FrameServer::Frame> nextFrame;
  };
}
}
#endif
