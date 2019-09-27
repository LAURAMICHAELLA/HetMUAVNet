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
#ifndef GZUAV_FRAMESERVER_HH_
#define GZUAV_FRAMESERVER_HH_

#include <gazebo/common/common.hh>

namespace gazebo
{
namespace GzUav
{
  class GAZEBO_VISIBLE FrameServer
  {
    /// \brief A timestamped frame.
    public: struct Frame
    {
        common::Time timestamp;
        std::vector<char> data;
    };

    /// \brief Constructor.
    public: FrameServer(const char *bindAddress, unsigned short bindPort, const char *imageDescription);

    /// \brief Destructor.
    public: ~FrameServer();

    /// \brief Set nextFrame.
    public: void pushFrame(Frame *frame);

    /// \brief Image description string (size and format).
    private: std::string imageDescription;

    /// \brief Mutex that synchronises access to nextFrame.
    private: pthread_mutex_t nextFrameMutex;

    /// \brief Frame to be served next.
    private: std::unique_ptr<Frame> nextFrame;

    // Output server
    private: void ServerThread();
    private: static void *ThreadFunc(void *me);
    private: void SendBlock(const void *data, size_t len);
    private: pthread_t serverThread;
    private: int serv, sock;
  };
}
}
#endif
