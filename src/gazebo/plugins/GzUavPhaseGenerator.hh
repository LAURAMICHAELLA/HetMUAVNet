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
#ifndef GZUAV_GZUAVPHASEGENERATOR_HH_
#define GZUAV_GZUAVPHASEGENERATOR_HH_

#include <gazebo/common/common.hh>

namespace gazebo
{
namespace GzUav
{
  class GAZEBO_VISIBLE GzUavPhaseGenerator
  {
    /// \brief Return the singleton instance.
    public: static GzUavPhaseGenerator *instance();

    /// \brief First WorldUpdateBegin event.
    public: event::EventT<void (const common::UpdateInfo &info)> updateBegin1;

    /// \brief Second WorldUpdateBegin event.
    public: event::EventT<void (const common::UpdateInfo &info)> updateBegin2;

    /// \brief Third WorldUpdateBegin event.
    public: event::EventT<void (const common::UpdateInfo &info)> updateBegin3;

    /// \brief WorldUpdateEnd event.
    public: event::EventT<void ()> updateEnd;

    /// \brief Constructor.
    private: GzUavPhaseGenerator();

    /// \brief WorldUpdateBegin callback.
    private: void OnWorldUpdateBegin(const common::UpdateInfo &info);

    /// \brief WorldUpdateEnd callback.
    private: void OnWorldUpdateEnd();

    /// \brief Pointers to the update event connections.
    private: event::ConnectionPtr updateConnectionBegin, updateConnectionEnd;
  };
}
}
#endif
