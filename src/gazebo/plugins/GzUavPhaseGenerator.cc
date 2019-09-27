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
#include "GzUavPhaseGenerator.hh"

using namespace gazebo;

GzUav::GzUavPhaseGenerator::GzUavPhaseGenerator()
{
    updateConnectionBegin = event::Events::ConnectWorldUpdateBegin(std::bind(
        &GzUavPhaseGenerator::OnWorldUpdateBegin, this, std::placeholders::_1));
    updateConnectionEnd = event::Events::ConnectWorldUpdateEnd(std::bind(
        &GzUavPhaseGenerator::OnWorldUpdateEnd, this));
}

GzUav::GzUavPhaseGenerator *GzUav::GzUavPhaseGenerator::instance()
{
    // init on first call
    static GzUavPhaseGenerator inst;

    return &inst;
}

void GzUav::GzUavPhaseGenerator::OnWorldUpdateBegin(const common::UpdateInfo &info)
{
    updateBegin1(info);
    updateBegin2(info);
    updateBegin3(info);
}

void GzUav::GzUavPhaseGenerator::OnWorldUpdateEnd()
{
    updateEnd();
}
