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
#include "GzUavCameraPlugin/GzUavCameraPlugin.hh"

#include "GzUavPhaseGenerator.hh"

#include <gazebo/rendering/rendering.hh>

#include <stdio.h>

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(GzUav::GzUavCameraPlugin)

GzUav::GzUavCameraPlugin::GzUavCameraPlugin()
: previousTime(common::Time::Zero),
  previousTriggerTime(common::Time::Zero)
{
}

void GzUav::GzUavCameraPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
    // Call base class
    CameraPlugin::Load(_sensor, _sdf);

    this->period = 1 / this->parentSensor->UpdateRate();

    // This plugin takes control of the rate control logic.
    this->parentSensor->SetUpdateRate(1e9);
    this->parentSensor->SetActive(false);

    updateConnectionBegin = GzUavPhaseGenerator::instance()->updateBegin2.Connect(
        std::bind(&GzUavCameraPlugin::OnUpdateBegin, this, std::placeholders::_1));
    updateConnectionEnd = GzUavPhaseGenerator::instance()->updateEnd.Connect(
        std::bind(&GzUavCameraPlugin::OnUpdateEnd, this));

    // Extract the root model's name from the fully scoped name of the sensor
    // e.g. worldname::rootmodelname::submodelname::linkname::sensorname
    std::string scopedName = _sensor->ScopedName();
    int modelBegin = scopedName.find("::");
    int modelEnd = scopedName.find("::", modelBegin + 2);
    std::string modelName(&scopedName[modelBegin + 2], &scopedName[modelEnd]);

    // Build image description string
    char imageDescr[128 + this->format.length()];
    sprintf(imageDescr, "%d %d %d %s", this->width, this->height, this->depth,
            this->format.c_str());

    // Start TCP server
    int port = atoi(getenv(("GZUAV_CAMBUFFER_PORT-" + modelName).c_str()));
    this->frameServer.reset(new FrameServer("0.0.0.0", port, imageDescr));
}

void GzUav::GzUavCameraPlugin::OnNewFrame(const unsigned char *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format)
{
    common::Time ts = this->camera->GetScene()->SimTime();
    size_t bytes = this->camera->ImageByteSize();

    std::unique_lock<std::mutex> lock(this->frameMutex);

    // If we just received a shoot for the desired timestamp, save it and resume
    // OnUpdate execution on the physics thread
    if (ts == this->previousTime)
    {
        FrameServer::Frame *frame = new FrameServer::Frame();
        frame->timestamp = ts;
        frame->data.assign(_image, _image + bytes);
        nextFrame.reset(frame);

        //fprintf(stderr, "CAM FRAME   : %.3f\n", ts.Double());

        this->parentSensor->SetActive(false);
        this->frameCondition.notify_one();
    }
}

void GzUav::GzUavCameraPlugin::OnUpdateBegin(const common::UpdateInfo &info)
{
    // Push previous frame (if present) to frameServer
    if (this->nextFrame != nullptr)
        this->frameServer->pushFrame(this->nextFrame.release());

    this->currentTime = info.simTime;

    // Attempting to shoot during the first step would result in a deadlock
    if (this->previousTime != common::Time::Zero)
    {
        if (this->currentTime < this->previousTime) // time was reset
        {
            this->previousTime = common::Time::Zero;
            this->previousTriggerTime = common::Time::Zero;
        }

        if (this->currentTime - this->previousTriggerTime >= this->period)
        {
            // Shoot a picture (the frame that we expect is the one whose
            // timestamp is equal to previousTime)
            this->previousTriggerTime = this->currentTime;
            this->parentSensor->SetActive(true);
        }
    }
}

void GzUav::GzUavCameraPlugin::OnUpdateEnd()
{
    std::unique_lock<std::mutex> lock(this->frameMutex);

    // Wait for the CameraSensor to complete our request
    while (this->parentSensor->IsActive())
        this->frameCondition.wait(lock);

    this->previousTime = this->currentTime;
}
