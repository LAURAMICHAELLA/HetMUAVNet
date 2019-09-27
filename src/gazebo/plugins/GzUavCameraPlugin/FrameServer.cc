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
#include "GzUavCameraPlugin/FrameServer.hh"

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

using namespace gazebo;

static void SendAll(int fd, const uint8_t *data, size_t len)
{
    while (len != 0)
    {
        int r = send(fd, data, len, MSG_NOSIGNAL);
        if (r < 0)
            return; // failure

        data += r;
        len -= r;
    }
}

GzUav::FrameServer::FrameServer(const char *bindAddress, unsigned short bindPort, const char *imageDescription)
: imageDescription(imageDescription)
{
   // Start TCP server
    struct sockaddr_in addr;
    int opt_val = 1;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(bindPort);
    addr.sin_addr.s_addr = inet_addr(bindAddress);

    this->serv = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(this->serv, SOL_SOCKET, SO_REUSEADDR, &opt_val, sizeof(int));
    if (bind(this->serv, (struct sockaddr*)&addr, sizeof(addr)) < 0)
        gzthrow("[GzUavCameraPlugin] bind failed");

    listen(this->serv, 1);
    this->sock = -1;

    // Start server thread
    pthread_mutex_init(&this->nextFrameMutex, nullptr);
    pthread_create(&this->serverThread, nullptr, ThreadFunc, (void*)this);
}

GzUav::FrameServer::~FrameServer()
{
    // Stop server thread
    pthread_cancel(this->serverThread);
    pthread_join(this->serverThread, nullptr);

    pthread_mutex_destroy(&this->nextFrameMutex);
}

void GzUav::FrameServer::pushFrame(Frame *frame)
{
    pthread_mutex_lock(&this->nextFrameMutex);
    this->nextFrame.reset(frame);
    pthread_mutex_unlock(&this->nextFrameMutex);
}

void GzUav::FrameServer::ServerThread()
{
    // Wait for the client
    while (this->sock < 0)
    {
        struct sockaddr_in addr;
        socklen_t len = sizeof(addr);
        this->sock = accept(this->serv, (struct sockaddr*)&addr, &len);
    }

    // Stop accepting connections
    close(this->serv);

    // Send image description
    SendBlock(imageDescription.c_str(), imageDescription.length());

    // Reply to image requests
    while (true)
    {
        int dummy;
        if (recv(this->sock, &dummy, 1, 0) != 1)
            return;

        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &dummy);

        // This block cannot be canceled by pthread_cancel
        {
            pthread_mutex_lock(&this->nextFrameMutex);
            Frame *frame = this->nextFrame.release();
            pthread_mutex_unlock(&this->nextFrameMutex);

            if (frame != nullptr)
            {
                // Append timestamp to image data
                double ts = frame->timestamp.Double();
                unsigned char *tsBytes = (unsigned char*)&ts;
                frame->data.insert(frame->data.end(), tsBytes, tsBytes + sizeof(ts));

                SendBlock(frame->data.data(), frame->data.size());
                delete frame;
            }
            else
            {
                // No image is available, send empty response
                SendBlock(nullptr, 0);
            }
        }

        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, &dummy);
    }
}

void *GzUav::FrameServer::ThreadFunc(void *me)
{
    ((FrameServer*)me)->ServerThread();
    return nullptr;
}

void GzUav::FrameServer::SendBlock(const void *data, size_t len)
{
    // Enable TCP_CORK
    int opt_val = 1;
    setsockopt(this->sock, IPPROTO_TCP, TCP_CORK, &opt_val, sizeof(opt_val));

    // Send data length as a uint64_t followed by actual data
    uint64_t len64 = len;
    SendAll(this->sock, (const uint8_t*)&len64, sizeof(uint64_t));
    if (len > 0)
        SendAll(this->sock, (const uint8_t*)data, len);

    // Disable TCP_CORK (this has the effect of immediatly flushing data)
    opt_val = 0;
    setsockopt(this->sock, IPPROTO_TCP, TCP_CORK, &opt_val, sizeof(opt_val));
}
