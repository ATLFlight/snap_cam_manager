/****************************************************************************
 *   Copyright (c) 2017 Stephen Chaves. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMIED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "ImageStreamer.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>

#include "SnapdragonCameraManager.hpp"

void *StreamerProcessingLoop(void *context)
{
  ImageStreamer *img_strmr_ptr = static_cast<ImageStreamer*>(context);

  if (img_strmr_ptr == NULL)
  {
    printf("ERROR: img_strmr_ptr is NULL\n");
    pthread_exit(NULL);
  }

  static int32_t frame_id = -1;
  while (frame_id == -1)
  {
    // Wait for first frame to become available
    frame_id = img_strmr_ptr->GetCameraManagerPtr()->GetNewestFrameId();
  }

  while (img_strmr_ptr->IsRunning())
  {
    // Check which type of images are configured
    if (img_strmr_ptr->GetCameraParamsPtr()->camera_config.cam_type ==
        Snapdragon::CameraType::STEREO)
    {
      // Stream stereo images
      img_strmr_ptr->StreamStereoImage();
    }
    else
    {
      // Stream camera image
      img_strmr_ptr->StreamImage();
    }

    // Receive commands here
    img_strmr_ptr->ReceiveCommand();

    // Check connection
    img_strmr_ptr->CheckConnection();

  } // end while
  pthread_exit(NULL);
}

int ImageStreamer::Initialize(Snapdragon::CameraManager* camera_manager,
    TcpServer* tcp_server, Snapdragon::CameraParameters* params_ptr)
{
  if (!initialized_)
  {
    camera_manager_ptr_ = camera_manager;
    tcp_server_ptr_ = tcp_server;
    params_ptr_ = params_ptr;

    initialized_ = true;
  }
  else
  {
    printf("WARN: ImageStreamer is already initialized.\n");
  }
  return 0;
}

int ImageStreamer::CheckConnection()
{
  if (initialized_ && running_)
  {
    if (tcp_server_ptr_->CheckSocket() < 0)
    {
      printf("ERROR: Error with TCP connection\n");
      running_ = false;
      return -1;
    }
  }
  return 0;
}

int ImageStreamer::StartProcessing()
{
  if (initialized_)
  {
    if (!running_)
    {
      printf("Creating TCP processing thread.\n");
      int ret = pthread_create(&streamer_main_thread_, NULL, StreamerProcessingLoop,
          static_cast<void*>(this));

      if (ret != 0)
      {
        printf("ERROR: TCP thread creation failed with code \n");
        return -1;
      }
      running_ = true;
    }
    else
    {
      printf("WARN: ImageStreamer has already started processing.\n");
      return -1;
    }
  }
  else
  {
    printf("ImageStreamer has not yet been initialized.\n");
    return -1;
  }
  return 0;
}

void ImageStreamer::ReceiveCommand()
{
  //
  // Receive any messages
  int ret_recv = tcp_server_ptr_->RecvMessage();

  // Arbitrarily chosen codes for gain +/- and exposure +/-
  if (ret_recv == 190 || ret_recv == 189 ||
      ret_recv == 192 || ret_recv == 191)
  {
    float gain = camera_manager_ptr_->GetGainValue();
    float exposure = camera_manager_ptr_->GetExposureValue();
    printf("Gain is set to %f, Exposure is set to %f\n", gain, exposure);

    if (ret_recv == 190) { gain += 0.05; }
    else if (ret_recv == 189) { gain -= 0.05; }
    else if (ret_recv == 192) { exposure += 0.05; }
    else if (ret_recv == 191) { exposure -= 0.05; }

    gain = std::max( (float) 0.0, std::min( gain, (float) 1.0 ) );
    exposure = std::max( (float) 0.0, std::min( exposure, (float) 1.0 ) );

    printf("Setting gain to %f and exposure to %f...\n", gain, exposure);
    camera_manager_ptr_->SetManualGain( gain );
    camera_manager_ptr_->SetManualExposure( exposure );

  }
  return;
}

void ImageStreamer::StreamImage()
{
  //
  uint64_t timestamp_ns = 0;
  uint32_t used = 0;
  size_t buffer_size = camera_manager_ptr_->GetImageSize();
  uint8_t* image_buffer = new uint8_t[buffer_size];
  uint8_t opts[4] = {0, 0, 0, 0};

  int64_t frame_id = camera_manager_ptr_->GetNewestFrameId();
  int32_t ret = camera_manager_ptr_->GetNextImageData(frame_id, &timestamp_ns,
      buffer_size, &used, image_buffer);

  if (ret == -1)
  {
    printf("ERROR: CameraManager has stopped running\n");
  }
  if (ret == -2)
  {
    printf("ERROR: ImageStreamer thread failed to get image %lld from queue; already lost!\n",
        frame_id);
    frame_id = camera_manager_ptr_->GetOldestFrameId();
    printf("ERROR: Reset frame id to %lld\n", frame_id);
  }
  else if (ret == -3)
  {
    printf("ERROR: ImageStreamer thread failed to allocate enough space to receive image \n");
  }
  else
  {
    if (frame_id % stream_nth_ == 0)
    {
      // Got a new image
      buffer_size = params_ptr_->camera_config.pixel_width *
        params_ptr_->camera_config.pixel_height;
      tcp_server_ptr_->SendMessage(image_buffer, buffer_size*sizeof(uint8_t), 2,
          params_ptr_->camera_config.pixel_width,
          params_ptr_->camera_config.pixel_height, &opts[0], frame_id, timestamp_ns);
    }
  }

  delete[] image_buffer;
  return;
}

void ImageStreamer::StreamStereoImage()
{
  //
  uint64_t timestamp_ns = 0;
  uint32_t used = 0;
  size_t buffer_size = camera_manager_ptr_->GetImageSize();
  uint8_t* left_buffer = new uint8_t[buffer_size];
  uint8_t* right_buffer = new uint8_t[buffer_size];
  uint8_t opts[4] = {0, 0, 0, 0};

  int64_t frame_id = camera_manager_ptr_->GetNewestFrameId();
  int32_t ret = camera_manager_ptr_->GetNextStereoImageData(frame_id, &timestamp_ns,
      buffer_size, &used, left_buffer, right_buffer);

  if (ret == -1)
  {
    printf("ERROR: CameraManager has stopped running\n");
  }
  if (ret == -2)
  {
    printf("ERROR: ImageStreamer thread failed to get image %lld from queue; already lost!\n",
        frame_id);
    frame_id = camera_manager_ptr_->GetOldestFrameId();
    printf("ERROR: Reset frame id to %lld\n", frame_id);
  }
  else if (ret == -3)
  {
    printf("ERROR: ImageStreamer thread failed to allocate enough space to receive image \n");
  }
  else
  {
    if (frame_id % stream_nth_ == 0)
    {
      // Got a new image
      buffer_size = params_ptr_->camera_config.pixel_width *
        params_ptr_->camera_config.pixel_height;

      tcp_server_ptr_->SendMessage(left_buffer, buffer_size*sizeof(uint8_t), 20,
          params_ptr_->camera_config.pixel_width,
          params_ptr_->camera_config.pixel_height, &opts[0], frame_id, timestamp_ns);

      tcp_server_ptr_->SendMessage(right_buffer, buffer_size*sizeof(uint8_t), 21,
          params_ptr_->camera_config.pixel_width,
          params_ptr_->camera_config.pixel_height, &opts[0], frame_id, timestamp_ns);
    }
  }

  delete[] left_buffer;
  delete[] right_buffer;
  return;
}

