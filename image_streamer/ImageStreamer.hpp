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
#ifndef IMAGE_STREAMER_HPP_
#define IMAGE_STREAMER_HPP_

#include <cstdbool>
#include <cstring>
#include <deque>
#include <fstream>
#include <pthread.h>

#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraTypes.hpp"
#include "TcpUtils.hpp"

class ImageStreamer
{
public:
  /**
   * Constructor
   */
  ImageStreamer(bool verbose, int stream_nth)
    : verbose_(verbose), stream_nth_(stream_nth)
  {
    initialized_ = false;
    running_ = false;

    camera_manager_ptr_ = NULL;
    tcp_server_ptr_ = NULL;
  }

  /**
   * Initialize the ImageStreamer
   */
  int Initialize(Snapdragon::CameraManager*, TcpServer*,
                 Snapdragon::CameraParameters*);

  /**
   * Check the TCP/IP connection
   */
  int CheckConnection();

  /**
   * Start streaming images from the camera
   */
  int StartProcessing();

  /**
   * Reset
   */
  void Reset();

  /**
   * Check if the ImageStreamer is still running
   */
  inline bool IsRunning()
  {
    return running_;
  }

  /**
   * Stop streaming images
   */
  inline void StopProcessing()
  {
    running_ = false;
    // Block until thread returns
    pthread_join(streamer_main_thread_, NULL);
  }

  /**
   * Get the pointer to the CameraManager
   */
  inline Snapdragon::CameraManager* GetCameraManagerPtr()
  {
    return camera_manager_ptr_;
  }

  /**
   * Get the pointer to the TcpServer
   */
  inline TcpServer* GetTcpServerPtr()
  {
    return tcp_server_ptr_;
  }

  /**
   * Get the pointer to the CameraParameters
   */
  inline Snapdragon::CameraParameters* GetCameraParamsPtr()
  {
    return params_ptr_;
  }

  /**
   * Receive a keyboard command from the connected computer
   */
  void ReceiveCommand();

  /**
   * Stream an image by sending over TCP/IP connection
   */
  void StreamImage();

  /**
   * Stream a stereo image by sending over TCP/IP connection
   */
  void StreamStereoImage();

private:
  Snapdragon::CameraManager* camera_manager_ptr_;
  TcpServer* tcp_server_ptr_;
  Snapdragon::CameraParameters* params_ptr_;

  bool initialized_;
  bool running_;

  const bool verbose_;
  const int stream_nth_;

  pthread_t streamer_main_thread_;
};

#endif // IMAGE_STREAMER_HPP_
