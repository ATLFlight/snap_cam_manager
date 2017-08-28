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
#include <iostream>
#include <signal.h>
#include <stdbool.h>
#include <syslog.h>
#include <unistd.h>

#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraTypes.hpp"
#include "ImageStreamer.hpp"

static void PrintUsage()
{
  std::cout << "-h                    print this message." << std::endl;
  std::cout << "-c [CAM=DFT]          0=HIRES, 1=DFT, 2=RIGHT_STEREO, 3=STEREO" << std::endl;
  std::cout << "-i [IP=192.168.1.1]   IP address of the server" << std::endl;
  std::cout << "-p [PORT=5556]        port number of the server" << std::endl;
  std::cout << std::endl;
  std::cout << "-e [EXP=0.36]         set exposure; EXP in [0.0,1.0]" << std::endl;
  std::cout << "-g [GAIN=0.35]        set gain; GAIN in [0.0,1.0]" << std::endl;
  std::cout << "-n [NUM=1]            stream every n-th image" << std::endl;
}

static bool caught_sig_int = false;
static void SigIntHandler(int sig)
{
  caught_sig_int = true;
}


int main(int argc, char* argv[])
{

  int c;
  char* ip_addr = (char*)"192.168.1.1";
  int port_num = 5556;
  bool is_stereo = false;
  int stream_nth = 1;

  Snapdragon::CameraParameters param;
  param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
  param.camera_config.cam_format = Snapdragon::CameraFormat::YUV_FORMAT;
  param.camera_config.fps = 30;

  // Getopt command line arguments
  while ((c = getopt(argc, argv, "c:i:p:he:g:n:")) != -1)
  {
    switch (c)
    {
      case 'h':
        PrintUsage();
        return 0;
      case 'c':
        switch (atoi(optarg))
        {
          case 0:
           std::cout << "HIRES camera selected." << std::endl;
           param.camera_config.cam_type = Snapdragon::CameraType::HIRES;
           param.camera_config.num_image_buffers = 3;
           break;
          case 1:
           std::cout << "DFT camera selected." << std::endl;
           param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
           break;
          case 2:
           std::cout << "RIGHT_STEREO camera selected." << std::endl;
           param.camera_config.cam_type = Snapdragon::CameraType::RIGHT_STEREO;
           break;
          case 3:
           std::cout << "STEREO camera selected." << std::endl;
           param.camera_config.cam_type = Snapdragon::CameraType::STEREO;
           is_stereo = true;
           break;
          default:
           std::cout << "No camera selected, using DFT." << std::endl;
           param.camera_config.cam_type = Snapdragon::CameraType::OPTIC_FLOW;
           break;
        }
        break;
      case 'i':
        ip_addr = optarg;
        break;
      case 'p':
        port_num = atoi(optarg);
        break;
      case 'e':
        param.camera_config.exposure = atof(optarg);
        break;
      case 'g':
        param.camera_config.gain = atof(optarg);
        break;
      case 'n':
        stream_nth = atoi(optarg);
        break;
      default:
        printf("ERROR: Unsupported option.\n");
        return -1;
    }
  }

  // CAMERA MANAGER
  Snapdragon::CameraManager camera( &param );

  if (camera.Initialize() != 0)
  {
    printf("ERROR: Error initializing CameraManager. Exiting.\n");
    return -1;
  }

  printf("Starting camera image streaming...\n");
  if (camera.Start() != 0)
  {
    printf("ERROR: Error starting camera streaming. Exiting.\n");
    return -1;
  }

  // TCP SERVER
  TcpServer server;
  server.CreateSocket(ip_addr, port_num);
  server.BindSocket();
  server.ConnectClient();

  // IMAGE STREAMER
  ImageStreamer img_strmr(false, stream_nth);
  if (img_strmr.Initialize(&camera, &server, &param) != 0)
  {
    printf("ERROR: Error initializing ImageStreamer. Exiting.\n");
  }
  if (img_strmr.StartProcessing() != 0)
  {
    printf("ERROR: Error starting ImageStreamer processing. Exiting.\n");
    return -1;
  }

  // MAIN LOOP
  signal(SIGINT, SigIntHandler);
  while (img_strmr.IsRunning() && camera.IsRunning() && !caught_sig_int)
  {
    usleep(1e5);
  }

  // CLEANUP
  printf("Stopping ImageStreamer processing...\n");
  img_strmr.StopProcessing();

  printf("Stopping CameraManager processing...\n");
  camera.Stop();
  camera.Terminate();

  printf("Done.\n");

  return 0;
}

