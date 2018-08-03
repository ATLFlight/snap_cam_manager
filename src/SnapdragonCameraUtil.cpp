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
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "SnapdragonCameraUtil.hpp"
#include "camera.h"

#ifdef QC_SOC_TARGET_APQ8074
int32_t Snapdragon::FindCamera( Snapdragon::CameraType cam_type, int32_t* camera_id )
{
  int num_cams = camera::getNumberOfCameras();

  if (num_cams < 1)
  {
    printf("ERROR: No cameras detected. Exiting.\n");
    return -1;
  }

  bool found = false;

  for (int i = 0; i < num_cams; ++i)
  {
    camera::CameraInfo info;
    getCameraInfo(i, info);
    if (info.func == static_cast<int>(cam_type))
    {
      *camera_id = i;
      found = true;
    }
  }

  if (!found)
  {
    printf("ERROR: Could not find camera of type %d. Exiting\n", cam_type);
    return -1;
  }
  printf("Camera of type %d has ID = %d\n", cam_type, *camera_id);

  return 0;
}
#endif

#ifdef QC_SOC_TARGET_APQ8096
int32_t Snapdragon::FindCamera( Snapdragon::CameraType cam_type, int32_t* camera_id )
{
  if( cam_type == Snapdragon::CameraType::RIGHT_STEREO )
  {
    *camera_id = static_cast<int>(Snapdragon::CameraType::STEREO);
  }
  else
  {
    *camera_id = static_cast<int>(cam_type);
  }
  
  return 0;
}
#endif

