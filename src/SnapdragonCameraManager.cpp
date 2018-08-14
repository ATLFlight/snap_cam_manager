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
#include "SnapdragonCameraManager.hpp"
#include "SnapdragonCameraUtil.hpp"

#include <fstream>
#include <iostream>
#include <string>
#include <chrono>

Snapdragon::CameraManager::CameraManager( Snapdragon::CameraParameters* params_ptr) {
  initialized_ = false;
  running_     = false;

  snap_camera_param_ptr_ = params_ptr;
  camera_config_ptr_     = &params_ptr->camera_config;

  camera_ptr_           = NULL;
#ifdef QC_SOC_TARGET_APQ8096
  camera_sub_ptr_       = NULL;
#endif
  if( camera_config_ptr_->cam_type == CameraType::STEREO ) {
    preview_size_.width = camera_config_ptr_->pixel_width * 2;
  }
#ifdef QC_SOC_TARGET_APQ8096
  // for 8096, get full stereo and just copy half the frame
  else if( camera_config_ptr_->cam_type == CameraType::RIGHT_STEREO ) {
    preview_size_.width = camera_config_ptr_->pixel_width * 2;
  }
#endif
  else {
    preview_size_.width = camera_config_ptr_->pixel_width;
  }
  preview_size_.height  = camera_config_ptr_->pixel_height;
  image_size_bytes_     = camera_config_ptr_->pixel_height * camera_config_ptr_->memory_stride *1.5;
  exposure_setting_     = camera_config_ptr_->min_exposure
                          + camera_config_ptr_->exposure
                          * (camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure);

  gain_setting_         = camera_config_ptr_->min_gain + camera_config_ptr_->gain
                          * (camera_config_ptr_->max_gain - camera_config_ptr_->min_gain);

  exposure_target_      = exposure_setting_;
  gain_target_          = gain_setting_;
  fps_avg_              = 0;
  timestamp_last_nsecs_ = 0;

  next_frame_id_ = 0;
  frame_q_write_index_ = 0;
  frame_q_read_index_ = -1;

  for (unsigned int ii = 0; ii < camera_config_ptr_->num_image_buffers; ++ii)
  {
    CameraFrameType null_frame;
    frame_queue_.push_back( null_frame );
  }
}

int32_t Snapdragon::CameraManager::Initialize(){
  if (!initialized_) {
    int32_t cam_id;
printf( "%s %d\n", __FILE__, __LINE__ );
    if( Snapdragon::FindCamera( camera_config_ptr_->cam_type, &cam_id ) != 0 ) {
      printf( "ERROR: Cannot Find Camera Id for Type: %d\n", camera_config_ptr_->cam_type );
      return -1;
    }
printf( "%s %d\n", __FILE__, __LINE__ );
    if (camera_config_ptr_->is_cam_master)
    {
      int ret = camera::ICameraDevice::createInstance(cam_id, &camera_ptr_);
      if (ret != 0) {
        printf("ERROR: Could not open camera %d\n", cam_id);
        return ret;
      }
      else {
        printf("Opened camera %d Type: %d\n", cam_id, camera_config_ptr_->cam_type );
      }
printf( "%s %d\n", __FILE__, __LINE__ );
      camera_ptr_->addListener(this);
printf( "%s %d\n", __FILE__, __LINE__ );
      ret = params_.init(camera_ptr_);
      if (ret != 0) {
        printf("ERROR: Failed to initialize camera parameters.\n");
        camera::ICameraDevice::deleteInstance(&camera_ptr_);
        return ret;
      }

      // Check desired FPS against supported FPS values
      std::vector<camera::Range> preview_fps_ranges = params_.getSupportedPreviewFpsRanges();
      int fps_index = -1;
      for (unsigned int ii = 0; ii < preview_fps_ranges.size(); ++ii) {
        printf("Preview FPS range %d: [ %d, %d ]\n",
            ii, preview_fps_ranges[ii].min / 1000, preview_fps_ranges[ii].max / 1000);
        if (preview_fps_ranges[ii].max / 1000 == camera_config_ptr_->fps) {
          fps_index = static_cast<int>(ii);
        }
      }

      // Check desired preview size against supported sizes
      std::vector<camera::ImageSize> preview_sizes = params_.getSupportedPreviewSizes();
      int psize_index = -1;
      for (unsigned int ii = 0; ii < preview_sizes.size(); ++ii) {
        printf("Preview size %d: [ %d x %d ]\n",
            ii, preview_sizes[ii].width, preview_sizes[ii].height);
        if (preview_sizes[ii].width == preview_size_.width &&
            preview_sizes[ii].height == preview_size_.height) {
          psize_index = static_cast<int>(ii);
        }
      }

      // Print supported preview formats
      // Default format is YUV
      std::vector<std::string> preview_formats = params_.getSupportedPreviewFormats();
      for (unsigned int ii = 0; ii < preview_formats.size(); ++ii) {
        printf("Preview format %d: %s\n", ii, preview_formats[ii].c_str());
      }

      {
        std::lock_guard<std::mutex> lock( frame_mutex_ );

        if (fps_index != -1)
        {
          printf("Setting FPS to %d\n", camera_config_ptr_->fps);
          params_.setPreviewFpsRange(preview_fps_ranges[fps_index]);
        }
        else
        {
          printf("ERROR: Invalid FPS value of %d. Using camera default.\n",
              camera_config_ptr_->fps);
        }

        if (psize_index != -1)
        {
          printf("Setting preview size to %dx%d\n", preview_size_.width, preview_size_.height);
          params_.setPreviewSize(preview_size_);
        }
        else
        {
          printf("ERROR: Invalid preview size of %dx%d. Using camera default.\n",
              preview_size_.width, preview_size_.height);
        }

        if( camera_config_ptr_->cam_format == CameraFormat::RAW_FORMAT )
        {
          printf("Setting preview format to RAW_FORMAT\n");
          params_.set( "preview-format", "bayer-rggb" );
          params_.set( "picture-format", "bayer-mipi-10gbrg" );
          params_.set( "raw-size", "640x480" );
#ifdef QC_SOC_TARGET_APQ8096
          params_.setPreviewFormat(camera::FORMAT_RAW10);
#endif
        }
        else if( camera_config_ptr_->cam_format == CameraFormat::NV12_FORMAT )
        {
          printf("Setting preview format to NV12_FORMAT\n");
          params_.set( "preview-format", "nv12" );
        }
        else
        {
          printf("Using default preview format of YUV_FORMAT\n");
        }

#ifdef QC_SOC_TARGET_APQ8074
        char exposure_string[6];
        sprintf(exposure_string,"%d", exposure_setting_);
        char gain_string[6];
        sprintf(gain_string,"%d", gain_setting_);
        params_.set("qc-exposure-manual",exposure_string);
        params_.set("qc-gain-manual",gain_string);
#endif
#ifdef QC_SOC_TARGET_APQ8096
        params_.setManualExposure(exposure_setting_);
        params_.setManualGain(gain_setting_);
#endif

        // commit the camera parameters.
        if (params_.commit() != 0)
        {
          printf("ERROR: Error setting initial camera params.\n");
          return -1;
        }
      }
    }
    else
    {
#ifdef QC_SOC_TARGET_APQ8096
      bool ret = camera::ICameraSubscriber::createInstance(cam_id, camera_sub_ptr_);

      if (!ret) {
        printf("ERROR: could not open camera subscriber for cam id %d\n", cam_id);
        return -1;
      }
      camera_sub_ptr_->addListener(this);
#else
      printf("8074 does not support ICameraSubscriber");
#endif
    }

    initialized_ = true;
  }
  else {
    printf("WARN: CameraManager is already initialized.\n");
  }

  return 0;
}

int32_t Snapdragon::CameraManager::Terminate() {
  if( camera_ptr_ != nullptr ) {
    //stop the camera.
    if (running_) {
      Stop();
    }
    // remove this as a listener.
    camera_ptr_->removeListener( this );
    //delete the camera ptr
    camera::ICameraDevice::deleteInstance(&camera_ptr_);
    camera_ptr_ = nullptr;
  }
#ifdef QC_SOC_TARGET_APQ8096
  if (camera_sub_ptr_ != nullptr) {
    if (running_) {
      Stop();
    }
    camera_sub_ptr_->removeListener(this);
    camera::ICameraSubscriber::deleteInstance(camera_sub_ptr_);
    camera_sub_ptr_ = nullptr;
  }
#endif
  return 0;

}

int32_t Snapdragon::CameraManager::Start() {
  if (initialized_) {
    if (!running_) {
      if (camera_config_ptr_->is_cam_master)
      {
        int ret = camera_ptr_->startPreview();
        if (ret == 0) {
          running_ = true;
        }
        else {
          return ret;
        }
      }
      else {
#ifdef QC_SOC_TARGET_APQ8096
        if (camera_sub_ptr_->startPreview()) {
          running_ = true;
        }
        else { return -1; }
#else
      printf("8074 does not support ICameraSubscriber");
      return -1;
#endif
      }
    }
    else {
      printf( "WARN: CameraManager is already started.\n" );
    }
  }
  else {
    printf( "ERROR: CameraManager has not yet been initialized.\n"  );
    return -1;
  }

  return 0;
}

int32_t Snapdragon::CameraManager::Stop() {
  if( running_ ) {
    if (camera_config_ptr_->is_cam_master) {
      camera_ptr_->stopPreview();
    }
    else {
#ifdef QC_SOC_TARGET_APQ8096
      camera_sub_ptr_->stopPreview();
#else
      printf("8074 does not support ICameraSubscriber");
      return -1;
#endif
    }
  }
  running_ = false;

  std::lock_guard<std::mutex> lock( frame_mutex_ );
  //increment the frame id so that the condition variable is woken up.
  next_frame_id_++;
  frame_cv_.notify_all();

  //free the frames
  for (int ii = 0; ii < camera_config_ptr_->num_image_buffers; ++ii) {
    if( frame_queue_[ii].frame_ != nullptr ) {
      frame_queue_[ii].frame_->releaseRef();
      CameraFrameType null_frame;
      frame_queue_[ii] = null_frame;
    }
  }
  return 0;
}

void Snapdragon::CameraManager::SetManualExposure( float exposure )
{
  exposure_target_ = static_cast<int>(camera_config_ptr_->min_exposure + exposure
      * (camera_config_ptr_->max_exposure - camera_config_ptr_->min_exposure));
}

void Snapdragon::CameraManager::SetManualGain( float gain )
{
  gain_target_ = static_cast<int>(camera_config_ptr_->min_gain + gain
      * (camera_config_ptr_->max_gain - camera_config_ptr_->min_gain));
}

void Snapdragon::CameraManager::UpdateGainAndExposure()
{

  static std::chrono::system_clock::time_point time_last;
  auto time_now = std::chrono::system_clock::now();
  auto diff = time_now - time_last;

  if( diff.count() > 0
      ||
      abs(exposure_target_ - exposure_setting_) > camera_config_ptr_->exposure_change_threshold
      ||
      abs(gain_target_ - gain_setting_) > camera_config_ptr_->gain_change_threshold
      &&
      (exposure_target_ != exposure_setting_ || gain_target_ != gain_setting_)
    ) {

        time_last = time_now;

#ifdef QC_SOC_TARGET_APQ8074
        char exposure_string[6];
        sprintf(exposure_string,"%d",exposure_target_);
        char gain_string[6];
        sprintf(gain_string,"%d",gain_target_);
        params_.set("qc-exposure-manual",exposure_string);
        params_.set("qc-gain-manual",gain_string);
#endif
#ifdef QC_SOC_TARGET_APQ8096
        params_.setManualExposure(exposure_target_);
        params_.setManualGain(gain_target_);
#endif
        params_.commit();

        // The exposure and gain take effect in the second frame
        // after the params are committed i.e. there is one frame in between
        // committing params and receiving a frame with the params applied
        static int exposure_temp = exposure_setting_;
        static int gain_temp = gain_setting_;
        exposure_setting_ = exposure_temp;
        gain_setting_ = gain_temp;
        exposure_temp = exposure_target_;
        gain_temp = gain_target_;
  }
}

void Snapdragon::CameraManager::onError()
{
  printf("ERROR: Camera error.\n");
  if( running_ ) {
    camera_ptr_->stopPreview();
  }
  running_ = false;
}

int32_t Snapdragon::CameraManager::FindFrameIndex( int64_t frame_id )
{
  // find the corresponding frame_id in the frame_queue
  for (unsigned int ii = 0; ii < camera_config_ptr_->num_image_buffers; ++ii) {
    if (frame_queue_[ii].frame_id_ == frame_id) {
      frame_q_read_index_ = ii;
      return 0;
    }
  }
  // the requested frame does not exist
  frame_q_read_index_ = -1;
  return -1;
}

bool Snapdragon::CameraManager::HasFrameArrived( int64_t frame_id )
{
  bool ready = false;
  if (next_frame_id_ >= frame_id) {
    ready = true;
  }
  return ready;
}

int32_t Snapdragon::CameraManager::GetNextImageData( int64_t frame_id,
  uint64_t* timestamp_ns, uint32_t size, uint32_t* used,
  uint8_t* image_data ) {

  int32_t result = GetImageData( frame_id, timestamp_ns, size, used,
      image_data, nullptr );

  return result;
}

int32_t Snapdragon::CameraManager::GetNextStereoImageData( int64_t frame_id,
  uint64_t* timestamp_ns, uint32_t size, uint32_t* used,
  uint8_t* image_data_left, uint8_t* image_data_right ) {

  int32_t result = GetImageData( frame_id, timestamp_ns, size, used,
      image_data_left, image_data_right );

  return result;
}

int32_t Snapdragon::CameraManager::GetImageData( int64_t frame_id,
  uint64_t* timestamp_ns, uint32_t size, uint32_t* used,
  uint8_t* image_data, uint8_t* image_data_right ) {

  std::unique_lock<std::mutex> lock( frame_mutex_ );

  if( !running_ ) {
    // the camera has stopped. so return an error code.
    return -1;
  }

  // wait for specified frame if in the future.
  frame_cv_.wait_for(lock, std::chrono::seconds(1),
    std::bind(&Snapdragon::CameraManager::HasFrameArrived, this, frame_id));

  // find corresponding frame in the queue.
  int32_t frame_id_return = FindFrameIndex(frame_id);
  if (frame_id_return < 0) {
    // the requested frame does not exist in the queue.
    printf( "ERROR: The requested frame_id(%lld) does not exist in the queue.\n", frame_id );
    return -2;
  }

  if( size < image_size_bytes_ ) {
    printf( "ERROR: Insuffient image buffer size: given: %d expected: %d\n",
      size, image_size_bytes_ );
    return -3;
  }

  // the read index now corresponds to the requested frame
  *timestamp_ns = frame_queue_[frame_q_read_index_].timestamp_;
  *used = image_size_bytes_;

  //copy stereo camera data
  if( camera_config_ptr_->cam_type == CameraType::STEREO && image_data_right != nullptr ) {
    for( int i = 0; i < camera_config_ptr_->pixel_height; ++i ) {
      uint8_t* src_left = reinterpret_cast<uint8_t*>(
          frame_queue_[frame_q_read_index_].frame_->data + i * camera_config_ptr_->pixel_width * 2 );
      uint8_t* src_right = reinterpret_cast<uint8_t*>(
          src_left + camera_config_ptr_->pixel_width );
      memcpy( image_data + i * camera_config_ptr_->pixel_width, src_left, camera_config_ptr_->pixel_width );
      memcpy( image_data_right + i * camera_config_ptr_->pixel_width, src_right, camera_config_ptr_->pixel_width);
    }
  }
#ifdef QC_SOC_TARGET_APQ8096
  else if( camera_config_ptr_->cam_type == CameraType::RIGHT_STEREO ) {
    for( int i = 0; i < camera_config_ptr_->pixel_height; ++i ) {
      uint8_t* src_left = reinterpret_cast<uint8_t*>(
          frame_queue_[frame_q_read_index_].frame_->data + i * camera_config_ptr_->pixel_width * 2 );
      uint8_t* src_right = reinterpret_cast<uint8_t*>(
          src_left + camera_config_ptr_->pixel_width );
      //memcpy( image_data + i * camera_config_ptr_->pixel_width, src_left, camera_config_ptr_->pixel_width );
      memcpy( image_data + i * camera_config_ptr_->pixel_width, src_right, camera_config_ptr_->pixel_width);
    }
  }
#endif // QC_SOC_TARGET_APQ8096
  //monocular image data
  else {
    if (camera_config_ptr_->is_cam_master) {
      memcpy( image_data, reinterpret_cast<uint8_t*>( frame_queue_[frame_q_read_index_].frame_->data ), image_size_bytes_ );
    }
    else {
      memcpy( image_data, reinterpret_cast<uint8_t*>( frame_queue_[frame_q_read_index_].data_ ), image_size_bytes_ );
    }
  }

  return 0;
}

int32_t Snapdragon::CameraManager::GetExposureCenterTimestamp(int64_t frame_id, uint64_t* timestamp_coe) {

  std::lock_guard<std::mutex> lock( frame_mutex_ );
  int32_t frame_id_return = FindFrameIndex(frame_id);
  if (frame_id_return < 0) {
    return -1;
  }
  *timestamp_coe = frame_queue_[frame_q_read_index_].timestamp_coe_;
  return 0;
}

int32_t Snapdragon::CameraManager::GetExposureTimeUs(int64_t frame_id, float* exposure_time_us) {

  std::lock_guard<std::mutex> lock( frame_mutex_ );
  int32_t frame_id_return = FindFrameIndex(frame_id);
  if (frame_id_return < 0) {
    return -1;
  }
  *exposure_time_us = static_cast<float>(frame_queue_[frame_q_read_index_].exposure_time_ns_)/1000.f;
  return 0;
}

int32_t Snapdragon::CameraManager::GetExposureTime(int64_t frame_id, uint64_t* exposure_time_ns) {
  std::lock_guard<std::mutex> lock( frame_mutex_ );
  int32_t frame_id_return = FindFrameIndex(frame_id);
  if (frame_id_return < 0) {
    return -1;
  }
  *exposure_time_ns = frame_queue_[frame_q_read_index_].exposure_time_ns_;
  return 0;
}

void Snapdragon::CameraManager::onPreviewFrame(camera::ICameraFrame* frame)
{
  static uint64_t cntr = 0;
  if (frame->timeStamp != timestamp_last_nsecs_)
  {
    //increment the frameid;
    int64_t old_frame_id = next_frame_id_;

    //set the image_size bytes information.
    image_size_bytes_ = frame->size;
    if( camera_config_ptr_->cam_type == CameraType::STEREO ) {
      image_size_bytes_ = image_size_bytes_ / 2;
    }
    next_frame_id_++;

    //now add the frame to the queue.
    {
      std::lock_guard<std::mutex> lock( frame_mutex_ );

      //check if the queue already has an valid frame.  If yes, release the frame to be used
      // by the camera pipeline.
      if( frame_queue_[ frame_q_write_index_ ].frame_id_ != -1 &&
        frame_queue_[ frame_q_write_index_ ].frame_ != nullptr ) {
        frame_queue_[ frame_q_write_index_ ].frame_->releaseRef();
      }

      // add the frame to the queue.
      frame->acquireRef();
      CameraFrameType new_frame;
      new_frame.frame_id_ = next_frame_id_;
      new_frame.timestamp_ = frame->timeStamp;
      new_frame.frame_ = frame;
      // exposure timestamps
#ifdef QC_SOC_TARGET_APQ8074
      new_frame.exposure_time_ns_ = static_cast<int64_t>(1e3 * exposure_setting_ * camera_config_ptr_->row_period_us);
      float correction = static_cast<float>(new_frame.exposure_time_ns_)/2.f;
      new_frame.timestamp_coe_ = new_frame.timestamp_ - static_cast<int64_t>(correction);
#endif
#ifdef QC_SOC_TARGET_APQ8096
      new_frame.exposure_time_ns_ = params_.getFrameExposureTime(frame);
      float correction = static_cast<float>(new_frame.exposure_time_ns_)/2.f;
      new_frame.timestamp_coe_ = new_frame.timestamp_ + static_cast<int64_t>(correction);
#endif
      frame_queue_[ frame_q_write_index_ ] = new_frame;

      //update the gain and exposure only one every 4 frames, as there a delay in taking the new
      // exposure parameters to take into effect.
#ifdef QC_SOC_TARGET_APQ8074
      if( next_frame_id_ > 0 && (next_frame_id_ % 4 ) == 0 ) {
        UpdateGainAndExposure();
      }
#endif
#ifdef QC_SOC_TARGET_APQ8096
      if( next_frame_id_ > 0 && (next_frame_id_ % 8 ) == 0 ) {
        UpdateGainAndExposure();
      }
#endif

      frame_q_write_index_++;
      frame_q_write_index_ = (frame_q_write_index_ >= camera_config_ptr_->num_image_buffers )?0:frame_q_write_index_;

      // notify the caller who is waiting on a frame.
      frame_cv_.notify_all();
    }

    // update the frame stats.
    uint64_t timestamp_nsecs = frame->timeStamp;
    uint32_t dt_nsecs = timestamp_nsecs - timestamp_last_nsecs_;
    timestamp_last_nsecs_ = timestamp_nsecs;
    fps_avg_ = ((fps_avg_ * old_frame_id) + (1e9 / dt_nsecs)) / (next_frame_id_);
  }
  else{
    printf( "WARN: Got duplicate image frame at timestamp: %lld frame_id: %lld\n", frame->timeStamp, next_frame_id_ );
  }
}

void Snapdragon::CameraManager::onVideoFrame(camera::ICameraFrame* frame)
{
  // Should only get here if video is requested, which is not supported
  printf("WARN: Got video frame!\n");
}

#ifdef QC_SOC_TARGET_APQ8096
void Snapdragon::CameraManager::onError(camera::SUBSCRIBER_STATUS status)
{
  printf("ERROR: Camera error.\n");
  if( running_ ) {
    camera_sub_ptr_->stopPreview();
  }
  running_ = false;
}

void Snapdragon::CameraManager::onPreviewFrame(camera::ICameraSubscriberFrame* frame)
{
  static uint64_t cntr = 0;
  if (frame->timeStamp != timestamp_last_nsecs_)
  {
    //increment the frameid;
    int64_t old_frame_id = next_frame_id_;

    //set the image_size bytes information.
    image_size_bytes_ = frame->size;
    if( camera_config_ptr_->cam_type == CameraType::STEREO ) {
      image_size_bytes_ = image_size_bytes_ / 2;
    }
    next_frame_id_++;

    //now add the frame to the queue.
    if (image_size_bytes_ <= IMAGE_DATA_BUFFER_SIZE)
    {
      std::lock_guard<std::mutex> lock( frame_mutex_ );

      // add the frame to the queue.
      CameraFrameType new_frame;
      new_frame.frame_id_ = next_frame_id_;
      new_frame.timestamp_ = frame->timeStamp;
      std::memcpy(new_frame.data_, frame->data, image_size_bytes_);

      camera::tFrameMetaData metaData;
      if(!frame->getMetaData(metaData))
      {
        printf( "ERROR: Metadata invalid\n");
      }

      new_frame.exposure_time_ns_ = metaData.actualFrameExposureTime;

      float correction = static_cast<float>(new_frame.exposure_time_ns_)/2.f;
      new_frame.timestamp_coe_ = new_frame.timestamp_ + static_cast<int64_t>(correction);
      frame_queue_[ frame_q_write_index_ ] = new_frame;


      frame_q_write_index_++;
      frame_q_write_index_ = (frame_q_write_index_ >= camera_config_ptr_->num_image_buffers )?0:frame_q_write_index_;

      // notify the caller who is waiting on a frame.
      frame_cv_.notify_all();
    }

    // update the frame stats.
    uint64_t timestamp_nsecs = frame->timeStamp;
    uint32_t dt_nsecs = timestamp_nsecs - timestamp_last_nsecs_;
    timestamp_last_nsecs_ = timestamp_nsecs;
    fps_avg_ = ((fps_avg_ * old_frame_id) + (1e9 / dt_nsecs)) / (next_frame_id_);
  }
  else{
    printf( "WARN: Got duplicate image frame at timestamp: %lld frame_id: %lld\n", frame->timeStamp, next_frame_id_ );
  }
}

void Snapdragon::CameraManager::onVideoFrame(camera::ICameraSubscriberFrame* frame)
{
  // Should only get here if video is requested, which is not supported
  printf("WARN: Got video frame from subscriber!\n");
}

#endif

