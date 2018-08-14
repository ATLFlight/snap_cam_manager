cmake_minimum_required(VERSION 2.6.0)
project(camera_manager)

add_definitions(-std=c++11) 

# Use the APQ8096 flag for Snapdragon 8x96 processors
if ("${QC_SOC_TARGET}" STREQUAL "APQ8096")
  add_definitions( -DQC_SOC_TARGET_APQ8096 )
  message("Building for APQ8096")
  # Use the TOF flag for TOF Support on Snapdragon 8x96 processors
  if ("${QC_TOF_BUILD}" STREQUAL "TOF")
    add_definitions( -DQC_TOF_BUILD )
    message("Building for APQ8096 TOF")
  endif()

# Use the APQ8074 flag for Snapdragon 8x74 processors
# Like the Snapdragon Flight board
else()
  add_definitions( -DQC_SOC_TARGET_APQ8074 )
  message("Building for APQ8074")
endif()


include_directories(src/)
include_directories(open_camera/)
include_directories(image_streamer/)

# create an executable, and make it public
add_executable(snap-open-camera open_camera/main.cpp
                            src/SnapdragonCameraManager.cpp
                            src/SnapdragonCameraUtil.cpp
              )

target_link_libraries(snap-open-camera camera
                                       pthread)

# create an executable, and make it public
add_executable(snap-image-streamer image_streamer/main.cpp
                            image_streamer/ImageStreamer.cpp
                            image_streamer/TcpUtils.cpp
                            src/SnapdragonCameraManager.cpp
                            src/SnapdragonCameraUtil.cpp
              )

target_link_libraries(snap-image-streamer camera
                                          pthread)

