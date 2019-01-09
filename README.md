## Overview
This library provides an additional level of abstraction for Snapdragon Flight camera applications by wrapping the libcamera API. See the sample applications provided for examples on how to build and use this library. The API functions are documented in the main header file, `src/SnapdragonCameraManager.hpp`.

## Building
The sample applications are meant to be built on target. To build the sample applications on the Snapdragon Flight board:
```bash
mkdir build && cd build
cmake ..
make
```

If you are using a QFlight Pro board (8096), you will build the code inside your development docker.  You will also have to provide the following cmake variable:

``` cmake -DQC_SOC_TARGET=APQ8096```


### Requirements
Ensure that the Snapdragon Flight libcamera headers are present on target before attempting to build, which should be found at `/usr/include/camera.h` and `/usr/include/camera_parameters.h`.

## Sample Applications
The library is distributed with two sample applications.

### snap-open-camera
This is a minimal program that opens a camera with specified parameters. It is useful for debugging camera functionality and as a representative application for basic camera use. To see command line options, run:
```bash
./snap-open-camera -h
```

Typical usage and output looks something like this:
```bash
./snap-open-camera -c 1 -e 0.5 -g 0.6

DFT camera selected.
Camera of type 1 has ID = 2
[KPI Perf  711396879] openCamera: E PROFILE_OPEN_CAMERA camera id 2
Opened camera 2 Type: 1
Preview FPS range 0: [ 15, 15 ]
Preview FPS range 1: [ 24, 24 ]
Preview FPS range 2: [ 30, 30 ]
Preview FPS range 3: [ 60, 60 ]
Preview FPS range 4: [ 90, 90 ]
Preview FPS range 5: [ 120, 120 ]
Preview FPS range 6: [ 7, 200 ]
Preview size 0: [ 640 x 480 ]
Preview size 1: [ 640 x 240 ]
Preview size 2: [ 576 x 432 ]
Preview size 3: [ 480 x 320 ]
Preview size 4: [ 384 x 288 ]
Preview size 5: [ 352 x 288 ]
Preview size 6: [ 320 x 240 ]
Preview size 7: [ 240 x 160 ]
Preview size 8: [ 176 x 144 ]
Preview format 0: yuv420sp
Preview format 1: yuv420p
Preview format 2: nv12-venus
Preview format 3: bayer-rggb
Setting FPS to 30
Setting preview size to 640x480
Setting preview format to RAW_FORMAT
Opening camera and starting preview frame...
[0] Camera is running...
[1] Camera is running...
[2] Camera is running...
[3] Camera is running...
[4] Camera is running...
[5] Camera is running...
[6] Camera is running...
[7] Camera is running...
[8] Camera is running...
[9] Camera is running...
[10] Camera is running...
[11] Camera is running...
[12] Camera is running...
```

### snap-image-streamer
This program opens a camera and streams images to a connected computer. It is very useful for verifying that the cameras are working properly and the images are as expected. In addition, the viewer on the connected computer can save snapshots of the images. To see command line options, run:
```bash
./snap-image-streamer -h
```

For example, to stream every 4th image from the stereo cameras with a target IP address of 192.168.1.1, run:
```bash
./snap-image-streamer -c 3 -n 4 -i 192.168.1.1
```

#### Viewing the Images
There is a python program called **image_viewer** that runs on the connected computer for viewing the streamed images. It is a GUI that includes a window for live-viewing the images, and buttons for changing gain and exposure and saving an image. **image_viewer** is located in the repository at `image_streamer/image_viewer.py`. To run on the connected computer:
```bash
python image_viewer.py -i 192.168.1.1
```

##### Requirements
**image_viewer** requires Python and PyQt4. Install on MacOSX using Homebrew:
```bash
brew install python
brew link --overwrite python
brew tap homebrew/boneyard
brew install pyqt
```

Or, install on Linux:
```bash
sudo apt-get install python-dev
sudo apt-get install python-qt4
```
