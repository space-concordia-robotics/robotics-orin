# ros2_aruco

ROS2 Wrapper for OpenCV Aruco Marker Tracking

This package depends on a recent version of OpenCV python bindings and transforms3d library:

```
pip3 install opencv-contrib-python transforms3d
```

## ROS2 API for the ros2_aruco Node

This node locates Aruco AR markers in images and publishes their ids and poses.

Published Topics:
* `/aruco_poses` (`geometry_msgs.msg.PoseArray`) - Poses of all detected markers (suitable for rviz visualization)
* `/aruco_markers` (`ros2_aruco_interfaces.msg.ArucoMarkers`) - Provides an array of all poses along with the corresponding marker ids

Parameters:
* `marker_size` - size of the markers in meters (default .0625)
* `aruco_dictionary_id` - dictionary that was used to generate markers (default `DICT_5X5_250`)
* `poll_delay_seconds` - how many seconds to wait between captures
* `camera_index` - which camera index to open (0 corresponds to /dev/video0)
* `camera_destination_index` - If present, will attempt to use v4l2loopback 
                                to allow another process to access the camera.
                                Needs ffmpeg and v4l2loopback-dev installed.

## Running Marker Detection

1. Using the launch file - parameters will be loaded from _aruco\_parameters.yaml_.
```
ros2 launch ros2_aruco aruco_recognition.launch.py
```
2. As a single ROS 2 node - you can specify parameter values at startup by adding `--ros-args -p marker_size:=.05`, for example.
```
ros2 run ros2_aruco aruco_node
```

### Getting Marker Poses
To know where the actual markers are in 3D space relative to the camera, the camera
must be calibrated, and the size of the markers must be known. The following steps
explain how to set it up:

1. Get some way to publish a camera to a ROS topic.
One way is installing v4l2_camera with sudo apt-get install ros-${ROS_DISTRO}-v4l2-camera, then 
run with a command such as `ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:="/dev/video2" -p image_size:=[1920,1080]`.
Check the **video_device** argument, to ensure the correct webcam is being accessed. Rviz2 can visualize them with an Image view.
2. Calibrate the camera. Install the `camera_calibration` ros package, then run it. More info is [here](https://docs.ros.org/en/rolling/p/camera_calibration/).
A command such as `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.020 image:=/image_raw` will work, ensure
the checkerboard used (eg [here](https://markhedleyjones.com/projects/calibration-checkerboard-collection)) is the right size and 
**square size**. The **size** is the number of corners in the checkerboard, the **square size** is the width of each square (in m). 
3. Move the camera around the checkerboard to calibrate (it should show dots and lines overlaid on the image)
4. Click "calibrate"
5. Click "save" and open the calibration at /tmp/calibrationdata.tar.gz
6. Paste the "camera_matrix" and "distortion_coefficients" into aruco_parameters.yaml
7. Set the "marker_size" to the marker width (in m)

## Generating Marker Images

```
ros2 run ros2_aruco aruco_generate_marker
```

Pass the `-h` flag for usage information: 

```
usage: aruco_generate_marker [-h] [--id ID] [--size SIZE] [--dictionary]

Generate a .png image of a specified maker.

optional arguments:
  -h, --help     show this help message and exit
  --id ID        Marker id to generate (default: 1)
  --size SIZE    Side length in pixels (default: 200)
  --dictionary   Dictionary to use. Valid options include: DICT_4X4_100,
                 DICT_4X4_1000, DICT_4X4_250, DICT_4X4_50, DICT_5X5_100,
                 DICT_5X5_1000, DICT_5X5_250, DICT_5X5_50, DICT_6X6_100,
                 DICT_6X6_1000, DICT_6X6_250, DICT_6X6_50, DICT_7X7_100,
                 DICT_7X7_1000, DICT_7X7_250, DICT_7X7_50, DICT_ARUCO_ORIGINAL
                 (default: DICT_5X5_250)
```
