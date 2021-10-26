# `kinect_ros2`

## Interface

### Overview
Basic Kinect-v1 (for the Xbox 360) node, with IPC support, based on [libfreenect](https://github.com/OpenKinect/libfreenect).
For now, it only supports a single Kinect device. (If multiple devices present, the first one listed by the `freenect_num_devices` will be selected).

### Published topics
* `~image_raw` - RGB image(rgb8) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~camera_info` - RGB camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))
* `~depth/image_raw` - Depth camera image(mono16) ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
* `~depth/camera_info` - Depth camera_info ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))

## Instalation
### 1. Install libfreenect
The package was tested using a manual build from the [libfreenect](https://github.com/OpenKinect/libfreenect) github because the Kinect used, had a firmware version that requires specific build flags.

### 2. Copy the repo
Copy the repo to your workspace source folder.
~~~
cd ~/ws/src
git clone https://github.com/fadlio/kinect_ros2
~~~

### 3. Install any missing ROS packages
Use `rosdep` from the top directory of your workspace to install any missing ROS related dependency.
~~~
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y
~~~

### 4. Build your workspace
From the top directory of your workspace, use `colcon` to build your packages.
~~~
cd ~/ws
colcon build
~~~

## Using this package

## Devices tested
* Kinect Model 1473