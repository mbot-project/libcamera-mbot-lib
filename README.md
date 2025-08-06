# libcamera-mbot-lib
For ROS2 Jazzy MBot Classic Camera - Ubuntu 24.04

## Installing Camera Node
```
# create workspace
mkdir -p ~/camera_ws/src
cd ~/camera_ws/src

# check out libcamera
sudo apt -y install python3-colcon-meson

git clone git@github.com:mbot-project/libcamera-mbot-lib.git

cd ~/camera_ws/
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera
colcon build --event-handlers=console_direct+

```

## Running Camera Node

```
ros2 run camera_ros camera_node
```

If you get this error:
```
sacchin@sacchin:~/camera_ws$ ros2 run camera_ros camera_node 
[0:17:33.793904189] [5149]  INFO Camera camera_manager.cpp:330 libcamera v0.5.1+94-b65df7e7
terminate called after throwing an instance of 'std::runtime_error'
  what():  no cameras available
[ros2run]: Aborted
```

Try doing this and running the camera node:
```
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
```

Install for camera calib:
```
sudo apt install ros-jazzy-image-pipeline
```

April Tag detection
```
sudo apt install ros-jazzy-vision-opencv ros-jazzy-apriltag-msgs \
                 libopencv-dev libopencv-contrib-dev
```