#  YoloV5_D415_ROS
The code uses the official realsense documentation and yolov5_ros from mats-robotics (https://github.com/mats-robotics/yolov5_ros)

## Install
### 1.Install cuda && pytorch
> install CUDA URL：https://developer.nvidia.com/cuda-toolkit

> install pytorch URL：https://pytorch.org/  

### 2.git && install requirements

> mkdir -p ~/catkin_ws/src

> cd ~/catkin_ws/src/

#### realsense:
> git clone https://github.com/IntelRealSense/realsense-ros.git

> git clone https://github.com/pal-robotics/ddynamic_reconfigure.git

> cd realsense-ros/

> git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`

> cd ..

#### yolov5_ros:
> git clone https://github.com/mats-robotics/detection_msgs.git

> git clone https://github.com/Jumbo-zczlbj0/YoloV5_D415_ROS.git

> cd ./YoloV5_D415_ROS/src

> chmod +x detect.py

#### yolov5:
> git clone https://github.com/ultralytics/yolov5.git

> pip3 install requirements.txt

### 3.Install Intel RealSense SDK 2.0

> Choose the version that suits you based on your computer system and install SDK-2：https://www.intelrealsense.com/sdk-2/

> Install pyrealsense2：pip install pyrealsense2

### 4.ROS

> Ubuntu20.04 + ROS: https://wiki.ros.org/noetic/Installation/Ubuntu

### 5.Build the ROS package:

> cd ~/catkin_ws

> catkin_make

## Run

cd ~/catkin_ws

> source ~/catkin_ws/devel/setup.bash

> roslaunch realsense2_camera rs_camera.launch

Open another terminal
> roslaunch yolo_topic yolo_topic.launch
