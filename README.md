#  YoloV5_D415_ROS
The code from YoloV5_D415 (https://github.com/Jumbo-zczlbj0/YoloV5_D415) has been modified. Speech recognition and multithreading features were removed, and an output for object category and depth in the ROS topic was added

## Install
### 1.Install cuda && pytorch
> install CUDA URL：https://developer.nvidia.com/cuda-toolkit

> install pytorch URL：https://pytorch.org/  

### 2.git && install requirements

> mkdir -p ~/catkin_ws/src

> cd ~/catkin_ws/src/

> git clone [https://github.com/ultralytics/yolov5.git](https://github.com/Jumbo-zczlbj0/YoloV5_D415_Ros_Topic.git)

> cd ./YoloV5_D415_Ros_Topic/scripts

> chmod +x pyrealsense2_camera.py

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

> roslaunch yolo_topic yolo_topic.launch

