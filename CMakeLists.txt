cmake_minimum_required(VERSION 3.0.2)
project(yolo_topic)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_message_files(
   FILES
   ObjectDepth.msg

 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  LIBRARIES yolo_topic
  CATKIN_DEPENDS rospy std_msgs sensor_msgs cv_bridge message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
   scripts/pyrealsense2_camera.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )


