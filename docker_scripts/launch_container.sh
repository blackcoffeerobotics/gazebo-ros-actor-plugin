#!/bin/bash
xhost +local:root

docker run -it -d --privileged --net=host \
--name ros1_actor_plugin \
-v $PWD/..:/root/ros1_ws/src/gazebo_ros_actor_plugin/ \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
 bcr_ros-noetic_gz-11:latest

xhost -local:root
