#!/bin/bash
xhost +local:root

docker run -it -d --privileged --net=host \
--name ros2_actor_plugin \
-v $PWD/..:/root/ros2_ws/src/gazebo_ros_actor_plugin/ \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
 bcr_ros2-humble_gz-fortress:latest

xhost -local:root
