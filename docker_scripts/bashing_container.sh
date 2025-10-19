#!/bin/bash
xhost +local:root

docker exec -it \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
ros2_actor_plugin bash

xhost -local:root

