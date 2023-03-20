#!/bin/bash
xhost +local:root

docker exec -it \
--env="DISPLAY"  \
--env="QT_X11_NO_MITSHM=1"  \
ros1_actor_plugin bash

xhost -local:root

