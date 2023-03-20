# ROS Noetic
FROM ros:noetic

# Prevent console from interacting with the user
ARG DEBIAN_FRONTEND=noninteractive

# This is required else apt-get update throws Hash mismatch error
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Ensure timeouts are set to a minimum to enable faster exit
RUN sed -i -e 's/_TIMEOUT_SIGINT  = 15.0/_TIMEOUT_SIGINT  = 1e-323/g' \
    -e 's/_TIMEOUT_SIGTERM = 2.0/_TIMEOUT_SIGTERM = 1e-323/g' \
    /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py

# Set folder for RUNTIME_DIR for RViz
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# Install catkin_tools for catkin build, RViz and Gazebo
RUN apt-get install --no-install-recommends -yqqq \
    python3-catkin-tools \
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-gazebo-ros

# Optional
#--------------
# Add additional dependencies here as a separate step
# Don't modify above steps, as it will trigger a rebuild

# Non Python/ROS Dependencies
RUN apt-get install --no-install-recommends -yqqq \
    vim

# Python Dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

# ROS Dependencies
# RUN apt-get install --no-install-recommends -yqqq \
#     ros-$ROS_DISTRO-xacro

# Install gazebo_ros_pkgs
RUN apt-get install --no-install-recommends -yqqq \
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

# Install teleop_twist_keyboard to send cmd_vel commands
RUN apt-get install --no-install-recommends -yqqq \
    ros-noetic-teleop-twist-keyboard

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
