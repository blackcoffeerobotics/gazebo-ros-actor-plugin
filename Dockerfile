# ROS 2 Humble
FROM ros:humble

# Prevent console from interacting with the user
ARG DEBIAN_FRONTEND=noninteractive

# This is required else apt-get update throws Hash mismatch error
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR for RViz2
RUN mkdir -p /tmp/runtime-root && chmod 0700 /tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# Install colcon for ROS 2 build, RViz2 and Gazebo
RUN apt-get install --no-install-recommends -yqqq \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-ros-gz

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

# Install ros_gz packages for Gazebo integration
RUN apt-get install --no-install-recommends -yqqq \
    ros-$ROS_DISTRO-ros-gz-sim \
    ros-$ROS_DISTRO-ros-gz-bridge

# Install teleop_twist_keyboard to send cmd_vel commands
RUN apt-get install --no-install-recommends -yqqq \
    ros-$ROS_DISTRO-teleop-twist-keyboard

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
