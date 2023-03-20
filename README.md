# gazebo_ros_actor_plugin

## About

    A Gazebo plugin and ROS package to control actors dynamically using position or velocity commands.

## System Requirements

Before using this package, please make sure that the following requirements are met:

- ROS 1 Noetic 
- Gazebo Classic (Version 11)
- gazebo_ros_pkgs 

## Installation

To use this package with a Docker container, follow these steps:

1. Build the Docker image using `docker build -t bcr_ros-noetic_gz-11:latest .`
2. Launch the container by running `cd docker_scripts` and `./launch_container.sh`. This will mount the package to `/root/ros2_ws/src/`.
3. To enter a bash session, run `./bashing_container.sh`.
4. To stop the container, run `./stop_container.sh`.

## Usage

To use the gazebo_ros_actor_plugin, follow these steps:

1. Build the package using `catkin_make`.
2. Source `setup.bash` for the workspace containing this package.
3. Edit the parameters in the `velocity_follow.world` file and choose the method of subscription using the "follow" tag corresponding to the plugin of the actor "actor1". It could be either subscribing to path or velocity commands. 
4. Launch the `sim.launch` file by running:

        ```
        roslaunch gazebo_ros_actor_plugin sim.launch
        ```
5. Start the desired publisher. This package has two example publishers set up to try the plugin:

    a. Velocity publisher: To run this publisher, enter the following command. Then give keyboard inputs to command linear and angular velocities. This publisher is based on the `ros-noetic-teleop-twist-keyboard` package.

        ```
        rosrun teleop_twist_keyboard teleop_twist_keyboard.py
        ```

    b. Path publisher: To run this publisher, enter the following command. Note that the workspace needs to be sourced before this command is run. This file can be found in the `/scripts` directory in the package.

        ```
        rosrun gazebo_ros_actor_plugin path_publisher.py
        ```

## ROS API

The gazebo_ros_actor_plugin subscribes to information from the following inbound topics:

- `/cmd_vel`: to receive linear and angular velocity commands
- `/cmd_path`: to receive path commands

Note that the names of the topics can be overridden using the `.world` file present in this package.
