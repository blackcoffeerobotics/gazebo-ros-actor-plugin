#!/usr/bin/env python3
"""
Launch file for Gazebo Harmonic with ROS2 actor plugin
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_gazebo_ros_actor_plugin = get_package_share_directory('gazebo_ros_actor_plugin')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    world_file = os.path.join(pkg_gazebo_ros_actor_plugin, 'config', 'worlds', 'move_actor.world')
    model_path = os.path.join(pkg_gazebo_ros_actor_plugin, 'config', 'skins')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode (no GUI)')
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Run Gazebo with verbose output')
    
    bridge_arg = DeclareLaunchArgument(
        'enable_bridge',
        default_value='true',
        description='Enable ROS-Gazebo topic bridge')
    
    # Set Gazebo resource paths
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=model_path)
    
    # Gazebo arguments
    gz_args = ['-r', world_file]
    
    # Add verbose if requested
    verbose = LaunchConfiguration('verbose')
    gz_args_with_verbose = ['-v', '4'] + gz_args
    
    # Add headless if requested  
    headless = LaunchConfiguration('headless')
    gz_args_with_headless = ['-s'] + gz_args
    
    # Launch Gazebo Harmonic
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': ' '.join(gz_args_with_verbose),
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # ROS-Gazebo Bridge for /cmd_vel and /cmd_path
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/cmd_path@nav_msgs/msg/Path]gz.msgs.Path',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        headless_arg,
        verbose_arg,
        bridge_arg,
        gz_resource_path,
        gz_sim,
        ros_gz_bridge,
    ])
