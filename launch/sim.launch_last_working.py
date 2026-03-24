#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo_ros_actor_plugin = get_package_share_directory('gazebo_ros_actor_plugin')
    pkg_pablo_worlds = get_package_share_directory('pablo_worlds')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    #world_file = os.path.join(pkg_gazebo_ros_actor_plugin, 'config', 'worlds', 'move_actor.world')
    world_file = os.path.join(pkg_pablo_worlds, 'worlds', 'street.world')
    #world_file = os.path.join(pkg_pablo_worlds, 'worlds', 'city.world')
    model_path = os.path.join(pkg_gazebo_ros_actor_plugin, 'config', 'skins')

    # Declare launch arguments
    verbose_arg = DeclareLaunchArgument(
        'verbose', default_value='True', description='Enable verbose mode for Gazebo'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='False', description='Enable headless mode for Gazebo'
    )

    verbose = LaunchConfiguration('verbose')
    headless = LaunchConfiguration('headless')

    gz_resource_path = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=model_path)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': PythonExpression([
                f"'{world_file} -r'",
                " + (' -v' if '", verbose, "' == 'True' else '')",
                " + (' -s' if '", headless, "' == 'True' else '')"
            ])
        }.items()
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel1@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/cmd_path1@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',

            '/cmd_vel2@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/cmd_path2@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',

            '/cmd_vel3@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/cmd_path3@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',

           '/cmd_vel4@geometry_msgs/msg/Twist@gz.msgs.Twist',
           '/cmd_path4@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V'
            
        ],
        output='screen'
    )


    actor_file1 = os.path.join(pkg_gazebo_ros_actor_plugin,
                              'config', 'skins', 'DoctorFemaleWalk', 'model1.sdf')

    actor_file2 = os.path.join(pkg_gazebo_ros_actor_plugin,
                                  'config', 'skins', 'DoctorFemaleWalk', 'model2.sdf')

    actor_file3 = os.path.join(pkg_gazebo_ros_actor_plugin,
                                  'config', 'skins', 'DoctorFemaleWalk', 'model3.sdf')
    actor_file4 = os.path.join(pkg_gazebo_ros_actor_plugin,
                                  'config', 'skins', 'DoctorFemaleWalk', 'model4.sdf')
                                  
    spawn_actor1 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', actor_file1,
            '-name', 'actor1',
            '-x', '-3',
            '-y', '-3',
            
        ],
    )

    spawn_actor2 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', actor_file2,
            '-name', 'actor2',
            '-x', '3',
            '-y', '-3'
        ],
    )

    spawn_actor3 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', actor_file3,
                '-name', 'actor3',
                '-x', '-3',
                '-y', '3'
            ],
        )

    spawn_actor4 = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', actor_file4,
                '-name', 'actor4',
                '-x', '1.5',
                '-y', '1.5'
            ],
        )



    return LaunchDescription([
        verbose_arg,
        headless_arg,
        gz_resource_path,
        gz_sim,
        ros_gz_bridge,
        spawn_actor1,
        spawn_actor2,
        spawn_actor3,
        spawn_actor4
    ])
