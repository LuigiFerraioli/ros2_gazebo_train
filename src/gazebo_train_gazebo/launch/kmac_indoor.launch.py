#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    world_file_name = 'ros2_worlds/' + 'kmac_indoor' + '.model'
    world = os.path.join(get_package_share_directory('gazebo_train_gazebo'),
                         'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('gazebo_train_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        Node(package='gazebo_train_driver',executable='steuerung',name='steuerung',output='screen'),
        #Node(package='rqt_image_view',executable='rqt_image_view',name='image_view',output='screen'),
	    Node(package='rviz2',executable='rviz2', name='rviz2', arguments=['-d' +os.path.join(get_package_share_directory('gazebo_train_gazebo'), 'rviz', 'gazebo_train_rviz.rviz')]),
        Node(package='gazebo_train_driver',executable='gazebo_train_converter',name='gazebo_train_converter',output='screen'),  
    ])
