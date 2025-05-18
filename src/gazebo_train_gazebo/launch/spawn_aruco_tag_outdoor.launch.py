#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    model_name = 'aruco_visual_marker_1'
    model_sdf_file = 'model.sdf'

    sdf_path = os.path.join(get_package_share_directory(
        'gazebo_train_gazebo'), 'models', model_name, model_sdf_file)

    xml = open(sdf_path, 'r').read()
    xml = xml.replace('"', '\\"')

    # Add pose information to the XML string
    pose = '<pose>-3.5 0 1.1 0 1.570796  0</pose>'
    xml_with_pose = xml.replace('</model>', pose + '</model>')

    spawn_args = '{name: \"' + model_name + \
        '\", xml: \"' + xml_with_pose + '\" }'

    return LaunchDescription([
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.18',
            description='Size of the marker'
        ),

        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity',
                 'gazebo_msgs/SpawnEntity', spawn_args],
            output='screen'),

        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='marker_publisher',
            output='screen',
            remappings=[
                ('image', '/zed2i_depth/image_raw'),
                ('camera_info', '/zed2i_depth/depth/camera_info')
            ],
            parameters=[{'marker_size': LaunchConfiguration('marker_size')}]
        ),

        Node(
            package='train_control',
            executable='aruco_original_control',
            name='aruco_original_control',
            output='screen',
        ),
    ])
