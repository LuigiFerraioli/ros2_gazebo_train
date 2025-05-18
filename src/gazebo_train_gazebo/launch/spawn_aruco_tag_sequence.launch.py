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
    marker_size = LaunchConfiguration('marker_size', default='0.18')

    model_name_prefix = 'aruco_visual_marker_'
    model_sdf_file = 'model.sdf'
    model_path = get_package_share_directory('gazebo_train_gazebo')
    positions = [10.0, 0.0, -10.0]  # x positions for the three markers

    spawn_commands = []

    for i, x_position in enumerate(positions):
        model_name = model_name_prefix + str(i + 1)
        sdf_path = os.path.join(model_path, 'models',
                                model_name, model_sdf_file)
        xml = open(sdf_path, 'r').read()
        xml = xml.replace('"', '\\"')

        # Add pose information to the XML string
        pose = f'<pose>{x_position} 0 1.1 0 1.57 0</pose>'
        xml_with_pose = xml.replace('</model>', pose + '</model>')

        spawn_args = '{name: \"' + model_name + \
            '\", xml: \"' + xml_with_pose + '\" }'

        spawn_commands.append(
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/spawn_entity',
                     'gazebo_msgs/SpawnEntity', spawn_args],
                output='screen'
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'marker_size',
            default_value='0.18',
            description='Size of the marker'
        ),
    ] + spawn_commands + [

        Node(
            package='aruco_ros',
            executable='marker_publisher',
            name='marker_publisher',
            output='screen',
            remappings=[
                ('image', '/zed2i_depth/image_raw'),
                ('camera_info', '/zed2i_depth/depth/camera_info')
            ],
            parameters=[{'marker_size': marker_size}]
        ),

        Node(
            package='train_control',
            executable='aruco_control',
            name='aruco_control',
            output='screen',
        ),
    ])
