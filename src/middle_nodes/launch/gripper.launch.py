#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for robot_ip with a default value.
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='172.16.0.2',
        description='IP address of the robot'
    )
    
    # Get the path to the franka_gripper package launch directory.
    franka_gripper_launch_dir = os.path.join(
        get_package_share_directory('franka_gripper'),
        'launch'
    )

    # Include the franka_gripper launch file with the robot_ip parameter.
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(franka_gripper_launch_dir, 'gripper.launch.py')
        ),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items()
    )

    # Launch your custom gripper command node.
    gripper_command_node = Node(
        package='middle_nodes',         
        executable='gripper_command.py',    
        name='gripper_command_node',
        output='screen'
    )

    return LaunchDescription([
        robot_ip_arg,
        gripper_launch,
        gripper_command_node
    ])

