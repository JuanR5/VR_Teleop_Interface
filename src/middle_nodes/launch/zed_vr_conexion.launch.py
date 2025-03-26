#!/usr/bin/env python3

"""
@file zed_system_launch.py
@brief Launches the ZED camera driver, a custom image bridge node, and the Unity ROS TCP server.
"""

import os
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    """
    @brief Creates the launch description to bring up ZED camera, image bridge, and Unity TCP endpoint in order.
    
    @return LaunchDescription object to be run by the ROS 2 launch system.
    """

    return LaunchDescription([
        # === Launch ZED Camera using zed_wrapper ===
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py',
                'camera_model:=zedm',
                'pub_resolution:=HD1080',
                'pub_frame_rate:=15',
                'grab_resolution:=HD1080',
                'grab_frame_rate:=30'
            ],
            name='zed_camera',
            output='screen',
            shell=True  # Needed to pass ROS args properly
        ),

        # === Delay to allow ZED node startup before bridging ===
        TimerAction(
            period=2.0,
            actions=[

                # Launch custom image bridge node from 'middle_nodes'
                ExecuteProcess(
                    cmd=['ros2', 'run', 'middle_nodes', 'zedimage_bridge'],
                    name='zedimage_bridge',
                    output='screen'
                ),

                # Launch Unity ROS-TCP endpoint server
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'ros_tcp_endpoint', 'default_server_endpoint',
                        '--ros-args',
                        '-p', 'ROS_IP:=10.4.0.11',
                        '-p', 'ROS_TCP_PORT:=10000'
                    ],
                    name='ros_tcp_endpoint',
                    output='screen'
                )
            ]
        ),

        # === Final Launch Message ===
        LogInfo(msg="✅ Launched ZED camera, image bridge, and Unity TCP endpoint.")
    ])
