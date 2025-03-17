import os
from launch import LaunchDescription
from launch.actions import LogInfo, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch the ZED camera from the 'zed_wrapper' package using ExecuteProcess
        ExecuteProcess(
            cmd=['ros2', 'launch', 'zed_wrapper', 'zed_camera.launch.py', 
                 'camera_model:=zedm', 'pub_resolution:=HD1080', 
                 'pub_frame_rate:=15', 'grab_resolution:=HD1080', 
                 'grab_frame_rate:=30'],
            name='zed_camera',
            output='screen',
            shell=True
        ),

        # Add a delay of 2 seconds
        TimerAction(
            period=2.0,
            actions=[
                # Launch the zedimage_bridge from 'middle_nodes' package (simple_ws)
                ExecuteProcess(
                    cmd=['ros2', 'run', 'middle_nodes', 'zedimage_bridge'],
                    name='zedimage_bridge',
                    output='screen'
                ),

                # Launch the default_server_endpoint from ros_tcp_endpoint package (simple_ws)
                ExecuteProcess(
                    cmd=['ros2', 'run', 'ros_tcp_endpoint', 'default_server_endpoint', '--ros-args', 
                         '-p', 'ROS_IP:=10.4.0.11', '-p', 'ROS_TCP_PORT:=10000'],
                    name='ros_tcp_endpoint',
                    output='screen'
                ),
            ]
        ),

        # Log message indicating the completion
        LogInfo(msg="Launched all nodes in the correct order!")
    ])
