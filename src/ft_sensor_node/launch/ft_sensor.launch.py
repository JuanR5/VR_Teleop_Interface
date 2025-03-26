"""
@file ft_sensor_launch.py
@brief Launch file for starting the FT sensor node system with warm-up handling and delayed startup.

This launch file performs the following sequence:
1. Starts a warm-up process (`serial_driver`)
2. Kills it after 5 seconds
3. Starts `ft_sensor_node` after 5 seconds
4. Starts `ft_filter_node` 2 seconds after `ft_sensor_node` starts
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    """
    @brief Generate the launch description with warm-up logic and delayed startup for sensor and filter nodes.
    @return LaunchDescription object to be run by ROS 2 launch system.
    """

    # === Step 0: Locate the path to the installed binary `serial_driver` ===
    ft_sensor_prefix = get_package_prefix('ft_sensor_node')
    serial_driver_path = os.path.join(ft_sensor_prefix, 'lib', 'ft_sensor_node', 'serial_driver')

    # === Step 1: Launch warm-up process ===
    warmup_process = ExecuteProcess(
        cmd=[serial_driver_path],
        output='screen'
    )

    # === Step 2: Kill warm-up process after 5 seconds ===
    kill_warmup = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'serial_driver'],
                output='screen'
            )
        ]
    )

    # === Step 3: Delay ft_sensor_node startup until after warm-up ===
    ft_sensor_node = Node(
        package='ft_sensor_node',
        executable='ft_sensor_node',
        name='ft_sensor_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',     # Change if using different port
            'baudrate': 460800,         # Match device setting
            'frame_id': 'ft_sensor_link'
        }]
    )

    ft_sensor_node_delayed = TimerAction(
        period=5.0,
        actions=[ft_sensor_node]
    )

    # === Step 4: Launch ft_filter_node 2 seconds after ft_sensor_node starts ===
    delayed_ft_filter_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ft_sensor_node,
            on_start=[
                TimerAction(
                    period=2.0,  # Optional additional delay
                    actions=[
                        Node(
                            package='ft_sensor_node',
                            executable='ft_filter_node',
                            name='ft_filter_node',
                            output='screen',
                            parameters=[{
                                'max_range': 1.0,           # Tune based on expected input
                                'calibration_time': 5.0     # Seconds to gather baseline
                            }]
                        )
                    ]
                )
            ]
        )
    )

    # === Final launch sequence ===
    return LaunchDescription([
        warmup_process,
        kill_warmup,
        ft_sensor_node_delayed,
        delayed_ft_filter_node
    ])
