from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, TimerAction
from launch.event_handlers import OnProcessStart

from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    
    # Dynamically locate the path to the installed serial_driver
    ft_sensor_prefix = get_package_prefix('ft_sensor_node')
    serial_driver_path = os.path.join(ft_sensor_prefix, 'lib', 'ft_sensor_node', 'serial_driver')

    # Step 1: Launch the warm-up process
    warmup_process = ExecuteProcess(
        cmd=[serial_driver_path],
        output='screen'
    )

    # Step 2: After 5 seconds, kill the warm-up process
    kill_warmup = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['pkill', '-f', 'serial_driver'],
                output='screen'
            )
        ]
    )

    # Step 3: Delay the start of ft_sensor_node until after 5 seconds (after warm-up)
    ft_sensor_node = Node(
        package='ft_sensor_node',
        executable='ft_sensor_node',
        name='ft_sensor_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',   # Adjust as needed
            'baudrate': 460800,       # Adjust as needed
            'frame_id': 'ft_sensor_link'
        }]
    )

    ft_sensor_node_delayed = TimerAction(
        period=5.0,
        actions=[ft_sensor_node]
    )

    # Step 4: Once ft_sensor_node starts, wait 2 seconds and then launch ft_filter_node
    delayed_ft_filter_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ft_sensor_node,
            on_start=[
                TimerAction(
                    period=2.0,  # Additional delay after sensor node starts
                    actions=[
                        Node(
                            package='ft_sensor_node',
                            executable='ft_filter_node',
                            name='ft_filter_node',
                            output='screen',
                            parameters=[{
                                'max_range': 1.0,         # Adjust as needed
                                'calibration_time': 5.0    # Adjust as needed
                            }]
                        )
                    ]
                )
            ]
        )
    )

    return LaunchDescription([
        warmup_process,
        kill_warmup,
        ft_sensor_node_delayed,
        delayed_ft_filter_node
    ])
