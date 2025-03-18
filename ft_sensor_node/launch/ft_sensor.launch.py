from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Define the ft_sensor node
    ft_sensor_node = Node(
        package='ft_sensor_node',
        executable='ft_sensor_node',
        name='ft_sensor_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',  # Adjust as needed
            'baudrate': 460800,       # Adjust as needed
            'frame_id': 'ft_sensor_link'
        }]
    )

    # Define the ft_filter node
    ft_filter_node = Node(
        package='ft_sensor_node',
        executable='ft_filter_node',
        name='ft_filter_node',
        output='screen',
        parameters=[{
            'max_range': 1.0,         # Adjust as needed
            'calibration_time': 5.0    # Adjust as needed
        }]
    )

    # Delay the start of ft_filter_node by 2 seconds after ft_sensor_node starts
    delayed_ft_filter_node = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ft_sensor_node,
            on_start=[
                TimerAction(
                    period=2.0,  # Delay of 2 seconds
                    actions=[ft_filter_node]
                )
            ]
        )
    )

    # Create the launch description
    return LaunchDescription([
        ft_sensor_node,
        delayed_ft_filter_node
    ])