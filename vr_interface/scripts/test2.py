import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    # Declare launch arguments
    robot_ip = LaunchConfiguration('robot_ip')
    arm_id = LaunchConfiguration('arm_id')
    use_rviz = LaunchConfiguration('use_rviz')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    load_gripper = LaunchConfiguration('load_gripper')

    # Load robot description
    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', 'fr3', 'fr3.urdf.xacro'
    )
    robot_description_config = Command(
        [
            FindExecutable(name='xacro'), ' ', franka_xacro_file, 
            ' hand:=', load_gripper, 
            ' robot_ip:=', robot_ip, 
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
            ' ros2_control:=true'
        ]
    )
    robot_description = {'robot_description': ParameterValue(robot_description_config, value_type=str)}

    # Robot state publisher with delay to ensure robot_description is published
    robot_state_publisher = TimerAction(
        period=3.0,  # Delay to ensure robot_description is published
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[robot_description],
            )
        ]
    )

    # Include Franka bringup launch
    franka_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('franka_bringup'),
                'launch', 'franka.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': robot_ip,
            'arm_id': arm_id,
            'load_gripper': load_gripper,
            'use_fake_hardware': use_fake_hardware,
            'fake_sensor_commands': fake_sensor_commands,
            'use_rviz': use_rviz,
        }.items(),
    )

    # ROS 2 control node
    ros2_controllers_path = os.path.join(
        get_package_share_directory('franka_fr3_moveit_config'),
        'config', 'fr3_ros_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        remappings=[('joint_states', 'franka/joint_states')],
        output={'stdout': 'screen', 'stderr': 'screen'},
        on_exit=Shutdown(),
    )

    # Load controllers with error handling and increased delay
    load_controllers = []
    for controller in [ 'fr3_arm_controller']:
        load_controllers.append(
            TimerAction(
                period=5.0,  # Increased delay for stability
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', controller],
                        output='screen',
                        on_exit=Shutdown(),  # Ensure failure shuts down correctly
                    )
                ]
            )
        )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[
            {'source_list': ['franka/joint_states', 'fr3_gripper/joint_states'], 'rate': 30}
        ],
    )

    # Move test node (actual VR interface controller)
    move_test_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='vr_interface',
                executable='move_test',
                name='move_test',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='172.16.0.2', description='IP address of the robot.'),
        DeclareLaunchArgument('arm_id', default_value='fr3', description='Type of arm used (fr3, fer, fp3).'),
        DeclareLaunchArgument('use_rviz', default_value='false', description='Enable Rviz visualization.'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false', description='Use fake hardware.'),
        DeclareLaunchArgument('fake_sensor_commands', default_value='false', description='Fake sensor commands (valid only with fake hardware).'),
        DeclareLaunchArgument('load_gripper', default_value='false', description='Use Franka Gripper as an end-effector.'),

        #robot_state_publisher,
        franka_bringup_launch,
        #ros2_control_node,
        #joint_state_publisher,
        *load_controllers,  # Ensure controllers are loaded sequentially
        move_test_node,
    ])
