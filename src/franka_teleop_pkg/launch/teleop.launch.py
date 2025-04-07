#!/usr/bin/env python3
"""
@file combined_launch.py
@brief Unified launch file for the Franka robot bringup and teleoperation.
 
This launch file brings up the Franka robot by:
  - Processing the robot xacro file to generate the URDF.
  - Launching the robot_state_publisher and ros2_control_node.
  - Spawning necessary controllers and joint state publishers.
  - Optionally launching the Franka gripper.
  - Launching RViz for visualization.
  - Launching the cartesian impedance controller for hand-eye calibration.
  
All launch arguments are declared at the top and used to configure the nodes.
"""

import os
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def robot_description_dependent_nodes_spawner(context: LaunchContext,
                                              robot_ip,
                                              arm_id,
                                              use_fake_hardware,
                                              fake_sensor_commands,
                                              load_gripper):
    """
    @brief Spawn nodes that depend on the robot description.
    
    Processes the xacro file to generate the URDF and launches:
      - robot_state_publisher
      - ros2_control_node (controller manager)
    
    @param context Launch context for substitution evaluation.
    @param robot_ip LaunchConfiguration for the robot's IP address.
    @param arm_id LaunchConfiguration for the robot arm identifier.
    @param use_fake_hardware LaunchConfiguration indicating whether to use fake hardware.
    @param fake_sensor_commands LaunchConfiguration indicating whether to fake sensor commands.
    @param load_gripper LaunchConfiguration indicating whether to load the gripper.
    @return List of Node actions.
    """
    # Perform substitutions on the launch arguments
    robot_ip_str = context.perform_substitution(robot_ip)
    arm_id_str = context.perform_substitution(arm_id)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    fake_sensor_commands_str = context.perform_substitution(fake_sensor_commands)
    load_gripper_str = context.perform_substitution(load_gripper)

    # Construct path to the robot xacro file
    franka_xacro_filepath = os.path.join(
        get_package_share_directory('franka_description'),
        'robots', arm_id_str, f'{arm_id_str}.urdf.xacro'
    )
    # Process the xacro file with provided mappings to generate the URDF
    robot_description = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            'ros2_control': 'true',
            'arm_id': arm_id_str,
            'robot_ip': robot_ip_str,
            'hand': load_gripper_str,
            'use_fake_hardware': use_fake_hardware_str,
            'fake_sensor_commands': fake_sensor_commands_str,
        }
    ).toprettyxml(indent='  ')

    # Define the path to the controllers configuration file
    franka_controllers = PathJoinSubstitution([
        FindPackageShare('franka_bringup'),
        'config', 'controllers.yaml'
    ])

    # Return nodes that use the processed robot description
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                franka_controllers,
                {'robot_description': robot_description},
                {'arm_id': arm_id}
            ],
            remappings=[('joint_states', 'franka/joint_states')],
            output={'stdout': 'screen', 'stderr': 'screen'},
            on_exit=Shutdown(),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """
    @brief Generate the unified launch description.
    
    Declares all required launch arguments and creates nodes for:
      - Robot state publishing and control.
      - Joint state publishing.
      - Controller spawners.
      - Optional gripper and visualization (RViz).
      - Cartesian impedance controller for hand-eye calibration.
    
    @return LaunchDescription object.
    """
    # -----------------------------
    # Declare Launch Argument Names
    # -----------------------------
    robot_ip_parameter = 'robot_ip'
    arm_id_parameter = 'arm_id'
    load_gripper_parameter = 'load_gripper'
    use_fake_hardware_parameter = 'use_fake_hardware'
    fake_sensor_commands_parameter = 'fake_sensor_commands'
    use_rviz_parameter = 'use_rviz'
    cartesian_impedance_controller_config_path_parameter = 'cartesian_impedance_controller_config_path'

    # -----------------------------
    # Create LaunchConfiguration Variables
    # -----------------------------
    robot_ip = LaunchConfiguration(robot_ip_parameter)
    arm_id = LaunchConfiguration(arm_id_parameter)
    load_gripper = LaunchConfiguration(load_gripper_parameter)
    use_fake_hardware = LaunchConfiguration(use_fake_hardware_parameter)
    fake_sensor_commands = LaunchConfiguration(fake_sensor_commands_parameter)
    use_rviz = LaunchConfiguration(use_rviz_parameter)
    cartesian_impedance_controller_config_path = LaunchConfiguration(cartesian_impedance_controller_config_path_parameter)

    # -----------------------------
    # Define Default File Paths
    # -----------------------------
    rviz_file = os.path.join(
        get_package_share_directory('franka_description'),
        'rviz', 'visualize_franka.rviz'
    )
    default_cartesian_impedance_controller_config_path = str(
        Path(get_package_share_directory("franka_teleop_pkg")) / "config" / "cartesian_impedance_controller.yaml"
    )

    # -----------------------------
    # Declare Launch Arguments
    # -----------------------------
    launch_arguments = [
        DeclareLaunchArgument(
            robot_ip_parameter,
            default_value='172.16.0.2',
            description='Hostname or IP address of the robot.'
        ),
        DeclareLaunchArgument(
            arm_id_parameter,
            default_value='fr3',
            description='ID of the type of arm used. Supported values: fer, fr3, fp3'
        ),
        DeclareLaunchArgument(
            use_rviz_parameter,
            default_value='true',
            description='Visualize the robot in RViz.'
        ),
        DeclareLaunchArgument(
            use_fake_hardware_parameter,
            default_value='false',
            description='Use fake hardware.'
        ),
        DeclareLaunchArgument(
            fake_sensor_commands_parameter,
            default_value='false',
            description='Fake sensor commands. Only valid when fake hardware is enabled.'
        ),
        DeclareLaunchArgument(
            load_gripper_parameter,
            default_value='true',
            description='Use Franka Gripper as an end-effector. If false, the robot is loaded without a gripper.'
        ),
        DeclareLaunchArgument(
            cartesian_impedance_controller_config_path_parameter,
            default_value=TextSubstitution(text=default_cartesian_impedance_controller_config_path),
            description='(string) Path to the parameter configuration file for cartesian_impedance_controller.'
        )
    ]

    # -----------------------------
    # Define Nodes from Code 1
    # -----------------------------
    # Spawn joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['franka/joint_states', 'franka_gripper/joint_states'],
            'rate': 30
        }]
    )

    # Create an opaque function to spawn nodes that depend on the robot description
    robot_description_nodes = OpaqueFunction(
        function=robot_description_dependent_nodes_spawner,
        args=[robot_ip, arm_id, use_fake_hardware, fake_sensor_commands, load_gripper]
    )

    # Spawn the joint state broadcaster controller
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Spawn the Franka robot state broadcaster (only when not using fake hardware)
    franka_robot_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['franka_robot_state_broadcaster'],
        parameters=[{'arm_id': arm_id}],
        output='screen',
        condition=UnlessCondition(use_fake_hardware)
    )

    # -----------------------------
    # Optional: Include the gripper launch file
    # -----------------------------
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('franka_gripper'),
                'launch', 'gripper.launch.py'
            ])
        ]),
        launch_arguments={
            robot_ip_parameter: robot_ip,
            use_fake_hardware_parameter: use_fake_hardware
        }.items(),
        condition=IfCondition(load_gripper)
    )

    # -----------------------------
    # Visualization: RViz Node
    # -----------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['--display-config', rviz_file],
        condition=IfCondition(use_rviz)
    )

    # -----------------------------
    # Node from Code 2: Cartesian Impedance Controller Spawner
    # -----------------------------
    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_impedance_controller",
            "--param-file",
            cartesian_impedance_controller_config_path
        ],
        output="screen"
    )

    # Node for New Goal Pose
    new_goal_pose = Node(
        package='middle_nodes',
        executable='modify_pose.py',
        output='screen'
    )

    # -----------------------------
    # Construct and Return Launch Description
    # -----------------------------
    ld = LaunchDescription(launch_arguments)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_description_nodes)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(franka_robot_state_broadcaster_spawner)
    ld.add_action(gripper_launch)
    ld.add_action(rviz_node)
    ld.add_action(cartesian_impedance_controller_spawner)
    ld.add_action(new_goal_pose)

    return ld


if __name__ == '__main__':
    # For debugging: print the generated launch description
    ld = generate_launch_description()
    print(ld)
