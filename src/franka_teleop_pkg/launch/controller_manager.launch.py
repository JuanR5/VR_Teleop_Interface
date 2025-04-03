from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description() -> LaunchDescription:
    # ---------- Define common paths ----------
    default_controller_manager_config_path = str(
        Path(get_package_share_directory("franka_teleop_pkg")) / "config" / "controller_manager.yaml"
    )

    # ---------- Declare launch arguments ----------
    launch_args = []

    # Parameter configuration file for the controller manager.
    controller_manager_config_path_arg = DeclareLaunchArgument(
        "controller_manager_config_path",
        default_value=TextSubstitution(text=default_controller_manager_config_path),
        description="Path to the parameter configuration file for the controller manager.",
    )
    launch_args.append(controller_manager_config_path_arg)

    # Declare the robot_description argument; its value should be provided as a string.
    robot_description_arg = DeclareLaunchArgument(
        "robot_description",
        default_value="",
        description="Robot description URDF string",
    )
    launch_args.append(robot_description_arg)

    # ---------- Declare executables ----------
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration(controller_manager_config_path_arg.name),
            {
                "robot_description": ParameterValue(
                    LaunchConfiguration("robot_description"), value_type=str
                )
            },
        ],
        output={"stdout": "screen", "stderr": "screen"},
        on_exit=Shutdown(),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output={"stdout": "screen", "stderr": "screen"},
    )

    return LaunchDescription(
        launch_args + [
            controller_manager,
            joint_state_broadcaster,
        ]
    )
