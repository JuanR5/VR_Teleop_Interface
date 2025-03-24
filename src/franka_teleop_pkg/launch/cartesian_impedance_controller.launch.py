from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Define common paths ----------

    default_cartesian_impedance_controller_config_path = str(
        Path(get_package_share_directory("franka_teleop_pkg")) / "config" / "cartesian_impedance_controller.yaml"
    )

    # ---------- Declare launch arguments ----------

    launch_args = []

    # Specify the parameter configuration file path for cartesian_impedance_controller
    cartesian_impedance_controller_config_path_arg = DeclareLaunchArgument(
        "cartesian_impedance_controller_config_path",
        default_value=TextSubstitution(text=default_cartesian_impedance_controller_config_path),
        description="(string) Path to the parameter configuration file for cartesian_impedance_controller.",
    )
    launch_args.append(cartesian_impedance_controller_config_path_arg)

    # ---------- Declare executables ----------

    # Controller spawner executable
    cartesian_impedance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cartesian_impedance_controller",
            # "--inactive",  # Optional: when enabled, load and configure the controller but do not activate
            "--param-file",
            LaunchConfiguration(cartesian_impedance_controller_config_path_arg.name),
        ],
        output="screen",
    )

    # ---------- Construct launch description ----------

    return LaunchDescription(
        [
            *launch_args,  # Unpack the launch arguments
            cartesian_impedance_controller_spawner,
        ]
    )
