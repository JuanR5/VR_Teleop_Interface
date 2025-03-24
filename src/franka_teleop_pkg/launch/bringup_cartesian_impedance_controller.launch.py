from pathlib import Path

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    GroupAction, 
    IncludeLaunchDescription, 
    DeclareLaunchArgument, 
    OpaqueFunction,
    SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_robot_description(context):
    # Retrieve parameters from the launch configuration.
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    arm_id = LaunchConfiguration("arm_id").perform(context)
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands").perform(context)
    load_gripper = LaunchConfiguration("load_gripper").perform(context)

    # Build the path to the xacro file.
    franka_xacro_filepath = os.path.join(
        get_package_share_directory("franka_description"),
        "robots",
        arm_id,
        arm_id + ".urdf.xacro"
    )
    # Process the xacro file to generate the URDF string.
    doc = xacro.process_file(
        franka_xacro_filepath,
        mappings={
            "ros2_control": "true",
            "arm_id": arm_id,
            "robot_ip": robot_ip,
            "hand": load_gripper,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
        }
    )
    return doc.toprettyxml(indent="  ")


def set_robot_description_action(context, *args, **kwargs):
    # Compute the robot_description and set it as a launch configuration variable.
    rd = generate_robot_description(context)
    return [SetLaunchConfiguration("robot_description", rd)]


def generate_launch_description() -> LaunchDescription:
    """Bring up the Franka robot for the hand-eye calibration."""
    # ---------- Define common paths ----------

    # ---------- Declare launch arguments ----------

    launch_args = [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.16.0.2",
            description="IP address of the robot"
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fr3",
            description="ID of the robot arm (e.g. fr3)"
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Whether to use fake hardware"
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Fake sensor commands (only valid if use_fake_hardware is true)"
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Whether to load the Franka gripper"
        ),
    ]

    # Set the unified robot_description launch configuration.
    set_rd = OpaqueFunction(function=set_robot_description_action)

    # ---------- Declare executables ----------

    # Controller manager
    controller_manager_launch = GroupAction(
        actions=[
            # Define remappings which are applied to all nodes within this GroupAction
            SetRemap("joint_states", "franka/joint_states"),
            SetRemap("controller_manager/robot_description", "robot_description"),

            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("franka_teleop_pkg")) / "launch" / "controller_manager.launch.py"),
                    ]
                ),
                launch_arguments={
                    "robot_description": LaunchConfiguration("robot_description")
                }.items(),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True  # Any changes to launch arguments inside this GroupAction do not "spill" outside of the GroupAction
    )

    # Robot description
    robot_description_launch = GroupAction(
        actions=[
            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("franka_teleop_pkg")) / "launch" / "robot_description.launch.py"),
                    ]
                ),
                launch_arguments={
                    # Due to a bug in ros2_launch, required launch arguments are passed explicitly as a workaround
                    # See https://github.com/ros2/launch/issues/749
                    "robot_ip": LaunchConfiguration("robot_ip"),
                    "arm_id": LaunchConfiguration("arm_id"),
                    "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                    "fake_sensor_commands": LaunchConfiguration("fake_sensor_commands"),
                    "load_gripper": LaunchConfiguration("load_gripper"),
                }.items(),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True  # Any changes to launch arguments inside this GroupAction do not "spill" outside of the GroupAction
    )


    # cartesian_impedance_controller spawner
    cartesian_impedance_controller_launch = GroupAction(
        actions=[
            # Include the controller manager nodes
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        str(Path(get_package_share_directory("franka_teleop_pkg")) / "launch" / "cartesian_impedance_controller.launch.py"),
                    ]
                ),
            ),
        ],
        forwarding=True,  # Make the launch arguments from the current launch file available inside this GroupAction
        scoped=True  # Any changes to launch arguments inside this GroupAction do not "spill" outside of the GroupAction
    )



    # ---------- Construct launch description ----------

    return LaunchDescription(
        launch_args + [
            set_rd,
            controller_manager_launch,
            robot_description_launch,
            cartesian_impedance_controller_launch,
        ]
    )
