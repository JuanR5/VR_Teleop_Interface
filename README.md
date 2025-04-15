# ROS 2 Integration for Franka Robotics Research Robots

[![CI](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/frankaemika/franka_ros2/actions/workflows/ci.yml)

This repository contains the full ROS 2 workspace setup to interface with the **Franka Emika Research 3 robotic arm** for **teleoperation**. This setup is designed to work with ROS 2 and leverages the **Franka Control Interface (FCI)** for real-time control.

Another component of this integration is the use of the **SenseOne 6-axis force-torque sensor from Botasys**, mounted at the end effector of the robotic arm. The sensor data is utilized to enable haptic feedback teleoperation modes.

---

## Features

- ✅ Full ROS 2 integration with Franka Research 3 hardware via FCI
- 🧠 Real-time force/torque feedback using the SenseOne sensor from Botasys
- 🔄 Modular and extensible ROS 2 node structure
- 🧪 Easily adaptable for research in haptics, human-robot interaction, and impedance control
- 📦 Includes example launch files and configuration for quick startup

---

## Project Structure

```
ros2_ws/
├── src/
│   ├── franka_ros2/             # Official ROS 2 interface to Franka Emika and all related packages
│   ├── franka_teleop_pkg/       # Custom teleoperation control nodes
|   ├── franka_teleop_pkg_interfaces/    #generated with the pkg generator
│   ├── ft_sensor_node/          # Botasys SenseOne sensor drivers and interfaces
│   ├── middle_nodes/            # Additional nodes for data processing and control
│   ├── ros_tcp_endpoint/        # ROS 2 TCP endpoint for remote control
│   └── robot_launch/            # Unified launch files and configurations
├── install/ #Not included
├── build/ #Not included
└── log/ #Not included
```

---

## Getting Started

### Prerequisites

- Franka Research 3 robotic arm
- Ubuntu 22.04, Real time kernel
- Docker

---
## Franka Research 3

Please make sure everything is set up correctly, you followed the Real time kernel configuration, the FCI, the minimun requirements for 1kHz, If working with docker, have all the permisions and cpu acces. And to configure adequatly the cpu to performance [CPU Scaling](https://frankaemika.github.io/docs/troubleshooting.html#disabling-cpu-frequency-scaling)
also, when building the workspace in ros2, take into account that it must be done with ´´´ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release ´´´ and that some packages must be built before others due to requirements and dependencies. so first franka_msg, franka_hardware, franka_semantic_components. then ´´´ source install/setup.bash´´´ and the build the rest of the packages

---

## Running the System

### Launch Franka Robot with Teleoperation

```bash
ros2 launch robot_launch bringup_teleop.launch.py
```

This launch file will:

- Connect to the Franka robot via FCI
- Start the teleoperation control node
- Launch the SenseOne FT sensor node for data publishing
- Optionally visualize the robot in RViz2

---

## SenseOne Force-Torque Sensor

The SenseOne sensor is mounted at the end effector and is used to capture real-time force and torque data. This data is published as a standard `geometry_msgs/WrenchStamped` message and is integrated into the control loop to enable: haptic feedback applications

---

## Teleoperation Capabilities

Teleoperation can be achieved using supported input devices such as:

- Meta Quest 2
- Keyboard (limited functionality)

The control node maps device inputs to Cartesian commands.

---

## User testing

For Automated data collection a bash script is added wich will generate a rosbag with topics of interes, measure task execution, and allow to write down aditional notes for each task. `newTest.sh`

---

## Example Applications

- Remote manipulation with real-time force feedback
- Human-in-the-loop control experiments
- Assistive robotic tasks with compliant behaviors
- Research in haptic feedback and shared control

---

## Diagrams

### Franka Control Diagram

```mermaid
classDiagram
direction TB
class EquilibriumPosePublisher {
    - deadband: float
    - step_linear: float
    - step_angular: float
    - filter_alpha: float
    - latest_cmd: np.array[6]
    - has_new_input: bool
    - current_pose: PoseStamped
    - desired_pose: PoseStamped
    - sub_movement: Subscription<Twist>
    - sub_current_pose: Subscription<PoseStamped>
    - pub_equilibrium_pose: Publisher<PoseStamped>
    - timer: Timer
    + controller_movement_callback(msg: Twist)
    + current_pose_callback(msg: PoseStamped)
    + timer_callback()
}
class CartesianImpedanceController {
    - stiffness: Matrix6d
    - damping: Matrix6d
    - nullspace_stiffness: double
    - joint_names: vector<string>
    - arm_id: string
    - robot_state: franka::RobotState
    - pose_equilibrium: Pose
    - k_gains: Vector
    - d_gains: Vector
    - filter_params: struct
    + init()
    + update()
    + starting()
    + stopping()
    + setEquilibriumPose(pose: Pose)
    + calculateTorques()
}
class teleop_launch {
    <<launch>>
    + spawns EquilibriumPosePublisher
    + spawns CartesianImpedanceController
    + Include: GripperLaunch (condition: load_gripper)
    + robot_state_publisher
    + ros2_control_node
}
class Topic_new_goal_pose {
    - new_goal_pose: PoseStamped
}
EquilibriumPosePublisher --> Topic_new_goal_pose : publishes
Topic_new_goal_pose --> CartesianImpedanceController : subscribes
teleop_launch --> EquilibriumPosePublisher : launches
teleop_launch --> CartesianImpedanceController : spawns
```

### Gripper Diagram

```mermaid
classDiagram
direction TB
class GripperCommandNode {
    - max_width: float
    - open_speed: float
    - grasp_speed: float
    - grasp_force: float
    - grasp_epsilon: float
    - current_state: str
    - subscription: Subscription
    - _move_client: ActionClient
    - _grasp_client: ActionClient
    + __init__()
    + command_callback(msg: Float32MultiArray)
    - send_gripper_command(target_state: str)
    - send_move_goal()
    - move_response_callback(future)
    - move_result_callback(future)
    - send_grasp_goal()
    - grasp_response_callback(future)
    - grasp_result_callback(future)
}
class Move {
    <<action>>
    + Goal: width, speed
}
class Grasp {
    <<action>>
    + Goal: width, speed, force, epsilon
}
class Float32MultiArray {
    <<message>>
}
class Subscription {
    <<interface>>
}
class ActionClient {
    <<interface>>
}
GripperCommandNode --> Move : uses as action
GripperCommandNode --> Grasp : uses as action
GripperCommandNode --> Float32MultiArray : subscribes "/gripper_command"
GripperCommandNode --> Subscription : uses
GripperCommandNode --> ActionClient : uses
class GripperLaunch {
    <<launch>>
    + robot_ip : LaunchArgument
    + Include: franka_gripper/launch/gripper.launch.py
    + Node: gripper_command_node
}
class TeleopLaunch {
    <<launch>>
    + Include: GripperLaunch (condition: load_gripper)
    + robot_state_publisher
    + ros2_control_node
    + controller_spawners
}
TeleopLaunch --> GripperLaunch : includes
GripperLaunch --> GripperCommandNode : launches
```

### Force-Torque Sensor Diagram

```mermaid
classDiagram
    direction TB
    class FTSensorNode {
        - publisher_: Publisher<Wrench>
        - sensor_: MyBotaForceTorqueSensorComm
        - frame_id_: string
        - sensor_thread_: thread
        - running_: ~bool~
        + FTSensorNode()
        + ~FTSensorNode()
        - sensorLoop()
    }

    class BotaForceTorqueSensorCom {
        + serialReadBytes(data: uint8_t*, len: size_t): int
        + serialAvailable(): int
        + readFrame(): int
        + get_crc_count(): int
        # frame: SensorFrame
    }

    class FTFilterNode {
        - max_range_: double
        - calibration_time_: double
        - filter_window_size_: int
        - detection_threshold_: double
        - high_frequency_: double
        - low_frequency_: double
        - force_weight_: double
        - torque_weight_: double
        - torque_deadband_: double
        - calibration_start_time_: Time
        - reference_forces_: vector~double~
        - reference_torques_: vector~double~
        - calibration_data_forces_: vector~vector~double~~
        - calibration_data_torques_: vector~vector~double~~
        - filter_buffer_forces_: vector~vector~double~~
        - filter_buffer_torques_: vector~vector~double~~
        - is_calibrating_: bool
        - subscriber_: Subscription<Wrench>
        - publisher_: Publisher<Float32MultiArray>
        + FTFilterNode()
        - ftCallback(msg: Wrench)
        - calibrateSensor(forces, torques)
        - applyMovingAverage(buffer, new_values)
        - computeAverage(data)
        - computeIntensity(forces, torques): double
        - publishOutput(intensity, frequency)
        - getMaxIntensityVariable(forces, torques): string
        - roundToDecimal(value, decimal_places): double
    }
    class Float32MultiArray
    class ft_sensor_launch {
        + generate_launch_description()
    }
    FTSensorNode --> FTFilterNode : publishes Wrench
    FTFilterNode --> FTSensorNode : subscribes "ft_sensor/data"
    FTFilterNode --> Float32MultiArray : publishes "rumble_output"
    ft_sensor_launch --> BotaForceTorqueSensorCom : launch
    ft_sensor_launch --> FTSensorNode : launch
    ft_sensor_launch --> FTFilterNode : launch
    BotaForceTorqueSensorCom --> FTSensorNode : sensor
```

---

## License

All packages of `franka_ros2` are licensed under the [Apache 2.0 license](https://www.apache.org/licenses/LICENSE-2.0.html).

See the [Franka Control Interface (FCI) documentation](https://frankaemika.github.io/docs) for more detailed information about low-level control features and APIs.

---

## Acknowledgments

This project builds upon the excellent work from the following repositories and contributors:

- [frankaemika/franka_ros2](https://github.com/frankaemika/franka_ros2) – Official ROS 2 interface for the Franka Emika robot
- [botasys/SenseOne-ROS](https://gitlab.com/botasys/bota_serial_driver/-/tree/master?ref_type=heads) – C++ Driver for the SenseOne force-torque sensor
- [ros2-controller-package-create](https://github.com/jellehierck/ros2-pkg-create/tree/controllers-package) – Thanks to Jelle Hierck, for the automatic package generator with a franka controller template.
- [ROS-TCP-EndPoint](https://github.com/Unity-Technologies/ROS-TCP-Endpoint/tree/main-ros2)
- [Cartesian Impedance Controller](https://github.com/sp-sophia-labs/franka_ros2) – Implementation of Cartesian Impedance Controller for the Franka Emika Robot ROS2 
- The ROS 2 community and contributors for ongoing development of robust robotics middleware

Special thanks to the research community for their input and suggestions, and to all open-source contributors who make robotic development more accessible.

---
