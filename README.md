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
│   ├── franka_ros2/             # Official ROS 2 interface to Franka Emika
│   ├── teleop_control/          # Custom teleoperation control nodes
│   ├── senseone_ft_sensor/      # Botasys SenseOne sensor drivers and interfaces
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

## Example Applications

- Remote manipulation with real-time force feedback
- Human-in-the-loop control experiments
- Assistive robotic tasks with compliant behaviors
- Research in haptic feedback and shared control

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
- The ROS 2 community and contributors for ongoing development of robust robotics middleware

Special thanks to the research community for their input and suggestions, and to all open-source contributors who make robotic development more accessible.

---
