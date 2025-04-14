---
config:
  look: classic
  layout: elk
  theme: mc
---
flowchart TB
 subgraph ROS["ROS TCP Connector/ Topics"]
        R1[/"controller_movement Twist"/]
        R2[/"gripper_command Float32MultiArray"/]
        R3[/"zed2_unity: left & right images"/]
        R4[/"rumble_output Float32MultiArray"/]
  end
 subgraph UNITY_VR["UNITY VR System (Windows 11)"]
        U1["Quest2ControllerInput.cs"]
        U2["ControllerMovementPublisher.cs"]
        U3["ControllerVibrationManager.cs"]
        U4["RumbleSubscriber.cs"]
        U5["StereoCameraManager.cs"]
        U6["StereoCameraFeed.cs / StereoImageSubscriber.cs"]
        U7["SSHRunner.cs"]
        U8["VR Headset"]
        ROS
  end
 subgraph FRANKA["FRANKA CONTROL     (Ubuntu 22.04)"]
        F1["modify_pose.py"]
        F2["cartesian_impedance_controller.cpp"]
        F3["gripper_command.py"]
        F4["ft_sensor_node + ft_filter_node"]
  end
 subgraph ZED["ZED System (Camera | Ubuntu 22.04 | Docker)"]
        Z1["zed_wrapper.launch.py (ZED Driver)"]
        Z2["zed_image_bridge.py"]
        Z3["ros_tcp_endpoint"]
  end
    U1 --> U2
    U2 -- Publishes Twist & Gripper --> R1
    R4 --> U4
    U4 --> U3
    U5 --> U6
    U6 --> U8
    U7 -- Triggers via docker compose --> Z1
    R1 --> F1
    F1 --> F2
    R2 --> F3
    F4 -- Sends /rumble_output --> R4
    Z1 --> Z2
    Z2 -- Left: /zed2_unity/left_image --> R3
    Z2 -- Right: /zed2_unity/right_image --> R3
    R3 --> U6
