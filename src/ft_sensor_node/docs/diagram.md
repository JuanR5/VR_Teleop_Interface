---
config:
  theme: mc
  look: classic
---
classDiagram
direction TB
    class FTSensorNode {
	    - publisher: Publisher
	    - sensor: MyBotaForceTorqueSensorComm
	    - frame_id: string
	    - sensor_thread: thread
	    - running: bool
	    + FTSensorNode()
	    + ~FTSensorNode()
	    - sensorLoop()
    }
    class BotaForceTorqueSensorComm {
	    # frame: SensorFrame
	    + serialReadBytes(data: uint8_t*, len: size_t) : int
	    + serialAvailable() : int
	    + readFrame() : int
	    + get_crc_count() : int
	    + serialReadBytes() : virtual
	    + serialAvailable() : virtual
    }
    class FTFilterNode {
	    - max_range: double
	    - calibration_time: double
	    - filter_window_size: int
	    - detection_threshold: double
	    - high_frequency: double
	    - low_frequency: double
	    - force_weight: double
	    - torque_weight: double
	    - torque_deadband: double
	    - calibration_start_time: Time
	    - reference_forces: list
	    - reference_torques: list
	    - calibration_data_forces: list of lists
	    - calibration_data_torques: list of lists
	    - filter_buffer_forces: list of lists
	    - filter_buffer_torques: list of lists
	    - is_calibrating: bool
	    - subscriber: Subscription
	    - publisher: Publisher
	    + FTFilterNode()
	    - ftCallback(msg: Wrench)
	    - calibrateSensor(forces, torques)
	    - applyMovingAverage(buffer, new_values)
	    - computeAverage(data)
	    - computeIntensity(forces, torques) : double
	    - publishOutput(intensity, frequency)
	    - getMaxIntensityVariable(forces, torques) : string
	    - roundToDecimal(value, decimal_places) : double
    }
    class ft_sensor_launch {
	    + generate_launch_description()
    }
    class Float32MultiArray {
    }

    FTSensorNode <-- BotaForceTorqueSensorComm : sensor
    FTSensorNode --> FTFilterNode : publishes Wrench
    FTFilterNode --> FTSensorNode : subscribes "ft_sensor/data"
    FTFilterNode --> Float32MultiArray : publishes "rumble_output"
    ft_sensor_launch --> FTSensorNode : launches
    ft_sensor_launch --> FTFilterNode : launches
    ft_sensor_launch --> BotaForceTorqueSensorComm : launches
