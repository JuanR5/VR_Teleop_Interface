´´´mermaid
classDiagram
direction TB
class Quest2ControllerInput {
    + static float LeftTriggerValue
    + static float RightTriggerValue
    + static event OnThumbstickMoved(string, Vector2)
    + static event OnButtonPressed(string)
    + static event OnButtonReleased(string)
    - lastRightThumbstick: Vector2
    - lastLeftThumbstick: Vector2
    - joystickThreshold: float
    - triggerThreshold: float
    + Update()
}
class ControllerMovementPublisher {
    - ros: ROSConnection
    - topicName: string
    - gripperTopic: string
    - publishRate: float
    - triggerThreshold: float
    - timeElapsed: float
    - positionVector: Vector3
    - rotationVector: Vector3
    + Start()
    + OnDestroy()
    + Update()
    - HandleThumbstickInput()
    - HandleButtonInput()
    - HandleButtonRelease()
    - PublishMovement()
    - PublishGripperCommand()
}
class RumbleSubscriber {
    - ros: ROSConnection
    - rumbleOutputTopic: string
    - vibrationManager: ControllerVibrationManager
    + Start()
    - HandleRumbleMessage(msg: Float32MultiArrayMsg)
}
class ControllerVibrationManager {
    + TriggerVibration(controller: OVRInput.Controller, amplitude: float, duration: float)
    - StopVibration()
}
ControllerMovementPublisher --> Quest2ControllerInput : subscribes events
RumbleSubscriber --> ControllerVibrationManager : triggers
RumbleSubscriber --> ROSConnection : subscribes "/rumble_output"
ControllerMovementPublisher --> ROSConnection : publishes "/controller_movement", "/gripper_command"
´´´