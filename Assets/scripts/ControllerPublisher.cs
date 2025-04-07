using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System;

public class ControllerMovementPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "controller_movement";
    public string gripperTopic = "gripper_command";

    public float publishRate = 10f;
    private float timeElapsed;

    // XYZ from right controller, RPY from left controller
    private Vector3 positionVector = Vector3.zero;  // For linear movement (XYZ)
    private Vector3 rotationVector = Vector3.zero;  // For angular movement (RPY)

    // Threshold for triggers (0-1) to decide if the trigger is "pressed"
    public float triggerThreshold = 0.5f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
        ros.RegisterPublisher<Float32MultiArrayMsg>(gripperTopic);

        timeElapsed = 0f;

        // Subscribe to controller events (these events are published from your Quest2ControllerInput script)
        Quest2ControllerInput.OnThumbstickMoved += HandleThumbstickInput;
        Quest2ControllerInput.OnButtonPressed += HandleButtonInput;
        Quest2ControllerInput.OnButtonReleased += HandleButtonRelease;
    }

    void OnDestroy()
    {
        Quest2ControllerInput.OnThumbstickMoved -= HandleThumbstickInput;
        Quest2ControllerInput.OnButtonPressed -= HandleButtonInput;
        Quest2ControllerInput.OnButtonReleased -= HandleButtonRelease;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed >= 1f / publishRate)
        {
            PublishMovement();
            timeElapsed = 0f;
        }
    }

    private void HandleThumbstickInput(string thumbstick, Vector2 value)
    {
        if (thumbstick == "Right Thumbstick")
        {
            positionVector.x = value.x;
            positionVector.y = value.y;
        }
        else if (thumbstick == "Left Thumbstick")
        {
            rotationVector.x = value.x; // Roll
            rotationVector.y = value.y; // Pitch
        }
    }

    private void HandleButtonInput(string button)
    {
        if (button == "B Button (Right Controller)")
            positionVector.z = 1f;
        if (button == "A Button (Right Controller)")
            positionVector.z = -1f;

        if (button == "Y Button (Left Controller)")
            rotationVector.z = 1f;  // Yaw right
        if (button == "X Button (Left Controller)")
            rotationVector.z = -1f; // Yaw left
    }

    private void HandleButtonRelease(string button)
    {
        if (button == "B Button (Right Controller)" || button == "A Button (Right Controller)")
            positionVector.z = 0f;

        if (button == "Y Button (Left Controller)" || button == "X Button (Left Controller)")
            rotationVector.z = 0f;
    }

    private void PublishMovement()
    {
        // Publish the movement command as a Twist message
        TwistMsg twistMsg = new TwistMsg
        {
            linear = new RosMessageTypes.Geometry.Vector3Msg(positionVector.y, -positionVector.x, positionVector.z),
            angular = new RosMessageTypes.Geometry.Vector3Msg(rotationVector.x, rotationVector.y, rotationVector.z)
        };

        ros.Publish(topicName, twistMsg);
        Debug.Log($"Published Twist: Linear({positionVector}), Angular({rotationVector})");

        // Publish the gripper command based on trigger inputs
        PublishGripperCommand();
    }

    private void PublishGripperCommand()
    {
        // Instead of reading the controller triggers directly, we assume that
        // Quest2ControllerInput already updates these static properties.
        // (Make sure your Quest2ControllerInput script defines these.)
        float leftTriggerValue = Quest2ControllerInput.LeftTriggerValue;
        float rightTriggerValue = Quest2ControllerInput.RightTriggerValue;

        // Convert the analog trigger values to binary states: 1 if pressed above threshold, 0 otherwise.
        int leftState = leftTriggerValue >= triggerThreshold ? 1 : 0;
        int rightState = rightTriggerValue >= triggerThreshold ? 1 : 0;

        // Create and publish the gripper command as a Float32MultiArray.
        Float32MultiArrayMsg gripperMsg = new Float32MultiArrayMsg();
        gripperMsg.data = new float[] { leftState, rightState };

        ros.Publish(gripperTopic, gripperMsg);
        Debug.Log($"Published Gripper Command: [{leftState}, {rightState}]");
    }
}
