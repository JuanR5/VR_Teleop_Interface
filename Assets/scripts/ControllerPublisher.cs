using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using System;

/// <summary>
/// Publishes controller movement and trigger inputs to ROS using Twist and Float32MultiArray messages.
/// Combines thumbstick and button events to generate linear and angular control commands.
/// </summary>
public class ControllerMovementPublisher : MonoBehaviour
{
    private ROSConnection ros;

    /// <summary>
    /// ROS topic for publishing Twist movement messages.
    /// </summary>
    public string topicName = "controller_movement";

    /// <summary>
    /// ROS topic for publishing gripper trigger states.
    /// </summary>
    public string gripperTopic = "gripper_command";

    /// <summary>
    /// Frequency (Hz) at which data is published.
    /// </summary>
    public float publishRate = 10f;

    private float timeElapsed;

    private Vector3 positionVector = Vector3.zero;
    private Vector3 rotationVector = Vector3.zero;

    /// <summary>
    /// Threshold to convert analog trigger values into binary states.
    /// </summary>
    public float triggerThreshold = 0.5f;

    /// <summary>
    /// Initializes ROS publishers and subscribes to Quest2 input events.
    /// </summary>
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);
        ros.RegisterPublisher<Float32MultiArrayMsg>(gripperTopic);

        timeElapsed = 0f;

        Quest2ControllerInput.OnThumbstickMoved += HandleThumbstickInput;
        Quest2ControllerInput.OnButtonPressed += HandleButtonInput;
        Quest2ControllerInput.OnButtonReleased += HandleButtonRelease;
    }

    /// <summary>
    /// Unsubscribes from controller events.
    /// </summary>
    void OnDestroy()
    {
        Quest2ControllerInput.OnThumbstickMoved -= HandleThumbstickInput;
        Quest2ControllerInput.OnButtonPressed -= HandleButtonInput;
        Quest2ControllerInput.OnButtonReleased -= HandleButtonRelease;
    }

    /// <summary>
    /// Publishes messages at fixed rate based on timeElapsed.
    /// </summary>
    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed >= 1f / publishRate)
        {
            PublishMovement();
            timeElapsed = 0f;
        }
    }

    /// <summary>
    /// Maps thumbstick input to position or rotation vectors.
    /// </summary>
    /// <param name="thumbstick">The thumbstick source.</param>
    /// <param name="value">The 2D axis input value.</param>
    private void HandleThumbstickInput(string thumbstick, Vector2 value)
    {
        if (thumbstick == "Right Thumbstick")
        {
            positionVector.x = value.x;
            positionVector.y = value.y;
        }
        else if (thumbstick == "Left Thumbstick")
        {
            rotationVector.x = value.x;
            rotationVector.y = value.y;
        }
    }

    /// <summary>
    /// Handles button presses and maps them to Z-axis movement and yaw.
    /// </summary>
    private void HandleButtonInput(string button)
    {
        if (button == "B Button (Right Controller)")
            positionVector.z = 1f;
        if (button == "A Button (Right Controller)")
            positionVector.z = -1f;

        if (button == "Y Button (Left Controller)")
            rotationVector.z = 1f;
        if (button == "X Button (Left Controller)")
            rotationVector.z = -1f;
    }

    /// <summary>
    /// Resets Z-axis movement or yaw when buttons are released.
    /// </summary>
    private void HandleButtonRelease(string button)
    {
        if (button == "B Button (Right Controller)" || button == "A Button (Right Controller)")
            positionVector.z = 0f;

        if (button == "Y Button (Left Controller)" || button == "X Button (Left Controller)")
            rotationVector.z = 0f;
    }

    /// <summary>
    /// Publishes the current movement state to ROS.
    /// </summary>
    private void PublishMovement()
    {
        TwistMsg twistMsg = new TwistMsg
        {
            linear = new Vector3Msg(positionVector.y, -positionVector.x, positionVector.z),
            angular = new Vector3Msg(rotationVector.x, rotationVector.y, rotationVector.z)
        };

        ros.Publish(topicName, twistMsg);
        Debug.Log($"Published Twist: Linear({positionVector}), Angular({rotationVector})");

        PublishGripperCommand();
    }

    /// <summary>
    /// Publishes gripper open/close command to ROS based on trigger inputs.
    /// </summary>
    private void PublishGripperCommand()
    {
        float leftTriggerValue = Quest2ControllerInput.LeftTriggerValue;
        float rightTriggerValue = Quest2ControllerInput.RightTriggerValue;

        int leftState = leftTriggerValue >= triggerThreshold ? 1 : 0;
        int rightState = rightTriggerValue >= triggerThreshold ? 1 : 0;

        Float32MultiArrayMsg gripperMsg = new Float32MultiArrayMsg
        {
            data = new float[] { leftState, rightState }
        };

        ros.Publish(gripperTopic, gripperMsg);
        Debug.Log($"Published Gripper Command: [{leftState}, {rightState}]");
    }
}
