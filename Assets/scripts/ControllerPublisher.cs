using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using System;

public class ControllerMovementPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "controller_movement";

    public float publishRate = 10f;
    private float timeElapsed;

    // XYZ from right controller, RPY from left controller
    private Vector3 positionVector = Vector3.zero;  // For XYZ
    private Vector3 rotationVector = Vector3.zero;  // For RPY

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TwistMsg>(topicName);

        timeElapsed = 0f;

        // Subscribe to controller events
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
        TwistMsg twistMsg = new TwistMsg
        {
            linear = new RosMessageTypes.Geometry.Vector3Msg(positionVector.x, positionVector.y, positionVector.z),
            angular = new RosMessageTypes.Geometry.Vector3Msg(rotationVector.x, rotationVector.y, rotationVector.z)
        };

        ros.Publish(topicName, twistMsg);
        Debug.Log($"Published Twist: Linear({positionVector}), Angular({rotationVector})");
    }
}