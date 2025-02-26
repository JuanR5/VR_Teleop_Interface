using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System;

public class ControllerMovementPublisher : MonoBehaviour
{
    private ROSConnection ros;
    public string topicName = "controller_movement";

    public float publishRate = 10f;
    private float timeElapsed;

    // XYZ from right controller, RPY from left controller
    private Vector3Int positionVector = Vector3Int.zero;  // For XYZ
    private Vector3Int rotationVector = Vector3Int.zero;  // For RPY

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Int32MultiArrayMsg>(topicName);

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
        // Right controller handles X and Y position
        if (thumbstick == "Right Thumbstick")
        {
            positionVector.x = value.x > 0 ? 1 : (value.x < 0 ? -1 : 0);
            positionVector.y = value.y > 0 ? 1 : (value.y < 0 ? -1 : 0);
        }
        // Left controller handles Roll and Pitch
        else if (thumbstick == "Left Thumbstick")
        {
            rotationVector.x = value.x > 0 ? 1 : (value.x < 0 ? -1 : 0); // Roll
            rotationVector.y = value.y > 0 ? 1 : (value.y < 0 ? -1 : 0); // Pitch
        }
    }

    private void HandleButtonInput(string button)
    {
        // Right controller buttons handle Z position
        if (button == "B Button (Right Controller)")
            positionVector.z = 1;
        if (button == "A Button (Right Controller)")
            positionVector.z = -1;

        // Left controller buttons handle Yaw
        if (button == "Y Button (Left Controller)")
            rotationVector.z = 1;  // Yaw right
        if (button == "X Button (Left Controller)")
            rotationVector.z = -1; // Yaw left
    }

    private void HandleButtonRelease(string button)
    {
        // Reset Z position on right controller button release
        if (button == "B Button (Right Controller)" || button == "A Button (Right Controller)")
            positionVector.z = 0;

        // Reset Yaw on left controller button release
        if (button == "Y Button (Left Controller)" || button == "X Button (Left Controller)")
            rotationVector.z = 0;
    }

    private void PublishMovement()
    {
        // Combine position (XYZ) and rotation (RPY) into one array
        Int32MultiArrayMsg movementMsg = new Int32MultiArrayMsg
        {
            data = new int[] 
            { 
                positionVector.x, positionVector.y, positionVector.z,  // XYZ position
                rotationVector.x, rotationVector.y, rotationVector.z   // RPY rotation
            }
        };

        ros.Publish(topicName, movementMsg);
        Debug.Log($"Published Position: {positionVector}, Rotation: {rotationVector}");
    }
}