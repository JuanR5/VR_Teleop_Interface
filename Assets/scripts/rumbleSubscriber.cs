using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // For Float32 message type
using System; // For Math.Clamp

public class RumbleSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ControllerVibrationManager vibrationManager;

    // Define the topic name
    private string rumbleOutputTopic = "/rumble_output";

    // Vibration duration (you can make this configurable or part of the ROS message if needed)
    public float vibrationDuration = 0.1f;

    private void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        vibrationManager = FindObjectOfType<ControllerVibrationManager>(); // Find the vibration manager in the scene

        if (vibrationManager == null)
        {
            Debug.LogError("ControllerVibrationManager not found in the scene!");
            enabled = false; // Disable this script if the manager is missing
            return;
        }

        // Subscribe to the rumble output topic, using Float32 message type
        ros.Subscribe<Float32Msg>(rumbleOutputTopic, HandleRumbleMessage);

        Debug.Log($"Subscribed to {rumbleOutputTopic}");
    }

    private void HandleRumbleMessage(Float32Msg msg)
    {
        // Get the amplitude from the message
        float amplitude = msg.data;

        // Clip the amplitude to the allowed range [0, 1]
        amplitude = Mathf.Clamp(amplitude, 0f, 1f);

        Debug.Log($"Received amplitude: {amplitude}"); // Add this line

        // Trigger vibration on both controllers
        vibrationManager.TriggerVibration(OVRInput.Controller.RTouch, amplitude, vibrationDuration);
        vibrationManager.TriggerVibration(OVRInput.Controller.LTouch, amplitude, vibrationDuration);
    }
}