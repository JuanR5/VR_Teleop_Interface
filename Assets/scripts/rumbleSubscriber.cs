using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using System;

/// <summary>
/// Subscribes to a ROS topic and triggers haptic feedback on VR controllers.
/// </summary>
public class RumbleSubscriber : MonoBehaviour
{
    /// <summary>
    /// Reference to the ROSConnection singleton.
    /// </summary>
    private ROSConnection ros;

    /// <summary>
    /// Reference to the vibration manager handling controller rumble.
    /// </summary>
    private ControllerVibrationManager vibrationManager;

    /// <summary>
    /// Name of the ROS topic to subscribe to for rumble messages.
    /// </summary>
    private string rumbleOutputTopic = "/rumble_output";

    /// <summary>
    /// Initializes the ROS connection and subscribes to the rumble topic.
    /// </summary>
    private void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        vibrationManager = FindObjectOfType<ControllerVibrationManager>();

        if (vibrationManager == null)
        {
            Debug.LogError("ControllerVibrationManager not found in the scene!");
            enabled = false;
            return;
        }

        ros.Subscribe<Float32MultiArrayMsg>(rumbleOutputTopic, HandleRumbleMessage);
        Debug.Log($"Subscribed to {rumbleOutputTopic}");
    }

    /// <summary>
    /// Callback invoked when a Float32MultiArrayMsg is received.
    /// Triggers vibration based on the [amplitude, duration] values.
    /// </summary>
    /// <param name="msg">Incoming ROS message containing amplitude and duration.</param>
    private void HandleRumbleMessage(Float32MultiArrayMsg msg)
    {
        if (msg.data.Length >= 2)
        {
            float amplitude = msg.data[0];
            float duration = msg.data[1];

            amplitude = Mathf.Clamp(amplitude, 0f, 1f);
            duration = Mathf.Max(0f, duration);

            amplitude = (float)Math.Round(amplitude, 3);
            duration = (float)Math.Round(duration, 3);

            if (amplitude < 0.3f) amplitude = 0f;
            if (duration < 0.1f) duration = 0f;

            Debug.Log($"Received amplitude: {amplitude}, duration: {duration}");

            if (amplitude > 0f && duration > 0f)
            {
                vibrationManager.TriggerVibration(OVRInput.Controller.RTouch, amplitude, duration);
                vibrationManager.TriggerVibration(OVRInput.Controller.LTouch, amplitude, duration);
            }
            else
            {
                Debug.LogWarning("Vibration parameters are too low, ignoring rumble command.");
            }
        }
        else
        {
            Debug.LogWarning("Received rumble message with insufficient data. Expected [amplitude, duration] array.");
        }
    }
}