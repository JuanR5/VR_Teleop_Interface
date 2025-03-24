using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // For Float32MultiArray message type
using System; // For Math.Round

public class RumbleSubscriber : MonoBehaviour
{
    private ROSConnection ros;
    private ControllerVibrationManager vibrationManager;
    
    // Define the topic name
    private string rumbleOutputTopic = "/rumble_output";
    
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
        
        // Subscribe to the rumble output topic, using Float32MultiArray message type
        ros.Subscribe<Float32MultiArrayMsg>(rumbleOutputTopic, HandleRumbleMessage);
        Debug.Log($"Subscribed to {rumbleOutputTopic}");
    }
    
    private void HandleRumbleMessage(Float32MultiArrayMsg msg)
    {
        // Check if the array has at least 2 elements (amplitude and duration)
        if (msg.data.Length >= 2)
        {
            // Get the amplitude and duration from the message
            float amplitude = msg.data[0];
            float duration = msg.data[1];
            
            // Filter the input: Clamp amplitude and ensure positive duration
            amplitude = Mathf.Clamp(amplitude, 0f, 1f);
            duration = Mathf.Max(0f, duration);
            
            // Round both values to 3 decimal places
            amplitude = (float)Math.Round(amplitude, 3);
            duration = (float)Math.Round(duration, 3);
            
            // Floor values below 0.1 to 0
            if (amplitude < 0.1f)
            {
                amplitude = 0f;
            }
            if (duration < 0.1f)
            {
                duration = 0f;
            }
            
            Debug.Log($"Received amplitude: {amplitude}, duration: {duration}");
            
            // Trigger vibration only if both amplitude and duration are above zero
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
