using UnityEngine;
using System;

public class Quest2ControllerInput : MonoBehaviour
{
    public static event Action<string, Vector2> OnThumbstickMoved;
    public static event Action<string> OnButtonPressed;
    public static event Action<string> OnButtonReleased;

    private Vector2 lastRightThumbstick = Vector2.zero;
    private Vector2 lastLeftThumbstick = Vector2.zero;
    private float joystickThreshold = 0.2f; // Higher to ignore drift

    // Trigger threshold for detecting press (0-1 range)
    public float triggerThreshold = 0.5f;

    // Static properties to store current trigger analog values
    public static float LeftTriggerValue { get; private set; } = 0f;
    public static float RightTriggerValue { get; private set; } = 0f;

    void Update()
    {
        // ---- Right Controller Button Presses ----
        if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch))
            OnButtonPressed?.Invoke("A Button (Right Controller)");
        if (OVRInput.GetUp(OVRInput.Button.One, OVRInput.Controller.RTouch))
            OnButtonReleased?.Invoke("A Button (Right Controller)");

        if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.RTouch))
            OnButtonPressed?.Invoke("B Button (Right Controller)");
        if (OVRInput.GetUp(OVRInput.Button.Two, OVRInput.Controller.RTouch))
            OnButtonReleased?.Invoke("B Button (Right Controller)");

        // ---- Left Controller Button Presses ----
        if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch))
            OnButtonPressed?.Invoke("X Button (Left Controller)");
        if (OVRInput.GetUp(OVRInput.Button.One, OVRInput.Controller.LTouch))
            OnButtonReleased?.Invoke("X Button (Left Controller)");

        if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.LTouch))
            OnButtonPressed?.Invoke("Y Button (Left Controller)");
        if (OVRInput.GetUp(OVRInput.Button.Two, OVRInput.Controller.LTouch))
            OnButtonReleased?.Invoke("Y Button (Left Controller)");

        // ---- Right Thumbstick Movement ----
        Vector2 rightThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
        
        // Apply threshold to prevent drift (Right)
        Vector2 filteredRightInput = new Vector2(
            Mathf.Abs(rightThumbstick.x) > joystickThreshold ? rightThumbstick.x : 0,
            Mathf.Abs(rightThumbstick.y) > joystickThreshold ? rightThumbstick.y : 0
        );

        // Fire event only if significant movement detected (Right)
        if (filteredRightInput != lastRightThumbstick)
        {
            OnThumbstickMoved?.Invoke("Right Thumbstick", filteredRightInput);
            lastRightThumbstick = filteredRightInput;
        }

        // ---- Left Thumbstick Movement ----
        Vector2 leftThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
        
        // Apply threshold to prevent drift (Left)
        Vector2 filteredLeftInput = new Vector2(
            Mathf.Abs(leftThumbstick.x) > joystickThreshold ? leftThumbstick.x : 0,
            Mathf.Abs(leftThumbstick.y) > joystickThreshold ? leftThumbstick.y : 0
        );

        // Fire event only if significant movement detected (Left)
        if (filteredLeftInput != lastLeftThumbstick)
        {
            OnThumbstickMoved?.Invoke("Left Thumbstick", filteredLeftInput);
            lastLeftThumbstick = filteredLeftInput;
        }

        // ---- Trigger Button States ----
        // Right Trigger: Get analog value, update static property, and log state
        float rightTriggerValue = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        RightTriggerValue = rightTriggerValue;
        int rightTriggerState = rightTriggerValue >= triggerThreshold ? 1 : 0;
        Debug.Log("Right Trigger State: " + rightTriggerState + " (Analog value: " + rightTriggerValue + ")");

        // Left Trigger: Get analog value, update static property, and log state
        float leftTriggerValue = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
        LeftTriggerValue = leftTriggerValue;
        int leftTriggerState = leftTriggerValue >= triggerThreshold ? 1 : 0;
        Debug.Log("Left Trigger State: " + leftTriggerState + " (Analog value: " + leftTriggerValue + ")");
    }
}
