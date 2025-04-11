using UnityEngine;
using System;

/// <summary>
/// Handles Oculus Quest 2 controller input and exposes events for buttons, thumbsticks, and triggers.
/// </summary>
public class Quest2ControllerInput : MonoBehaviour
{
    /// <summary>
    /// Event invoked when a thumbstick is moved. Includes label and direction.
    /// </summary>
    public static event Action<string, Vector2> OnThumbstickMoved;

    /// <summary>
    /// Event invoked when a button is pressed.
    /// </summary>
    public static event Action<string> OnButtonPressed;

    /// <summary>
    /// Event invoked when a button is released.
    /// </summary>
    public static event Action<string> OnButtonReleased;

    private Vector2 lastRightThumbstick = Vector2.zero;
    private Vector2 lastLeftThumbstick = Vector2.zero;

    /// <summary>
    /// Threshold to prevent joystick drift from registering as movement.
    /// </summary>
    private float joystickThreshold = 0.2f;

    /// <summary>
    /// Trigger threshold for identifying press action.
    /// </summary>
    public float triggerThreshold = 0.5f;

    /// <summary>
    /// Current analog value of the left trigger.
    /// </summary>
    public static float LeftTriggerValue { get; private set; } = 0f;

    /// <summary>
    /// Current analog value of the right trigger.
    /// </summary>
    public static float RightTriggerValue { get; private set; } = 0f;

    /// <summary>
    /// Checks button and thumbstick input every frame and invokes relevant events.
    /// </summary>
    void Update()
    {
        // ---- Right Controller Buttons ----
        if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.RTouch))
            OnButtonPressed?.Invoke("A Button (Right Controller)");
        if (OVRInput.GetUp(OVRInput.Button.One, OVRInput.Controller.RTouch))
            OnButtonReleased?.Invoke("A Button (Right Controller)");

        if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.RTouch))
            OnButtonPressed?.Invoke("B Button (Right Controller)");
        if (OVRInput.GetUp(OVRInput.Button.Two, OVRInput.Controller.RTouch))
            OnButtonReleased?.Invoke("B Button (Right Controller)");

        // ---- Left Controller Buttons ----
        if (OVRInput.GetDown(OVRInput.Button.One, OVRInput.Controller.LTouch))
            OnButtonPressed?.Invoke("X Button (Left Controller)");
        if (OVRInput.GetUp(OVRInput.Button.One, OVRInput.Controller.LTouch))
            OnButtonReleased?.Invoke("X Button (Left Controller)");

        if (OVRInput.GetDown(OVRInput.Button.Two, OVRInput.Controller.LTouch))
            OnButtonPressed?.Invoke("Y Button (Left Controller)");
        if (OVRInput.GetUp(OVRInput.Button.Two, OVRInput.Controller.LTouch))
            OnButtonReleased?.Invoke("Y Button (Left Controller)");

        // ---- Right Thumbstick ----
        Vector2 rightThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.RTouch);
        Vector2 filteredRightInput = new Vector2(
            Mathf.Abs(rightThumbstick.x) > joystickThreshold ? rightThumbstick.x : 0,
            Mathf.Abs(rightThumbstick.y) > joystickThreshold ? rightThumbstick.y : 0
        );

        if (filteredRightInput != lastRightThumbstick)
        {
            OnThumbstickMoved?.Invoke("Right Thumbstick", filteredRightInput);
            lastRightThumbstick = filteredRightInput;
        }

        // ---- Left Thumbstick ----
        Vector2 leftThumbstick = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick, OVRInput.Controller.LTouch);
        Vector2 filteredLeftInput = new Vector2(
            Mathf.Abs(leftThumbstick.x) > joystickThreshold ? leftThumbstick.x : 0,
            Mathf.Abs(leftThumbstick.y) > joystickThreshold ? leftThumbstick.y : 0
        );

        if (filteredLeftInput != lastLeftThumbstick)
        {
            OnThumbstickMoved?.Invoke("Left Thumbstick", filteredLeftInput);
            lastLeftThumbstick = filteredLeftInput;
        }

        // ---- Triggers ----
        float rightTriggerValue = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.RTouch);
        RightTriggerValue = rightTriggerValue;
        int rightTriggerState = rightTriggerValue >= triggerThreshold ? 1 : 0;
        Debug.Log("Right Trigger State: " + rightTriggerState + " (Analog value: " + rightTriggerValue + ")");

        float leftTriggerValue = OVRInput.Get(OVRInput.Axis1D.PrimaryIndexTrigger, OVRInput.Controller.LTouch);
        LeftTriggerValue = leftTriggerValue;
        int leftTriggerState = leftTriggerValue >= triggerThreshold ? 1 : 0;
        Debug.Log("Left Trigger State: " + leftTriggerState + " (Analog value: " + leftTriggerValue + ")");
    }
}
