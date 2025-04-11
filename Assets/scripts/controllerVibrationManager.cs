using UnityEngine;

/// <summary>
/// Manages controller vibration using the OVRInput API.
/// Provides functionality to trigger and stop vibrations on supported VR controllers.
/// </summary>
public class ControllerVibrationManager : MonoBehaviour
{
    /// <summary>
    /// Triggers vibration on the specified controller.
    /// </summary>
    /// <param name="controller">The controller to vibrate (e.g., RTouch or LTouch).</param>
    /// <param name="amplitude">Intensity of the vibration [0, 1].</param>
    /// <param name="duration">Duration of the vibration in seconds.</param>
    public void TriggerVibration(OVRInput.Controller controller, float amplitude, float duration)
    {
        Debug.Log($"TriggerVibration called: controller={controller}, amplitude={amplitude}, duration={duration}");
        OVRInput.SetControllerVibration(duration, amplitude, controller);
        Invoke("StopVibration", duration);
    }

    /// <summary>
    /// Stops all controller vibrations.
    /// </summary>
    private void StopVibration()
    {
        OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.RTouch);
        OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.LTouch);
    }
}
