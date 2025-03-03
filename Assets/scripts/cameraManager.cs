using UnityEngine;

/// <summary>
/// Defines the type of camera input: USB or ROS.
/// </summary>
public enum CameraInputType
{
    USB,
    ROS
}

/// <summary>
/// Manages stereo camera input, settings, and switching between USB and ROS sources.
/// </summary>
public class StereoCameraManager : MonoBehaviour
{
    // ---- Input Selection ----
    [Header("Input Selection")]
    [SerializeField] private CameraInputType inputType = CameraInputType.USB;

    // ---- Camera References ----
    [Header("Camera References")]
    [SerializeField] private Camera leftEyeCamera;
    [SerializeField] private Camera rightEyeCamera;

    // ---- Common Stereo Camera Settings ----
    [Header("Common Settings")]
    [SerializeField, Range(0.0f, 0.075f)] private float ipd = 0.063f;                  // Interpupillary distance
    [SerializeField, Range(0.1f, 2f)] private float displayDistance = 1.28f;          // Distance of the display
    [SerializeField, Range(-0.5f, 0.5f)] private float yOffset = 0.0f;                // Vertical offset
    [SerializeField, Range(0.1f, 3f)] private float quadWidthScale = 0.42f;           // Scaling factor for quad width
    [SerializeField, Range(0.1f, 3f)] private float quadHeightScale = 0.9f;           // Scaling factor for quad height

    // ---- USB Camera Settings ----
    [Header("USB Camera Settings")]
    [SerializeField] private string stereoCameraName = "ZED-M";

    // ---- ROS Camera Settings ----
    [Header("ROS Settings")]
    [SerializeField] private string leftImageTopic = "/zed2_unity/left_image";
    [SerializeField] private string rightImageTopic = "/zed2_unity/right_image";

    // ---- Camera Components ----
    private StereoCameraFeed usbCamera;
    private StereoImageSubscriber rosCamera;
    private MonoBehaviour currentActiveCamera;

    /// <summary>
    /// Initializes the selected camera input on start.
    /// </summary>
    private void Start()
    {
        InitializeSelectedCamera();
    }

    /// <summary>
    /// Called in the editor when values are changed. Updates the active camera settings at runtime.
    /// </summary>
    private void OnValidate()
    {
        if (Application.isPlaying)
        {
            UpdateActiveCamera();
        }
    }

    /// <summary>
    /// Updates the settings of the currently active camera.
    /// </summary>
    private void UpdateActiveCamera()
    {
        if (usbCamera != null && currentActiveCamera == usbCamera)
        {
            UpdateUSBCameraSettings();
        }
        else if (rosCamera != null && currentActiveCamera == rosCamera)
        {
            UpdateROSCameraSettings();
        }
    }

    /// <summary>
    /// Initializes the selected camera type by adding the appropriate component.
    /// </summary>
    private void InitializeSelectedCamera()
    {
        // Cleanup any existing camera components
        if (usbCamera != null) Destroy(usbCamera);
        if (rosCamera != null) Destroy(rosCamera);

        // Add and configure the selected camera type
        switch (inputType)
        {
            case CameraInputType.USB:
                usbCamera = gameObject.AddComponent<StereoCameraFeed>();
                SetupUSBCamera();
                currentActiveCamera = usbCamera;
                break;

            case CameraInputType.ROS:
                rosCamera = gameObject.AddComponent<StereoImageSubscriber>();
                SetupROSCamera();
                currentActiveCamera = rosCamera;
                break;
        }
    }

    /// <summary>
    /// Sets up the USB camera component.
    /// </summary>
    private void SetupUSBCamera()
    {
        usbCamera.stereoCameraName = stereoCameraName;
        UpdateUSBCameraSettings();
    }

    /// <summary>
    /// Sets up the ROS camera component.
    /// </summary>
    private void SetupROSCamera()
    {
        rosCamera.leftImageTopic = leftImageTopic;
        rosCamera.rightImageTopic = rightImageTopic;
        UpdateROSCameraSettings();
    }

    /// <summary>
    /// Updates settings for the USB camera.
    /// </summary>
    private void UpdateUSBCameraSettings()
    {
        usbCamera.leftEyeCamera = leftEyeCamera;
        usbCamera.rightEyeCamera = rightEyeCamera;
        usbCamera.ipd = ipd;
        usbCamera.displayDistance = displayDistance;
        usbCamera.yOffset = yOffset;
        usbCamera.quadWidthScale = quadWidthScale;
        usbCamera.quadHeightScale = quadHeightScale;
    }

    /// <summary>
    /// Updates settings for the ROS camera.
    /// </summary>
    private void UpdateROSCameraSettings()
    {
        rosCamera.leftEyeCamera = leftEyeCamera;
        rosCamera.rightEyeCamera = rightEyeCamera;
        rosCamera.ipd = ipd;
        rosCamera.displayDistance = displayDistance;
        rosCamera.yOffset = yOffset;
        rosCamera.quadWidthScale = quadWidthScale;
        rosCamera.quadHeightScale = quadHeightScale;
    }

    /// <summary>
    /// Switches the camera input type and reinitializes the camera.
    /// </summary>
    public void SwitchCameraType(CameraInputType newType)
    {
        if (inputType != newType)
        {
            inputType = newType;
            InitializeSelectedCamera();
        }
    }
}
