using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Displays a live camera feed (USB or integrated webcam) to a Unity UI RawImage.
/// Optionally crops to show only the left or right half of the camera feed.
/// </summary>
public class CameraInput : MonoBehaviour
{
    /// <summary>
    /// UI element used to display the camera feed.
    /// </summary>
    public RawImage rawImage;

    /// <summary>
    /// Flag to control whether the left or right half of the camera feed is shown.
    /// True = Left side, False = Right side.
    /// </summary>
    public bool showLeftCamera = true;

    private WebCamTexture webcamTexture;

    /// <summary>
    /// Initializes the webcam feed and sets it to the UI.
    /// </summary>
    void Start()
    {
        WebCamDevice[] devices = WebCamTexture.devices;

        if (devices.Length > 0)
        {
            int selectedCameraIndex = 0;
            if (devices.Length > 1)
            {
                selectedCameraIndex = 1; // Prefer external USB camera if available
            }

            webcamTexture = new WebCamTexture(devices[selectedCameraIndex].name);
            rawImage.texture = webcamTexture;
            webcamTexture.Play();

            // Adjust UV rect to show left or right half of the feed
            rawImage.uvRect = showLeftCamera ?
                new Rect(0, 0, 0.5f, 1) :
                new Rect(0.5f, 0, 0.5f, 1);
        }
        else
        {
            Debug.LogError("No webcam found!");
        }
    }

    /// <summary>
    /// Stops the webcam feed when the application exits.
    /// </summary>
    void OnApplicationQuit()
    {
        if (webcamTexture != null && webcamTexture.isPlaying)
        {
            webcamTexture.Stop();
        }
    }
}
