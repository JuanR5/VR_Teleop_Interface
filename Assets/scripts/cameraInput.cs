using UnityEngine;
using UnityEngine.UI;

public class CameraInput : MonoBehaviour
{
    public RawImage rawImage;    // UI element to display the camera feed
    private WebCamTexture webcamTexture;  // To hold the camera feed

    public bool showLeftCamera = true;  // Set this flag to true to show the left camera feed, false for the right camera

    void Start()
    {
        // Get the available webcams
        WebCamDevice[] devices = WebCamTexture.devices;

        if (devices.Length > 0)
        {
            // Choose the USB camera (if connected) or the laptop camera (index 0)
            int selectedCameraIndex = 0; // Default to the laptop camera

            // If more than one camera is available, select the second one (USB camera)
            if (devices.Length > 1)
            {
                selectedCameraIndex = 1; // USB camera (index 1)
            }

            // Create a WebCamTexture from the selected camera
            webcamTexture = new WebCamTexture(devices[selectedCameraIndex].name);
            rawImage.texture = webcamTexture;

            // Start the camera feed
            webcamTexture.Play();

            // Optionally split the image to show only the left or right side
            if (showLeftCamera)
            {
                rawImage.uvRect = new Rect(0, 0, 0.5f, 1);  // Show the left side
            }
            else
            {
                rawImage.uvRect = new Rect(0.5f, 0, 0.5f, 1);  // Show the right side
            }
        }
        else
        {
            Debug.LogError("No webcam found!");
        }
    }

    void OnApplicationQuit()
    {
        // Stop the webcam when the application quits
        if (webcamTexture != null && webcamTexture.isPlaying)
        {
            webcamTexture.Stop();
        }
    }
}
