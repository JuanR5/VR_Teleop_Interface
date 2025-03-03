using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using UnityEngine.UI;

/// <summary>
/// Handles stereo image subscription from ROS and applies the images to display quads.
/// </summary>
public class StereoImageSubscriber : MonoBehaviour
{
    // ---- ROS Topic Settings ----
    [Header("ROS Topics")]
    public string leftImageTopic { get; set; }
    public string rightImageTopic { get; set; }

    // ---- Display & Camera Settings ----
    [Header("Display Settings")]
    public Camera leftEyeCamera { get; set; }
    public Camera rightEyeCamera { get; set; }
    public float ipd { get; set; }
    public float displayDistance { get; set; }
    public float yOffset { get; set; }
    public float quadWidthScale { get; set; }
    public float quadHeightScale { get; set; }

    // ---- ROS Connection & Image Data ----
    private ROSConnection ros;
    private GameObject leftQuad, rightQuad;
    private Texture2D leftTexture, rightTexture;

    /// <summary>
    /// Initializes ROS connection, subscribes to topics, and sets up display quads.
    /// </summary>
    private void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Subscribe to image topics
        ros.Subscribe<ImageMsg>(leftImageTopic, msg => {
            Debug.Log($"Left image encoding: {msg.encoding}");
            ReceiveLeftImage(msg);
        });

        ros.Subscribe<ImageMsg>(rightImageTopic, msg => {
            Debug.Log($"Right image encoding: {msg.encoding}");
            ReceiveRightImage(msg);
        });

        SetupCameras();
        CreateEyeDisplays();

        Debug.Log($"Subscribed to {leftImageTopic} and {rightImageTopic}");
    }

    /// <summary>
    /// Configures left and right eye cameras.
    /// </summary>
    private void SetupCameras()
    {
        if (!leftEyeCamera || !rightEyeCamera)
        {
            Debug.LogError("Both cameras must be assigned!");
            return;
        }

        int leftEyeLayer = LayerMask.NameToLayer("LeftEyeLayer");
        int rightEyeLayer = LayerMask.NameToLayer("RightEyeLayer");

        if (leftEyeLayer == -1 || rightEyeLayer == -1)
        {
            Debug.LogError("Layers 'LeftEyeLayer' and 'RightEyeLayer' must be created in Unity.");
            return;
        }

        SetupCamera(leftEyeCamera, true, leftEyeLayer);
        SetupCamera(rightEyeCamera, false, rightEyeLayer);
    }

    /// <summary>
    /// Sets up individual camera properties.
    /// </summary>
    private void SetupCamera(Camera camera, bool isLeftEye, int layer)
    {
        camera.cullingMask = 1 << layer;
        camera.stereoTargetEye = isLeftEye ? StereoTargetEyeMask.Left : StereoTargetEyeMask.Right;
    }

    /// <summary>
    /// Creates quads for displaying left and right eye images.
    /// </summary>
    private void CreateEyeDisplays()
    {
        int leftEyeLayer = LayerMask.NameToLayer("LeftEyeLayer");
        int rightEyeLayer = LayerMask.NameToLayer("RightEyeLayer");

        leftQuad = CreateEyeQuad(leftEyeCamera, true, leftEyeLayer);
        rightQuad = CreateEyeQuad(rightEyeCamera, false, rightEyeLayer);
    }

    /// <summary>
    /// Creates and configures a quad for displaying an eye's image.
    /// </summary>
    private GameObject CreateEyeQuad(Camera camera, bool isLeftEye, int layer)
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = isLeftEye ? "LeftEyeQuad" : "RightEyeQuad";
        quad.layer = layer;
        quad.transform.SetParent(camera.transform);

        float xOffset = ipd / 2f;
        Vector3 localPos = new Vector3(isLeftEye ? -xOffset : xOffset, yOffset, displayDistance);
        quad.transform.localPosition = localPos;
        quad.transform.localRotation = Quaternion.identity;

        quad.transform.localScale = new Vector3(1f, 1f, 1f); // Default scale, updated later

        Material eyeMaterial = new Material(Shader.Find("Unlit/Texture"));
        quad.GetComponent<MeshRenderer>().material = eyeMaterial;

        return quad;
    }

    /// <summary>
    /// Processes the left image from ROS.
    /// </summary>
    private void ReceiveLeftImage(ImageMsg imageMsg)
    {
        ProcessImage(imageMsg, true);
    }

    /// <summary>
    /// Processes the right image from ROS.
    /// </summary>
    private void ReceiveRightImage(ImageMsg imageMsg)
    {
        ProcessImage(imageMsg, false);
    }

    /// <summary>
    /// Converts image data from ROS and applies it to the appropriate quad.
    /// </summary>
    private void ProcessImage(ImageMsg imageMsg, bool isLeft)
    {
        GameObject quad = isLeft ? leftQuad : rightQuad;
        Texture2D tex = isLeft ? leftTexture : rightTexture;

        if (quad == null) return;

        // Initialize texture if needed
        if (tex == null || tex.width != imageMsg.width || tex.height != imageMsg.height)
        {
            tex = new Texture2D((int)imageMsg.width, (int)imageMsg.height, TextureFormat.RGB24, false);
            if (isLeft) leftTexture = tex; else rightTexture = tex;

            float aspectRatio = (float)imageMsg.width / imageMsg.height;
            float quadHeight = displayDistance * 2.0f * quadHeightScale;
            float quadWidth = quadHeight * aspectRatio * quadWidthScale;
            quad.transform.localScale = new Vector3(quadWidth, quadHeight, 1f);
        }

        // Convert image data to texture
        Color32[] colors = new Color32[imageMsg.width * imageMsg.height];
        int stride = (int)imageMsg.width * 3;

        for (int y = 0; y < imageMsg.height; y++)
        {
            for (int x = 0; x < imageMsg.width; x++)
            {
                int sourceIndex = (y * stride) + (x * 3);
                int targetIndex = ((int)imageMsg.height - 1 - y) * (int)imageMsg.width + x;

                colors[targetIndex] = new Color32(
                    imageMsg.data[sourceIndex],     // R
                    imageMsg.data[sourceIndex + 1], // G
                    imageMsg.data[sourceIndex + 2], // B
                    255                             // A
                );
            }
        }

        tex.SetPixels32(colors);
        tex.Apply();

        // Apply texture to quad
        var material = quad.GetComponent<MeshRenderer>().material;
        material.mainTexture = tex;
        material.mainTextureScale = new Vector2(1, 1);
        material.mainTextureOffset = new Vector2(0, 0);
    }

    /// <summary>
    /// Updates the position of the display quads based on the IPD setting.
    /// </summary>
    private void Update()
    {
        UpdateIPDPositions();
    }

    /// <summary>
    /// Adjusts the quad positions dynamically based on interpupillary distance (IPD).
    /// </summary>
    private void UpdateIPDPositions()
    {
        if (leftQuad && rightQuad)
        {
            float offset = ipd / 2.0f;
            leftQuad.transform.localPosition = new Vector3(-offset, yOffset, displayDistance);
            rightQuad.transform.localPosition = new Vector3(offset, -yOffset, displayDistance);
        }
    }

    /// <summary>
    /// Cleans up textures when the object is destroyed.
    /// </summary>
    private void OnDestroy()
    {
        if (leftTexture != null) Destroy(leftTexture);
        if (rightTexture != null) Destroy(rightTexture);
    }
}
