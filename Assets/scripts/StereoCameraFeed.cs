using UnityEngine;
using System.Collections;

public class StereoCameraFeed : MonoBehaviour
{
    [Header("Stereo Camera Settings")]
    public string stereoCameraName { get; set; }
    public Camera leftEyeCamera { get; set; }
    public Camera rightEyeCamera { get; set; }
    public float ipd { get; set; }
    public float displayDistance { get; set; }
    public float yOffset { get; set; }
    public float quadWidthScale { get; set; }
    public float quadHeightScale { get; set; }

    private WebCamTexture webCamTexture;
    private GameObject leftQuad, rightQuad;

    private void Start()
    {
        if (TryFindWebCam())
        {
            SetupCameras();
            CreateEyeDisplays();
        }
        else
        {
            Debug.LogError($"Stereo camera '{stereoCameraName}' not found.");
        }
    }

    /// <summary>
    /// Attempts to find and initialize the stereo webcam.
    /// </summary>
    private bool TryFindWebCam()
    {
        foreach (var device in WebCamTexture.devices)
        {
            if (device.name.Contains(stereoCameraName))
            {
                webCamTexture = new WebCamTexture(device.name);
                webCamTexture.Play();
                Debug.Log($"Stereo camera initialized: {device.name}");

                StartCoroutine(CheckWebCamResolution());
                return true;
            }
        }
        return false;
    }

    /// <summary>
    /// Waits until the webcam provides a valid resolution.
    /// </summary>
    private IEnumerator CheckWebCamResolution()
    {
        yield return new WaitUntil(() => webCamTexture.width > 100);
        Debug.Log($"WebCamTexture Resolution: {webCamTexture.width}x{webCamTexture.height}");
        Debug.Log($"Aspect Ratio: {(float)webCamTexture.width / webCamTexture.height}");
    }

    /// <summary>
    /// Configures the left and right eye cameras.
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

        ConfigureCamera(leftEyeCamera, true, leftEyeLayer);
        ConfigureCamera(rightEyeCamera, false, rightEyeLayer);
    }

    /// <summary>
    /// Configures an individual camera.
    /// </summary>
    private void ConfigureCamera(Camera camera, bool isLeftEye, int layer)
    {
        camera.cullingMask = 1 << layer;
        camera.stereoTargetEye = isLeftEye ? StereoTargetEyeMask.Left : StereoTargetEyeMask.Right;
    }

    /// <summary>
    /// Creates display quads for left and right eye views.
    /// </summary>
    private void CreateEyeDisplays()
    {
        int leftEyeLayer = LayerMask.NameToLayer("LeftEyeLayer");
        int rightEyeLayer = LayerMask.NameToLayer("RightEyeLayer");

        leftQuad = CreateEyeQuad(leftEyeCamera, true, leftEyeLayer);
        rightQuad = CreateEyeQuad(rightEyeCamera, false, rightEyeLayer);
    }

    /// <summary>
    /// Creates a quad for displaying an eye's webcam feed.
    /// </summary>
    private GameObject CreateEyeQuad(Camera camera, bool isLeftEye, int layer)
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = isLeftEye ? "LeftEyeQuad" : "RightEyeQuad";
        quad.layer = layer;
        quad.transform.SetParent(camera.transform);

        float xOffset = ipd / 2f;
        quad.transform.localPosition = new Vector3(isLeftEye ? -xOffset : xOffset, yOffset, displayDistance);
        quad.transform.localRotation = Quaternion.identity;

        // Calculate aspect ratio
        float aspectRatio = webCamTexture != null && webCamTexture.width > 0
            ? (float)webCamTexture.width / webCamTexture.height
            : 1.777f;  // Default aspect ratio

        float quadHeight = displayDistance * 2.0f * quadHeightScale;
        float quadWidth = quadHeight * aspectRatio * quadWidthScale;
        quad.transform.localScale = new Vector3(quadWidth, quadHeight, 1f);

        Debug.Log($"Quad {quad.name} - Width: {quadWidth}, Height: {quadHeight}, Aspect Ratio: {aspectRatio}");

        ApplyWebCamTextureToQuad(quad, isLeftEye);

        return quad;
    }

    /// <summary>
    /// Applies the webcam texture to a quad.
    /// </summary>
    private void ApplyWebCamTextureToQuad(GameObject quad, bool isLeftEye)
    {
        if (webCamTexture == null) return;

        Material eyeMaterial = new Material(Shader.Find("Unlit/Texture"));
        eyeMaterial.mainTexture = webCamTexture;
        eyeMaterial.mainTextureScale = new Vector2(0.5f, 1.0f);
        eyeMaterial.mainTextureOffset = isLeftEye ? Vector2.zero : new Vector2(0.5f, 0.0f);
        quad.GetComponent<MeshRenderer>().material = eyeMaterial;
    }

    /// <summary>
    /// Updates the interpupillary distance (IPD) dynamically.
    /// </summary>
    private void Update()
    {
        UpdateIPDPositions();
    }

    /// <summary>
    /// Adjusts the position of the display quads based on the IPD setting.
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
    /// Stops the webcam feed when the object is disabled.
    /// </summary>
    private void OnDisable()
    {
        if (webCamTexture != null && webCamTexture.isPlaying)
        {
            webCamTexture.Stop();
        }
    }
}
