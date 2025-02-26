using UnityEngine;
using System.Collections;


public class StereoCameraFeed : MonoBehaviour
{
    [Header("Stereo Camera Settings")]
    [SerializeField] private string stereoCameraName = "ZED-M";
    [SerializeField] private Camera leftEyeCamera;
    [SerializeField] private Camera rightEyeCamera;
    
    [Header("IPD Settings")]
    [SerializeField, Range(0.0f, 0.075f)] private float ipd = 0.063f;
    [SerializeField, Range(0.1f, 2f)] private float displayDistance = 1.28f;
    [SerializeField, Range(-0.5f, 0.5f)] private float yOffset = 0.0f;

    [Header("Quad Scaling")]
    [SerializeField, Range(0.1f, 3f)] private float quadWidthScale = 0.42f;
    [SerializeField, Range(0.1f, 3f)] private float quadHeightScale = 0.9f;
    
    private WebCamTexture webCamTexture;
    private GameObject leftQuad;
    private GameObject rightQuad;

    void Start()
    {
        InitializeStereoCam();
        SetupCameras();
        CreateEyeDisplays();
    }

    private void InitializeStereoCam()
    {
        var devices = WebCamTexture.devices;
        foreach (var device in devices)
        {
            if (device.name.Contains(stereoCameraName))
            {
                webCamTexture = new WebCamTexture(device.name);
                webCamTexture.Play();
                Debug.Log($"Stereo camera initialized: {device.name}");

                // Wait for the camera to start before logging resolution
                StartCoroutine(CheckWebCamResolution());
                break;
            }
        }

        if (webCamTexture == null)
        {
            Debug.LogError($"Stereo camera '{stereoCameraName}' not found.");
            return;
        }
    }

    private IEnumerator CheckWebCamResolution()
    {
        yield return new WaitUntil(() => webCamTexture.width > 100);

        Debug.Log($"WebCamTexture Resolution: {webCamTexture.width}x{webCamTexture.height}");
        Debug.Log($"Aspect Ratio: {(float)webCamTexture.width / webCamTexture.height}");
    }

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

    private void SetupCamera(Camera camera, bool isLeftEye, int layer)
    {
        camera.cullingMask = 1 << layer;
        camera.stereoTargetEye = isLeftEye ? StereoTargetEyeMask.Left : StereoTargetEyeMask.Right;
    }

   private void CreateEyeDisplays()
    {
        int leftEyeLayer = LayerMask.NameToLayer("LeftEyeLayer");
        int rightEyeLayer = LayerMask.NameToLayer("RightEyeLayer");

        leftQuad = CreateEyeQuad(leftEyeCamera, true, leftEyeLayer);
        rightQuad = CreateEyeQuad(rightEyeCamera, false, rightEyeLayer);
    }

    private GameObject CreateEyeQuad(Camera camera, bool isLeftEye, int layer)
    {
        GameObject quad = GameObject.CreatePrimitive(PrimitiveType.Quad);
        quad.name = isLeftEye ? "LeftEyeQuad" : "RightEyeQuad";
        quad.layer = layer;
        quad.transform.SetParent(camera.transform);

        float xOffset = ipd / 2f;
        Vector3 localPos = new Vector3(isLeftEye ? -xOffset : xOffset, 0.0f, displayDistance);
        quad.transform.localPosition = localPos;
        quad.transform.localRotation = Quaternion.identity;

        float aspectRatio = 1.777f;  // Default aspect ratio
        if (webCamTexture != null && webCamTexture.width > 0)
        {
            aspectRatio = (float)webCamTexture.width / webCamTexture.height;
        }

        float quadHeight = displayDistance * 2.0f * quadHeightScale; 
        float quadWidth = quadHeight * aspectRatio * quadWidthScale;

        quad.transform.localScale = new Vector3(quadWidth, quadHeight, 1f);

        Debug.Log($"Quad {quad.name} - Width: {quadWidth}, Height: {quadHeight}, Aspect Ratio: {aspectRatio}");

        Material eyeMaterial = new Material(Shader.Find("Unlit/Texture"));
        eyeMaterial.mainTexture = webCamTexture;
        eyeMaterial.mainTextureScale = new Vector2(0.5f, 1.0f);
        eyeMaterial.mainTextureOffset = isLeftEye ? Vector2.zero : new Vector2(0.5f, 0.0f);
        quad.GetComponent<MeshRenderer>().material = eyeMaterial;

        return quad;
    }

    void Update()
    {
        if (leftEyeCamera && rightEyeCamera)
        {
            UpdateIPDPositions();
        }
    }

    private void UpdateIPDPositions()
    {
        float offset = ipd / 2.0f;

        if (leftQuad && rightQuad)
        {
            leftQuad.transform.localPosition = new Vector3(-offset, yOffset, displayDistance);
            rightQuad.transform.localPosition = new Vector3(offset, 0.0f, displayDistance);
        }
    }


    void OnDisable()
    {
        if (webCamTexture != null && webCamTexture.isPlaying)
        {
            webCamTexture.Stop();
        }
    }
}
