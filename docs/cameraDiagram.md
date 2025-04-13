´´´mermaid
classDiagram
direction TB
class StereoCameraManager {
    - inputType: CameraInputType
    - leftEyeCamera: Camera
    - rightEyeCamera: Camera
    - ipd: float
    - displayDistance: float
    - yOffset: float
    - quadWidthScale: float
    - quadHeightScale: float
    - stereoCameraName: string
    - leftImageTopic: string
    - rightImageTopic: string
    - usbCamera: StereoCameraFeed
    - rosCamera: StereoImageSubscriber
    - currentActiveCamera: MonoBehaviour
    + SwitchCameraType(newType)
}

class StereoCameraFeed {
    - stereoCameraName: string
    - webCamTexture: WebCamTexture
    - leftQuad: GameObject
    - rightQuad: GameObject
    + TryFindWebCam(): bool
    + CreateEyeDisplays()
    + CreateEyeQuad()
    + ApplyWebCamTextureToQuad()
    + UpdateIPDPositions()
    + Update()
    + OnDisable()
}

class StereoImageSubscriber {
    - leftImageTopic: string
    - rightImageTopic: string
    - leftQuad: GameObject
    - rightQuad: GameObject
    - leftReceiver: ROSImageReceiver
    - rightReceiver: ROSImageReceiver
    + Start()
    + Update()
    + OnDestroy()
}
class AbstractStereoSource {
    <<abstract>>
    + leftEyeCamera: Camera
    + rightEyeCamera: Camera
    + ipd: float
    + displayDistance: float
    + yOffset: float
    + quadWidthScale: float
    + quadHeightScale: float
    + CreateEyeDisplays()
    + CreateEyeQuad()
    + ApplyTextureToQuad()
    + UpdateIPDPositions()
}

class MeshGenerator {
    <<utility>>
    + CreateQuadMesh(): Mesh
}

class SSHRunner {
    - username: string
    - password: string
    - host: string
    - command: string
    - sshClient: SshClient
    - cancellationTokenSource: CancellationTokenSource
    + Start()
    - RunSSHCommand()
    - RunCommandAsync(command: string, token)
    + OnApplicationQuit()
    + OnDestroy()
    - CancelSSHCommand()
}
StereoCameraFeed --|> AbstractStereoSource
StereoImageSubscriber --|> AbstractStereoSource
StereoCameraManager --> StereoCameraFeed : manages
StereoCameraManager --> StereoImageSubscriber : or manages
MeshGenerator--> Stereo Vision : uses left/right
SSHRunner --> SSH_AORUS_ZED : Run a Docker container
AbstractStereoSource --> MeshGenerator : uses to CreateEyeQuads
´´´