using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.PandaArmMsg;
using RosMessageTypes.Sensor;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class GetCameraData : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 7;

    [SerializeField]
    GameObject m_PandaArm;
    public GameObject PandaArm { get => m_PandaArm; set => m_PandaArm = value; }

    // ROS Connector
    ROSConnection m_Ros;
    Camera m_Camera;

    [SerializeField]
    double m_PublishRateHz = 25f;
    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
    double m_LastPublishTimeSeconds;
    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    Texture2D texture_cam;
    RenderTexture renderTexture_cam;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterRosService<YoloImageRequest, YoloImageResponse>("yolo_image");
        m_Ros.ImplementService<YoloImageRequest, YoloImageResponse>("yolo_image", HandleYoloImageService);
        m_Ros.RegisterPublisher<ImageMsg>("/unity_panda_camera");

        //根据名称查找相机
        GameObject cameraObject = GameObject.Find("Main Camera");
        m_Camera = cameraObject.GetComponent<Camera>();
        texture_cam = new Texture2D(m_Camera.pixelWidth, m_Camera.pixelHeight, TextureFormat.RGB24, false);
        renderTexture_cam = new RenderTexture(m_Camera.pixelWidth, m_Camera.pixelHeight, 24);

    }

    // Update is called once per frame
    void Update()
    {
        if (ShouldPublishMessage)
        {
            // 获取相机图像
            m_Camera.targetTexture = renderTexture_cam;
            m_Camera.Render();
            RenderTexture.active = renderTexture_cam;
            texture_cam.ReadPixels(new Rect(0, 0, renderTexture_cam.width, renderTexture_cam.height), 0, 0);
            texture_cam.Apply();
            m_Camera.targetTexture = null;
            RenderTexture.active = null;
            byte[] imageData = texture_cam.GetRawTextureData();

            var msgImage = new ImageMsg
            {
                height = (uint)texture_cam.height,
                width = (uint)texture_cam.width,
                encoding = "rgb8",
                is_bigendian = 0,
                step = (uint)(texture_cam.width * 3),
                data = imageData
            };

            m_Ros.Publish("/unity_panda_camera", msgImage);
            m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
        }

    }

    /// <summary>
    /// 处理 yolo_image 服务请求
    /// </summary>
    /// <param name="request">服务请求</param>
    /// <returns>服务响应</returns>
    private YoloImageResponse HandleYoloImageService(YoloImageRequest request)
    {
        Debug.Log($"Received YoloImageRequest with num_image: {request.num_image}");

        // 获取相机图像
        m_Camera.targetTexture = renderTexture_cam;
        m_Camera.Render();
        RenderTexture.active = renderTexture_cam;
        texture_cam.ReadPixels(new Rect(0, 0, renderTexture_cam.width, renderTexture_cam.height), 0, 0);
        texture_cam.Apply();
        m_Camera.targetTexture = null;
        RenderTexture.active = null;
        byte[] imageData = texture_cam.GetRawTextureData();

        var msgImage = new ImageMsg
        {
            height = (uint)texture_cam.height,
            width = (uint)texture_cam.width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(texture_cam.width * 3),
            data = imageData
        };

        // 模拟处理逻辑，例如生成检测结果
        var images = new ImageMsg[]
        {
            msgImage
        };

        // 创建响应
        var response = new YoloImageResponse
        {
            success = true,
            images = images
        };

        Debug.Log($"Sending YoloImageResponse with {response.images.Length} images.");
        return response;
    }
}
