using System;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using RosMessageTypes.Tf2;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class ROS2TalkerDemo : MonoBehaviour
{
    const string k_DemoTalkerTopic = "/demo_talker";
    const string k_ImageTopic = "/image_talker";

    [SerializeField]
    double m_PublishRateHz = 20f;
    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
    double m_LastPublishTimeSeconds;
    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;

    ROSConnection m_ROS;


    private int i;

    // Start is called before the first frame update
    void Start()
    {
        m_ROS = ROSConnection.GetOrCreateInstance();
        if (m_ROS == null)
        {
            Debug.LogError("ROSConnection instance is null!");
        }
        else
        {
            m_ROS.RegisterPublisher<StringMsg>(k_DemoTalkerTopic);
            m_ROS.RegisterPublisher<ImageMsg>(k_ImageTopic);
        }

        m_LastPublishTimeSeconds = Clock.time + PublishPeriodSeconds;
    }

    // Update is called once per frame
    void Update()
    {
        if (ShouldPublishMessage)
        {
            if (m_ROS == null)
            {
                Debug.LogError("m_ROS or m_Camera is null!");
                return;
            }

            i++;
            if (i > 1000000)
            {
                i = 0;
            }
            var msgDemoTalker = new StringMsg
            {
                data = "Hello, ROS2! send: " + i
            };
            m_ROS.Publish(k_DemoTalkerTopic, msgDemoTalker);

            Debug.Log("Published message: " + msgDemoTalker.data);
            m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
        }
    }
}

