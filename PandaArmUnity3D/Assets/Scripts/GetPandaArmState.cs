using System.Collections;
using System.Collections.Generic;
using RosMessageTypes.Std;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class GetPandaArmState : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 7;

    [SerializeField]
    GameObject m_PandaArm;
    public GameObject PandaArm { get => m_PandaArm; set => m_PandaArm = value; }

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    // ROS Connector
    ROSConnection m_Ros;

    [SerializeField]
    double m_PublishRateHz = 25f;
    double PublishPeriodSeconds => 1.0f / m_PublishRateHz;
    double m_LastPublishTimeSeconds;
    bool ShouldPublishMessage => Clock.NowTimeInSeconds > m_LastPublishTimeSeconds + PublishPeriodSeconds;
    // Start is called before the first frame update
    void Start()
    {
        m_Ros = ROSConnection.GetOrCreateInstance();

        // 注册话题
        m_Ros.RegisterPublisher<Float64MultiArrayMsg>("/unity_panda_joint_angles");

        string[] LinkNames = { "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", "/panda_link5", "/panda_link6", "/panda_link7" };
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_PandaArm.transform.Find(linkName).GetComponent<ArticulationBody>();
        }

    }

    // Update is called once per frame
    void Update()
    {

        if (ShouldPublishMessage)
        {
            //发布当前的机械臂角度
            m_Ros.Publish("/unity_panda_joint_angles", CurrentJointConfig());
            m_LastPublishTimeSeconds = Clock.FrameStartTimeInSeconds;
        }        
    }

    public Float64MultiArrayMsg CurrentJointConfig()
    {
        //var joints = new NiryoMoveitJointsMsg();
        Float64MultiArrayMsg jointAngles = new Float64MultiArrayMsg();
        //初始化数组
        jointAngles.data = new double[k_NumRobotJoints];

        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            jointAngles.data[i] = m_JointArticulationBodies[i].jointPosition[0];
        }

        return jointAngles;
    }
}


