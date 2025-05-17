using System;
using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.PandaArmMsg;
using RosMessageTypes.Trajectory;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using RosMessageTypes.Std;
using RosMessageTypes.Sensor;
using static UnityEditor.PlayerSettings;
using Unity.Robotics.Core;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 7;

    [SerializeField]
    GameObject m_PandaArm;
    public GameObject PandaArm { get => m_PandaArm; set => m_PandaArm = value; }

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        //订阅话题
        m_Ros.Subscribe<JointTrajectoryMsg>("/panda_joint_trajectory", ReceiveJointTrajectoryMessage); //尽量使用ros2定义的消息吧JointTrajectoryMsg

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        string[] LinkNames ={ "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", "/panda_link5", "/panda_link6", "/panda_link7"};

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_PandaArm.transform.Find(linkName).GetComponent<ArticulationBody>();
        }
    }

    private void Update()
    {

    }

    void ReceiveJointTrajectoryMessage(JointTrajectoryMsg jointTrajectoryMsg)
    {
        //Debug.Log("Trajectory returned.");
        //执行moveit发过来的轨迹
        foreach (var t in jointTrajectoryMsg.points)
        {
            var jointPositions = t.positions;
            var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

            // Set the joint values for every joint
            for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                joint1XDrive.target = result[joint];
                m_JointArticulationBodies[joint].xDrive = joint1XDrive;
            }
        }
        Debug.Log("Trajectory returned.");

    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    }
}
