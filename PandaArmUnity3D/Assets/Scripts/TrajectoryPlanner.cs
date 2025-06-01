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
using System.Threading.Tasks;
using RosMessageTypes.NiryoMoveit;
using static UnityEditor.ShaderData;

public class TrajectoryPlanner : MonoBehaviour
{
    // Hardcoded variables
    const int k_NumRobotJoints = 7;
    const float k_JointAssignmentWait = 0.1f;
    const float k_PoseAssignmentWait = 0.5f;

    [SerializeField]
    GameObject m_PandaArm;
    public GameObject PandaArm { get => m_PandaArm; set => m_PandaArm = value; }

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;
    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    // ROS Connector
    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        //订阅话题
        //m_Ros.Subscribe<JointTrajectoryMsg>("/panda_joint_trajectory", ReceiveJointTrajectoryMessage); //尽量使用ros2定义的消息吧JointTrajectoryMsg
        m_Ros.RegisterRosService<ControlUnityArmGripperRequest, ControlUnityArmGripperResponse>("control_unity_arm_gripper");
        m_Ros.ImplementService<ControlUnityArmGripperRequest, ControlUnityArmGripperResponse>("control_unity_arm_gripper", HandleControlGripperService);

        m_Ros.RegisterRosService<ControlUnityArmRequest, ControlUnityArmResponse>("control_unity_arm");
        m_Ros.ImplementService<ControlUnityArmRequest, ControlUnityArmResponse>("control_unity_arm", HandleControlArmService);

        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        string[] LinkNames ={ "world/panda_link0/panda_link1", "/panda_link2", "/panda_link3", "/panda_link4", "/panda_link5", "/panda_link6", "/panda_link7"};

        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            linkName += LinkNames[i];
            m_JointArticulationBodies[i] = m_PandaArm.transform.Find(linkName).GetComponent<ArticulationBody>();
        }
        // Find left and right fingers
        var rightGripper = linkName + "/panda_link8/panda_hand/panda_rightfinger";
        var leftGripper = linkName + "/panda_link8/panda_hand/panda_leftfinger";
        m_LeftGripper = m_PandaArm.transform.Find(leftGripper).GetComponent<ArticulationBody>();
        m_RightGripper = m_PandaArm.transform.Find(rightGripper).GetComponent<ArticulationBody>();

        //OpenGripper();
    }

    private ControlUnityArmResponse HandleControlArmService(ControlUnityArmRequest request)
    {
        var joint_trajectory = request.joint_trajectory;
        StartCoroutine(ExecuteTrajectories(joint_trajectory));

        // 创建响应
        var response = new ControlUnityArmResponse
        {
            success = true,
        };
        Debug.Log("Receive arm message.");

        return response;
    }

    IEnumerator ExecuteTrajectories(JointTrajectoryMsg joint_trajectory)
    {

        // For every robot pose in trajectory plan
        foreach (var t in joint_trajectory.points)
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

            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(k_JointAssignmentWait);
        }

    }

    private ControlUnityArmGripperResponse HandleControlGripperService(ControlUnityArmGripperRequest request)
    {

        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = (float)request.gripper_position;
        rightDrive.target = (float)request.gripper_position;

        Debug.Log($"Gripper position: {request.gripper_position}");

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;

        // 创建响应
        var response = new ControlUnityArmGripperResponse
        {
            success = true,
        };
        Debug.Log("Receive arm gripper message.");

        return response;
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
        Debug.Log("Receive joint trajectory message.");

    }
    private void Update()
    {

    }

}
