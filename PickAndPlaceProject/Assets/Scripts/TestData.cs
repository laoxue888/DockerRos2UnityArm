using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;
using UnityEngine.UIElements;
using RosMessageTypes.Geometry;

public class TestData : MonoBehaviour
{
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.1f;
    // Start is called before the first frame update

    [SerializeField]
    GameObject m_Target;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        PointMsg position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>();
        Debug.Log("Position: " + position.x + " " + position.y + " " + position.z);
    }
}
