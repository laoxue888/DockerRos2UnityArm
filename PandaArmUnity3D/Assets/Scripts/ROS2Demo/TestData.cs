using System.Collections;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;
using UnityEngine.UIElements;
using RosMessageTypes.Geometry;

public class TestData : MonoBehaviour
{
    [SerializeField]
    GameObject m_Target;

    [SerializeField]
    GameObject m_ReferenceModel;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

        // 获取世界坐标
        Vector3 worldPosition = m_Target.transform.position;
        Debug.Log("World Position: " + worldPosition);

        // 获取局部坐标
        Vector3 localPosition = m_Target.transform.localPosition;
        Debug.Log("Local Position: " + localPosition);

        // 获取世界坐标
        Vector3 baselinkPosition = m_ReferenceModel.transform.position;
        Debug.Log("Base Link World Position: " + baselinkPosition);

        // 获取相对于 m_ReferenceModel 的局部坐标
        Vector3 relativePosition = m_ReferenceModel.transform.InverseTransformPoint(m_Target.transform.position);
        Debug.Log("Relative Position in Reference Model: " + relativePosition);

    }
}
