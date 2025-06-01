using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreateMultiBoll : MonoBehaviour
{
    public GameObject spherePrefab; // �����Ԥ����
    public int rows = 5; // �������е�����
    public int columns = 5; // �������е�����
    public float spacing = 1.5f; // ����֮��ļ��

    // Start is called before the first frame update
    void Start()
    {
        CreateSphereArray();
    }

    void CreateSphereArray()
    {
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < columns; j++)
            {
                // ���������λ��
                Vector3 position = new Vector3(i * spacing, 0, j * spacing);
                // ʵ��������
                Instantiate(spherePrefab, position, Quaternion.identity);
            }
        }
    }
}