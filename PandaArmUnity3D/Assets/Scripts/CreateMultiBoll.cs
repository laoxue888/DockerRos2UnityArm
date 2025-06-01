using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CreateMultiBoll : MonoBehaviour
{
    public GameObject spherePrefab; // 球体的预制体
    public int rows = 5; // 球体阵列的行数
    public int columns = 5; // 球体阵列的列数
    public float spacing = 1.5f; // 球体之间的间距

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
                // 计算球体的位置
                Vector3 position = new Vector3(i * spacing, 0, j * spacing);
                // 实例化球体
                Instantiate(spherePrefab, position, Quaternion.identity);
            }
        }
    }
}