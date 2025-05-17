using UnityEngine;
using System.Collections.Generic;

public class AdvancedPandaIK : MonoBehaviour
{
    // Panda机械臂参数
    public Transform baseLink;
    public Transform[] joints = new Transform[7];
    public Transform endEffector;
    public Transform ikTarget;
    public Transform ikOrientation; // 可选的目标方向

    // IK参数
    public float positionThreshold = 0.005f;
    public float rotationThreshold = 0.5f; // 角度阈值(度)
    public int maxIterations = 100;
    public float learningRate = 0.1f;

    // 关节限制(弧度)
    private Vector2[] jointLimits = new Vector2[]
    {
        new Vector2(-2.8973f, 2.8973f),
        new Vector2(-1.7628f, 1.7628f),
        new Vector2(-2.8973f, 2.8973f),
        new Vector2(-3.0718f, -0.0698f),
        new Vector2(-2.8973f, 2.8973f),
        new Vector2(-0.0175f, 3.7525f),
        new Vector2(-2.8973f, 2.8973f)
    };

    void Update()
    {
        if (ikTarget != null)
        {
            SolveIK(ikTarget.position,
                   ikOrientation != null ? ikOrientation.rotation : endEffector.rotation);
        }
    }

    public void SolveIK(Vector3 targetPosition, Quaternion targetRotation)
    {
        // 初始化关节角度
        float[] angles = new float[7];
        for (int i = 0; i < 7; i++)
        {
            angles[i] = joints[i].localEulerAngles.z;
            if (angles[i] > 180) angles[i] -= 360; // 转换为-180到180范围
            angles[i] *= Mathf.Deg2Rad;
        }

        // IK求解
        for (int iter = 0; iter < maxIterations; iter++)
        {
            // 计算当前末端位姿
            Matrix4x4 forwardKinematics = ComputeForwardKinematics(angles);
            Vector3 currentPos = forwardKinematics.GetColumn(3);
            Quaternion currentRot = forwardKinematics.rotation;

            // 检查收敛
            float posError = Vector3.Distance(currentPos, targetPosition);
            float rotError = Quaternion.Angle(currentRot, targetRotation);

            if (posError < positionThreshold && rotError < rotationThreshold)
                break;

            // 计算雅可比矩阵
            Matrix4x4[] jacobian = ComputeJacobian(angles);

            // 计算误差
            Vector3 posDelta = targetPosition - currentPos;
            Quaternion rotDelta = targetRotation * Quaternion.Inverse(currentRot);
            rotDelta.ToAngleAxis(out float angle, out Vector3 axis);
            Vector3 rotDeltaVec = axis * (angle * Mathf.Deg2Rad);

            Vector6 error = new Vector6(
                posDelta.x, posDelta.y, posDelta.z,
                rotDeltaVec.x, rotDeltaVec.y, rotDeltaVec.z
            );

            // 阻尼最小二乘法求解
            float[,] J = new float[6, 7];
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 7; j++)
                {
                    J[i, j] = jacobian[j][i / 3, i % 3];
                }
            }

            // 伪逆求解
            float[] deltaAngles = DampedLeastSquares(J, error.ToArray(), 0.1f);

            // 更新角度
            for (int i = 0; i < 7; i++)
            {
                angles[i] += deltaAngles[i] * learningRate;
                // 应用关节限制
                angles[i] = Mathf.Clamp(angles[i], jointLimits[i].x, jointLimits[i].y);
            }
        }

        // 应用结果到Unity关节
        for (int i = 0; i < 7; i++)
        {
            Vector3 localEuler = joints[i].localEulerAngles;
            localEuler.z = angles[i] * Mathf.Rad2Deg;
            joints[i].localEulerAngles = localEuler;
        }
    }

    Matrix4x4 ComputeForwardKinematics(float[] angles)
    {
        // Panda机械臂的DH参数
        float[] d = { 0.333f, 0.0f, 0.316f, 0.0f, 0.384f, 0.0f, 0.107f };
        float[] a = { 0.0f, 0.0f, 0.0f, 0.0825f, -0.0825f, 0.0f, 0.088f };
        float[] alpha = { 0.0f, -Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2, -Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2 };

        Matrix4x4 result = Matrix4x4.identity;

        for (int i = 0; i < 7; i++)
        {
            float theta = angles[i];
            Matrix4x4 mat = Matrix4x4.identity;

            float ct = Mathf.Cos(theta);
            float st = Mathf.Sin(theta);
            float ca = Mathf.Cos(alpha[i]);
            float sa = Mathf.Sin(alpha[i]);

            mat[0, 0] = ct;
            mat[0, 1] = -st * ca;
            mat[0, 2] = st * sa;
            mat[0, 3] = a[i] * ct;

            mat[1, 0] = st;
            mat[1, 1] = ct * ca;
            mat[1, 2] = -ct * sa;
            mat[1, 3] = a[i] * st;

            mat[2, 0] = 0;
            mat[2, 1] = sa;
            mat[2, 2] = ca;
            mat[2, 3] = d[i];

            result *= mat;
        }

        return result;
    }

    Matrix4x4[] ComputeJacobian(float[] angles)
    {
        Matrix4x4[] jacobian = new Matrix4x4[7];
        Matrix4x4[] transforms = new Matrix4x4[8]; // 7关节+末端

        // 计算所有变换矩阵
        transforms[0] = Matrix4x4.identity;
        for (int i = 0; i < 7; i++)
        {
            transforms[i + 1] = transforms[i] * ComputeTransformMatrix(i, angles[i]);
        }

        // 末端位置
        Vector3 endPos = transforms[7].GetColumn(3);

        // 计算每个关节的雅可比列
        for (int i = 0; i < 7; i++)
        {
            Vector3 jointPos = transforms[i].GetColumn(3);
            Vector3 axis = transforms[i].GetColumn(2); // Z轴

            // 线性速度部分
            Vector3 linear = Vector3.Cross(axis, endPos - jointPos);

            // 角速度部分就是旋转轴本身
            Vector3 angular = axis;

            // 构建4x4矩阵(实际只用了前3行)
            Matrix4x4 j = Matrix4x4.zero;
            j.SetColumn(0, linear);
            j.SetColumn(1, angular);

            jacobian[i] = j;
        }

        return jacobian;
    }

    Matrix4x4 ComputeTransformMatrix(int jointIndex, float angle)
    {
        // DH参数
        float[] d = { 0.333f, 0.0f, 0.316f, 0.0f, 0.384f, 0.0f, 0.107f };
        float[] a = { 0.0f, 0.0f, 0.0f, 0.0825f, -0.0825f, 0.0f, 0.088f };
        float[] alpha = { 0.0f, -Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2, -Mathf.PI / 2, Mathf.PI / 2, Mathf.PI / 2 };

        float ct = Mathf.Cos(angle);
        float st = Mathf.Sin(angle);
        float ca = Mathf.Cos(alpha[jointIndex]);
        float sa = Mathf.Sin(alpha[jointIndex]);

        Matrix4x4 mat = Matrix4x4.identity;
        mat[0, 0] = ct;
        mat[0, 1] = -st * ca;
        mat[0, 2] = st * sa;
        mat[0, 3] = a[jointIndex] * ct;

        mat[1, 0] = st;
        mat[1, 1] = ct * ca;
        mat[1, 2] = -ct * sa;
        mat[1, 3] = a[jointIndex] * st;

        mat[2, 0] = 0;
        mat[2, 1] = sa;
        mat[2, 2] = ca;
        mat[2, 3] = d[jointIndex];

        return mat;
    }

    float[] DampedLeastSquares(float[,] J, float[] error, float damping)
    {
        int m = J.GetLength(0);
        int n = J.GetLength(1);

        // J^T
        float[,] JT = new float[n, m];
        for (int i = 0; i < n; i++)
            for (int j = 0; j < m; j++)
                JT[i, j] = J[j, i];

        // J^T * J
        float[,] JTJ = new float[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                JTJ[i, j] = 0;
                for (int k = 0; k < m; k++)
                    JTJ[i, j] += JT[i, k] * J[k, j];

                if (i == j)
                    JTJ[i, j] += damping * damping;
            }
        }

        // 解线性方程组 (JTJ + λI)x = JTe
        float[] JTe = new float[n];
        for (int i = 0; i < n; i++)
        {
            JTe[i] = 0;
            for (int j = 0; j < m; j++)
                JTe[i] += JT[i, j] * error[j];
        }

        // 这里简化处理，实际应用中应使用更稳健的线性代数库
        return SolveLinearSystem(JTJ, JTe);
    }

    float[] SolveLinearSystem(float[,] A, float[] b)
    {
        // 简单的高斯消元法，实际项目中应使用专业数学库
        int n = b.Length;
        float[] x = new float[n];

        // 前向消元
        for (int k = 0; k < n - 1; k++)
        {
            for (int i = k + 1; i < n; i++)
            {
                float factor = A[i, k] / A[k, k];
                for (int j = k + 1; j < n; j++)
                    A[i, j] -= factor * A[k, j];
                b[i] -= factor * b[k];
            }
        }

        // 回代
        x[n - 1] = b[n - 1] / A[n - 1, n - 1];
        for (int i = n - 2; i >= 0; i--)
        {
            float sum = b[i];
            for (int j = i + 1; j < n; j++)
                sum -= A[i, j] * x[j];
            x[i] = sum / A[i, i];
        }

        return x;
    }

    // 辅助结构: 6维向量(3位置+3旋转)
    struct Vector6
    {
        public float x, y, z, a, b, c;

        public Vector6(float x, float y, float z, float a, float b, float c)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.a = a;
            this.b = b;
            this.c = c;
        }

        public float[] ToArray()
        {
            return new float[] { x, y, z, a, b, c };
        }
    }
}