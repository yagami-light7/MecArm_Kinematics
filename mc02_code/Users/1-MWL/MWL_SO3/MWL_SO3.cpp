/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_SO3.cpp
  * @brief      提供 SO3 相关数学接口 主要用于旋转矩阵和轴角向量之间的转换
  * @note       Middileware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-24-2026     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "MWL_SO3.h"
#include "MWL_Matrix.h"

namespace
{
    /**
    * @brief 常量 Pi
    */
    static const float kPi = 3.14159265358979323846f;

    /**
    * @brief 浮点限幅
    */
    static float ClampScalar(float x, float x_min, float x_max)
    {
        if (x < x_min) return x_min;
        if (x > x_max) return x_max;
        return x;
    }

    /**
    * @brief 取两个浮点数中的较大值
    */
    static float MaxScalar(float a, float b)
    {
        return (a > b) ? a : b;
    }

    /**
    * @brief 浮点开方
    */
    static float SafeSqrt(float x)
    {
        if (x <= 0.0f)
        {
            return 0.0f;
        }

        float y = 0.0f;
        if (arm_sqrt_f32(x, &y) != ARM_MATH_SUCCESS)
        {
            return 0.0f;
        }

        return y;
    }


    static void SkewFromVector(const float (&v)[3], float (&S)[3][3])
    {
        S[0][0] = 0.0f;   S[0][1] = -v[2];  S[0][2] =  v[1];
        S[1][0] =  v[2];  S[1][1] = 0.0f;   S[1][2] = -v[0];
        S[2][0] = -v[1];  S[2][1] =  v[0];  S[2][2] = 0.0f;
    }

    /**
    * @brief 提取 3x3 反对称矩阵的 vee 运算
    * @note  对应：
    *        [  0  -z   y ]
    *        [  z   0  -x ]
    *        [ -y   x   0 ]
    *        -> [x, y, z]
    */
    static void VeeOfSkewPart(const float (&R)[3][3], float (&vee)[3])
    {
        vee[0] = R[2][1] - R[1][2];
        vee[1] = R[0][2] - R[2][0];
        vee[2] = R[1][0] - R[0][1];
    }
}

namespace MWL_SO3
{
    /**
    * @brief    计算3维向量的欧几里得范数
    * @param    v   3维向量
    *
    */
    float Norm3(const float (&v)[3])
    {
        return SafeSqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }


    /**
    * @brief SO(3) 对数映射，输出旋转向量 phi
    * @note  输入 R 必须是旋转矩阵
    *        输出 phi 的方向是旋转轴，模长是旋转角
    */
    void Log3(const float(&R)[3][3], float (&phi)[3])
    {
        const float trace_R = R[0][0] + R[1][1] + R[2][2];  // 旋转矩阵的迹
        const float cos_theta = ClampScalar(0.5f * (trace_R - 1.0f), -1.0f, 1.0f);    // 计算旋转角的余弦值
        const float one_minus_cos = 1.0f - cos_theta;

        float vee[3];
        VeeOfSkewPart(R, vee);

        // 采用小角度 一阶近似 log(R) = 0.5 * vee(R - R^T)
        if (one_minus_cos < 1.0e-6f)   // 小角度误差时避开acosf运算
        {
            phi[0] = 0.5 * vee[0];
            phi[1] = 0.5 * vee[1];
            phi[2] = 0.5 * vee[2];

            return;
        }

        const float theta = acosf(cos_theta);   // 旋转角

        // 接近 pi：常规 theta / (2 sin theta) 会数值不稳定
        if ((kPi - theta) < 1.0e-3f)
        {
            float axis[3];

            axis[0] = SafeSqrt(MaxScalar(0.0f, 0.5f * (R[0][0] + 1.0f)));
            axis[1] = SafeSqrt(MaxScalar(0.0f, 0.5f * (R[1][1] + 1.0f)));
            axis[2] = SafeSqrt(MaxScalar(0.0f, 0.5f * (R[2][2] + 1.0f)));

            // 选最大分量恢复符号，避免分母太小
            if (axis[0] >= axis[1] && axis[0] >= axis[2] && axis[0] > 1.0e-6f)
            {
                axis[1] = (R[0][1] + R[1][0]) / (4.0f * axis[0]);
                axis[2] = (R[0][2] + R[2][0]) / (4.0f * axis[0]);
            }
            else if (axis[1] >= axis[0] && axis[1] >= axis[2] && axis[1] > 1.0e-6f)
            {
                axis[0] = (R[0][1] + R[1][0]) / (4.0f * axis[1]);
                axis[2] = (R[1][2] + R[2][1]) / (4.0f * axis[1]);
            }
            else if (axis[2] > 1.0e-6f)
            {
                axis[0] = (R[0][2] + R[2][0]) / (4.0f * axis[2]);
                axis[1] = (R[1][2] + R[2][1]) / (4.0f * axis[2]);
            }
            else
            {
                // 极端退化时回退到 0，避免数值爆炸
                phi[0] = 0.0f;
                phi[1] = 0.0f;
                phi[2] = 0.0f;
                return;
            }

            phi[0] = theta * axis[0];
            phi[1] = theta * axis[1];
            phi[2] = theta * axis[2];
            return;
        }

        // 常规情况
        {
            const float scale = theta / (2.0f * arm_sin_f32(theta));

            phi[0] = scale * vee[0];
            phi[1] = scale * vee[1];
            phi[2] = scale * vee[2];
        }
    }

    void Exp3(const float (&phi)[3], float (&R)[3][3])
    {
        const float theta_sq = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];

        MWL_Matrix::MatIdentity3(R);

        float K[3][3];
        float K2[3][3];
        SkewFromVector(phi, K);
        MWL_Matrix::Matrix_Mul(K, K, K2);

        if (theta_sq < 1.0e-12f)
        {
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    R[i][j] += K[i][j] + 0.5f * K2[i][j];
                }
            }
            return;
        }

        const float theta = SafeSqrt(theta_sq);
        const float a = arm_sin_f32(theta) / theta;
        const float b = (1.0f - arm_cos_f32(theta)) / theta_sq;

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                R[i][j] += a * K[i][j] + b * K2[i][j];
            }
        }
    }

    /**
    * @brief 计算世界坐标系表达的姿态误差
    * @note  与 PC 侧 pose_error.py 完全一致：
    *        R_rel = R_cur^T * R_des
    *        e_rot_local = log3(R_rel)
    *        e_rot_world = R_cur * e_rot_local
    */
    void OrientationErrorWorld(const float (&R_des)[3][3],
                               const float (&R_cur)[3][3],
                               float (&e_rot)[3])
    {
        float R_cur_T[3][3];
        float R_rel[3][3];
        float e_rot_local[3];

        MWL_Matrix::Matrix_Transpose(R_cur, R_cur_T);
        MWL_Matrix::Matrix_Mul(R_cur_T, R_des, R_rel);

        Log3(R_rel, e_rot_local);
        MWL_Matrix::MatVec_Mul(R_cur, e_rot_local, e_rot);
    }

}