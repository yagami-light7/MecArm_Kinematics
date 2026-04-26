/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_Matrix.h
  * @brief      矩阵运算模块接口
  * @note       Middileware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-19-2026     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 本文件提供以下接口：
  * 1. 向量基础运算：清零、拷贝、加减、点乘、叉乘
  * 2. 矩阵基础运算：清零、拷贝、乘法、转置、矩阵乘向量
  * 3. 旋转矩阵工具：RotX / RotY / RotZ / RpyToRotZYX
  * 4. 线性方程求解：3x3 对称正定矩阵 LDLT 求解器   用于DLS算法
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"

#include <stddef.h>
#include <string.h>
#include "arm_math.h"

/**
 * @brief 宏定义
 */


/**
 * @brief 结构体
 */


/**
 * @brief 变量外部声明
 */


/**
 * @brief CPP部分
 */
#ifdef __cplusplus

#endif

namespace MWL_Matrix
{

/**
* @brief 向量清零
*/
template <size_t N>
inline void Vector_SetZero(float (&v)[N])
{
    memset(v, 0, sizeof(float) * N);
}


/**
 * @brief 向量拷贝
 */
template <size_t N>
inline void Vector_Copy(const float (&src)[N], float (&dst)[N])
{
    memcpy(dst, src, sizeof(float) * N);
}


/**
 * @brief 向量加法 out = a + b
 */
template <size_t N>
inline void Vector_Add(const float (&a)[N], const float (&b)[N], float (&out)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        out[i] = a[i] + b[i];
    }
}


/**
 * @brief 向量减法 out = a - b
 */
template <size_t N>
inline void Vector_Sub(const float (&a)[N], const float (&b)[N], float (&out)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        out[i] = a[i] - b[i];
    }
}


/**
 * @brief 向量数乘 out = scale * v
 */
template <size_t N>
inline void Vector_Scale(const float (&v)[N], float scale, float (&out)[N])
{
    for (size_t i = 0; i < N; ++i)
    {
        out[i] = scale * v[i];
    }
}


/**
 * @brief 向量点乘
 */
template <size_t N>
inline float Vector_Dot(const float (&a)[N], const float (&b)[N])
{
    float sum = 0.0f;
    for (size_t i = 0; i < N; ++i)
    {
        sum += a[i] * b[i];
    }
    return sum;
}


/**
 * @brief 三维向量叉乘 out = a x b
 */
inline void Vector_Cross3(const float (&a)[3], const float (&b)[3], float (&out)[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}


/**
 * @brief 矩阵清零
 */
template <size_t R, size_t C>
inline void Matrix_SetZero(float (&A)[R][C])
{
    memset(A, 0, sizeof(float) * R * C);
}


/**
 * @brief 矩阵拷贝
 */
template <size_t R, size_t C>
inline void Matrix_Copy(const float (&src)[R][C], float (&dst)[R][C])
{
    memcpy(dst, src, sizeof(float) * R * C);
}


/**
 * @brief 3x3 单位矩阵
 */
inline void MatIdentity3(float (&R)[3][3])
{
    Matrix_SetZero(R);
    R[0][0] = 1.0f;
    R[1][1] = 1.0f;
    R[2][2] = 1.0f;
}


/**
 * @brief 矩阵转置 out = A^T
 */
template <size_t R, size_t C>
inline void Matrix_Transpose(const float (&A)[R][C], float (&out)[C][R])
{
    for (size_t i = 0; i < R; ++i)
    {
        for (size_t j = 0; j < C; ++j)
        {
            out[j][i] = A[i][j];
        }
    }
}


/**
 * @brief 矩阵加法 out = A + B
 */
template <size_t R, size_t C>
inline void Matrix_Add(const float (&A)[R][C], const float (&B)[R][C], float (&out)[R][C])
{
    for (size_t i = 0; i < R; ++i)
    {
        for (size_t j = 0; j < C; ++j)
        {
            out[i][j] = A[i][j] + B[i][j];
        }
    }
}


/**
 * @brief 矩阵乘法 out = A * B
 */
template <size_t R, size_t K, size_t C>
inline void Matrix_Mul(const float (&A)[R][K], const float (&B)[K][C], float (&out)[R][C])
{
    float tmp[R][C];

    for (size_t i = 0; i < R; ++i)
    {
        for (size_t j = 0; j < C; ++j)
        {
            tmp[i][j] = 0.0f;
            for (size_t k = 0; k < K; ++k)
            {
                tmp[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    Matrix_Copy(tmp, out);
}


/**
 * @brief 3x3 矩阵对角线加标量 out = A + lambda * I
 */
    inline void Matrix_AddIdentityScaled3(const float (&A)[3][3], float lambda, float (&out)[3][3])
{
    Matrix_Copy(A, out);
    out[0][0] += lambda;
    out[1][1] += lambda;
    out[2][2] += lambda;
}


/**
 * @brief 矩阵乘向量 out = A * x
 */
template <size_t R, size_t C>
inline void MatVec_Mul(const float (&A)[R][C], const float (&x)[C], float (&out)[R])
{
    for (size_t i = 0; i < R; ++i)
    {
        out[i] = 0.0f;
        for (size_t j = 0; j < C; ++j)
        {
            out[i] += A[i][j] * x[j];
        }
    }
}


/**
 * @brief 绕 X 轴旋转矩阵
 */
inline void RotX(float angle, float (&R)[3][3])
{
    const float c = arm_cos_f32(angle);
    const float s = arm_sin_f32(angle);

    MatIdentity3(R);
    R[1][1] = c;
    R[1][2] = -s;
    R[2][1] = s;
    R[2][2] = c;
}

/**
* @brief 绕 Y 轴旋转矩阵
*/
inline void RotY(float angle, float (&R)[3][3])
{
    const float c = arm_cos_f32(angle);
    const float s = arm_sin_f32(angle);

    MatIdentity3(R);
    R[0][0] = c;
    R[0][2] = s;
    R[2][0] = -s;
    R[2][2] = c;
}

/**
* @brief 绕 Z 轴旋转矩阵
*/
inline void RotZ(float angle, float (&R)[3][3])
{
    const float c = arm_cos_f32(angle);
    const float s = arm_sin_f32(angle);

    MatIdentity3(R);
    R[0][0] = c;
    R[0][1] = -s;
    R[1][0] = s;
    R[1][1] = c;
}


/**
 * @brief RPY 转旋转矩阵
 * @note 采用与 Python 验证脚本一致的顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *        rpy[0] = roll, rpy[1] = pitch, rpy[2] = yaw
 */
inline void RpyToRotZYX(const float (&rpy)[3], float (&R)[3][3])
{
    float Rx[3][3];
    float Ry[3][3];
    float Rz[3][3];
    float tmp[3][3];

    RotX(rpy[0], Rx);
    RotY(rpy[1], Ry);
    RotZ(rpy[2], Rz);

    Matrix_Mul(Rz, Ry, tmp);
    Matrix_Mul(tmp, Rx, R);
}

/**
 * @brief 求解 3x3 对称正定线性方程 A * x = b
 * @note  使用 LDLT 分解，避免显式求逆
 * @param[in]  A    3x3 对称正定矩阵
 * @param[in]  b    右端项
 * @param[out] x    解向量
 * @param[in]  eps  正定性判断阈值
 * @retval     true  求解成功
 *             false 矩阵非正定或数值退化
 */
inline bool SolveSymmetricPositiveDefinite3x3LDLT(const float (&A)[3][3], const float (&b)[3], float (&x)[3], float eps = 1e-8f)
{
    // A = L * D * L^T
    float d0, d1, d2;
    float l10, l20, l21;

    d0 = A[0][0];
    if (d0 <= eps)
    {
        return false;
    }

    l10 = A[1][0] / d0;
    l20 = A[2][0] / d0;

    d1 = A[1][1] - l10 * l10 * d0;
    if (d1 <= eps)
    {
        return false;
    }

    l21 = (A[2][1] - l20 * l10 * d0) / d1;

    d2 = A[2][2] - l20 * l20 * d0 - l21 * l21 * d1;
    if (d2 <= eps)
    {
        return false;
    }

    // 先解 L * y = b
    float y0 = b[0];
    float y1 = b[1] - l10 * y0;
    float y2 = b[2] - l20 * y0 - l21 * y1;

    // 再解 D * z = y
    float z0 = y0 / d0;
    float z1 = y1 / d1;
    float z2 = y2 / d2;

    // 最后解 L^T * x = z
    x[2] = z2;
    x[1] = z1 - l21 * x[2];
    x[0] = z0 - l10 * x[1] - l20 * x[2];

    return true;
}

/**
 * @brief 求解 6x6 对称正定线性方程 A * x = b
 * @note  使用 LDLT 分解，避免显式求逆
 * @param[in]  A    6x6 对称正定矩阵
 * @param[in]  b    右端项
 * @param[out] x    解向量
 * @param[in]  eps  正定性判断阈值
 * @retval     true  求解成功
 *             false 矩阵非正定或数值退化
 */
inline bool SolveSymmetricPositiveDefinite6x6LDLT(const float (&A)[6][6], const float (&b)[6], float (&x)[6], float eps = 1e-8f)
{
    float L[6][6];
    float D[6];
    float y[6];
    float z[6];

    Matrix_SetZero(L);

    for (size_t i = 0; i < 6; ++i)
    {
        L[i][i] = 1.0f;
    }

    // A = L * D * L^T
    for (size_t j = 0; j < 6; ++j)
    {
        float d = A[j][j];

        for (size_t k = 0; k < j; ++k)
        {
            d -= L[j][k] * L[j][k] * D[k];
        }

        if (d <= eps)
        {
            return false;
        }

        D[j] = d;

        for (size_t i = j + 1; i < 6; ++i)
        {
            float lij = A[i][j];

            for (size_t k = 0; k < j; ++k)
            {
                lij -= L[i][k] * L[j][k] * D[k];
            }

            L[i][j] = lij / D[j];
        }
    }

    // 解 L * y = b
    for (size_t i = 0; i < 6; ++i)
    {
        y[i] = b[i];

        for (size_t k = 0; k < i; ++k)
        {
            y[i] -= L[i][k] * y[k];
        }
    }

    // 解 D * z = y
    for (size_t i = 0; i < 6; ++i)
    {
        z[i] = y[i] / D[i];
    }

    // 解 L^T * x = z
    for (int i = 5; i >= 0; --i)
    {
        x[i] = z[i];

        for (size_t k = static_cast<size_t>(i + 1); k < 6; ++k)
        {
            x[i] -= L[k][i] * x[k];
        }
    }

    return true;
}

} // namespace MWL_Matrix


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
