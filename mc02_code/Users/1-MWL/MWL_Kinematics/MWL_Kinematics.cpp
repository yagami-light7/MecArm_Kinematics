/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_Kinematics.cpp
  * @brief      提供串联型机械臂动力学计算模块接口
  * @note       Middileware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-18-2026     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 本文件实现以下功能：
  * 1.FK        运动学正解        实现关节空间到笛卡尔空间变换
  * 2.Jacobian  雅可比矩阵        实现雅可比矩阵的求解计算
  * 3.DLS       阻尼最小二乘      实现奇异位姿下的优化处理
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "MWL_Kinematics.h"
#include "MWL_Matrix.h"
#include "arm_math.h"
#include <string.h>
#include "MWL_SO3.h"

/**
 * @brief 命名空间
 */
namespace
{
    static const uint8_t kJointDim = 6U;
    static const uint8_t kCartesianDim = 3U;

    typedef struct
    {
        float xyz[3];
        float R_const[3][3];
        float axis_sign;
        float theta_bias;
    } JointChainParam_t;

    // 机械臂关节链参数，包含每个关节的安装位置、安装姿态、旋转轴符号和零位偏移
    static const JointChainParam_t kJointChain[kJointDim] =
    {
    {
        {0.0f, 0.0f, 0.0f },
    {
            { 1.000000000f,  0.000000000f,  0.000000000f },
            { 0.000000000f,  1.000000000f,  0.000000000f },
            { 0.000000000f,  0.000000000f,  1.000000000f }
             },
        1.0f,
        0.0f
        },

    {
        { 0.0f, 0.0325f, 0.193f },
    {
            { -0.892500992f, -0.451045429f,  0.000000166f },
            { -0.000000137f, -0.000000096f, -1.000000000f },
            {  0.451045429f, -0.892500992f,  0.000000024f }
             },
            1.0f,
            0.0f
        },
    {
        { 0.304f, 0.0f, -0.0025f },
    {
            { -0.892520696f,  0.451006438f,  0.000000000f },
            { -0.451006438f, -0.892520696f,  0.000000000f },
            {  0.000000000f,  0.000000000f,  1.000000000f }
            },
            1.0f,
            0.0f
        },
    {
        { 0.32542f, 0.0070546f, 0.034f },
    {
            { -0.000003673f,  0.021673303f,  0.999765106f },
            {  0.000000000f,  0.999765106f, -0.021673303f },
            { -1.000000000f, -0.000000080f, -0.000003672f }
            },
            1.0f,
            -0.3967f
        },
    {
        { -0.038577f, -0.016159f, 0.0995f },
    {
            { -0.386380114f,  0.000003388f,  0.922339638f },
            {  0.922339638f,  0.000001419f,  0.386380114f },
            {  0.000000000f,  1.000000000f, -0.000003673f }
            },
            -1.0f,
            1.44484f
        },
    {
        { -0.0038378f, 0.030308f, 0.041825f },
    {
            {  0.384958746f, -0.914343333f, -0.125630544f },
            {  0.048747485f, -0.115787376f,  0.992077097f },
            { -0.921645510f, -0.388032928f, -0.000001425f }
            },
            1.0f,
            -0.3f
        }
    };

    // TCP 相对Link6的变换参数：平移向量 + 旋转矩阵     Empty_Link6 -> TCP
    static const float kTcpTranslation[3] = {
            0.0f,
            0.0f,
            0.20f
    };

    static const float kTcpRotation[3][3] = {
            { 0.642990302f,  0.765874318f, 0.0f },
            {-0.765874318f,  0.642990302f, 0.0f },
            { 0.0f,          0.0f,         1.0f }
    };

    /**
    * @brief          正运动学计算
    * @param[in]      q       关节空间角度向量
    *                 p_end   笛卡尔空间末端位置向量
    */
    static void ComputeFk(const float (&q)[6], float (&p_end)[3])
    {
        float R_prev[3][3]; // 旋转矩阵
        float p_prev[3] = {0.0f, 0.0f, 0.0f};   // 位移向量

        MWL_Matrix::MatIdentity3(R_prev);

        for (int i = 0; i < kJointDim; ++i)
        {
            float R_joint[3][3];    // 固定joint的旋转矩阵
            float Rz_joint[3][3];   // 旋转joint后附加的旋转矩阵
            float rotated_xyz[3];   // 当前joint附加位移向量的基坐标系表达
            float p_joint[3];       // 当前joint位置向量的基坐标系表达

            // 计算关节扭转角
            const float theta_eff = kJointChain[i].axis_sign * q[i] + kJointChain[i].theta_bias;

            // 先应用当前 joint 的固定安装变换
            MWL_Matrix::MatVec_Mul(R_prev, kJointChain[i].xyz, rotated_xyz);
            MWL_Matrix::Vector_Add(p_prev, rotated_xyz, p_joint);
            MWL_Matrix::Matrix_Mul(R_prev, kJointChain[i].R_const, R_joint);

            // 再应用当前 joint 的转动
            MWL_Matrix::RotZ(theta_eff, Rz_joint);
            MWL_Matrix::Matrix_Mul(R_joint, Rz_joint, R_prev);
            MWL_Matrix::Vector_Copy(p_joint, p_prev);
        }

        // 最终末端位置
        p_end[0] = p_prev[0];
        p_end[1] = p_prev[1];
        p_end[2] = p_prev[2];
    }


    /**
    * @brief          FK和雅可比矩阵计算
    * @param[in]      q       关节空间角度向量
    *                 p_end   笛卡尔空间末端位置向量
    *                 J       雅可比矩阵
    */
    static void ComputeFkAndJacobian(const float (&q)[6], float (&p_end)[3], float (&J)[3][6])
    {
        float R_prev[3][3]; // 末端在基坐标系下的旋转矩阵
        float p_prev[3] = {0.0f, 0.0f, 0.0f};   // 末端在基坐标系下的位置向量

        float p_joint[kJointDim][3];    // 每个关节位置向量的基坐标系表达
        float z_joint[kJointDim][3];    // 每个关节旋转轴方向的基坐标系表达
        const float z_local[3] = {0.0f, 0.0f, 1.0f};    // 每个关节旋转轴在自身坐标系下的表达

        MWL_Matrix::MatIdentity3(R_prev);

        for (int i = 0; i < kJointDim; ++i)
        {
            float R_joint[3][3];
            float Rz_joint[3][3];
            float rotated_xyz[3];

            // 计算关节扭转角
            const float theta_eff = kJointChain[i].axis_sign * q[i] + kJointChain[i].theta_bias;

            // 先应用当前 joint 的固定安装变换
            MWL_Matrix::MatVec_Mul(R_prev, kJointChain[i].xyz, rotated_xyz);
            MWL_Matrix::Vector_Add(p_prev, rotated_xyz, p_joint[i]);
            MWL_Matrix::Matrix_Mul(R_prev, kJointChain[i].R_const, R_joint);

            // 当前 joint 轴方向，表达在基坐标系下
            MWL_Matrix::MatVec_Mul(R_joint, z_local, z_joint[i]);

            // 再应用当前 joint 的转动
            MWL_Matrix::RotZ(theta_eff, Rz_joint);
            MWL_Matrix::Matrix_Mul(R_joint, Rz_joint, R_prev);
            MWL_Matrix::Vector_Copy(p_joint[i], p_prev);
        }

        // 最终末端位置
        p_end[0] = p_prev[0];
        p_end[1] = p_prev[1];
        p_end[2] = p_prev[2];

        // 计算位置雅可比矩阵
        for (int i = 0; i < kJointDim; ++i)
        {
            float delta[3];
            float cross[3];

            MWL_Matrix::Vector_Sub(p_end, p_joint[i], delta);
            MWL_Matrix::Vector_Cross3(z_joint[i], delta, cross);

            J[0][i] = kJointChain[i].axis_sign * cross[0];
            J[1][i] = kJointChain[i].axis_sign * cross[1];
            J[2][i] = kJointChain[i].axis_sign * cross[2];
        }
    }

    /**
    * @brief          TCP FK + 6D Jacobian
    * @param[in]      q       6维关节角
    * @param[out]     p_tcp   TCP在基坐标系下的位置
    * @param[out]     R_tcp   TCP在基坐标系下的姿态
    * @param[out]     J_tcp   TCP 6D Jacobian，行顺序为 [v; w]
    */
    static void ComputeFkTcpAndJacobian6D(const float (&q)[6], float (&p_tcp)[3], float (&R_tcp)[3][3], float (&J_tcp)[6][6])
    {
        float R_prev[3][3]; // 末端在基坐标系下的旋转矩阵
        float p_prev[3] = {0.0f, 0.0f, 0.0f};   // 末端在基坐标系下的位置向量

        float p_joint[kJointDim][3];    // 每个关节位置向量的基坐标系表达
        float z_joint[kJointDim][3];    // 每个关节旋转轴方向的基坐标系表达
        const float z_local[3] = {0.0f, 0.0f, 1.0f};    // 每个关节旋转轴在自身坐标系下的表达

        MWL_Matrix::MatIdentity3(R_prev);

        for (int i = 0; i < kJointDim; i++)
        {
            float R_joint[3][3];
            float Rz_joint[3][3];
            float rotated_xyz[3];

            // 计算关节扭转角
            const float theta_eff = kJointChain[i].axis_sign * q[i] + kJointChain[i].theta_bias;

            // 先应用当前 joint 的固定安装变换
            MWL_Matrix::MatVec_Mul(R_prev, kJointChain[i].xyz, rotated_xyz);
            MWL_Matrix::Vector_Add(p_prev, rotated_xyz, p_joint[i]);
            MWL_Matrix::Matrix_Mul(R_prev, kJointChain[i].R_const, R_joint);

            // 当前 joint 轴方向，表达在基坐标系下
            MWL_Matrix::MatVec_Mul(R_joint, z_local, z_joint[i]);

            // 再应用当前 joint 的转动
            MWL_Matrix::RotZ(theta_eff, Rz_joint);
            MWL_Matrix::Matrix_Mul(R_joint, Rz_joint, R_prev);
            MWL_Matrix::Vector_Copy(p_joint[i], p_prev);
        }

        // 计算 TCP 在基坐标系下的位置和姿态
        float tcp_offset_world[3]; // TCP 相对于末端的偏移，表达在基坐标系下
        MWL_Matrix::MatVec_Mul(R_prev, kTcpTranslation, tcp_offset_world);
        MWL_Matrix::Vector_Add(p_prev, tcp_offset_world, p_tcp);
        MWL_Matrix::Matrix_Mul(R_prev, kTcpRotation, R_tcp);

        // 计算 TCP 的 6D 雅可比矩阵 几何法
        for (int i = 0; i < kJointDim; i++)
        {
            float delta[3];
            float cross[3];

            MWL_Matrix::Vector_Sub(p_tcp, p_joint[i], delta);
            MWL_Matrix::Vector_Cross3(z_joint[i], delta, cross);

            // 线速度雅可比
            J_tcp[0][i] = kJointChain[i].axis_sign * cross[0];
            J_tcp[1][i] = kJointChain[i].axis_sign * cross[1];
            J_tcp[2][i] = kJointChain[i].axis_sign * cross[2];

            // 角速度雅可比
            J_tcp[3][i] = kJointChain[i].axis_sign * z_joint[i][0];
            J_tcp[4][i] = kJointChain[i].axis_sign * z_joint[i][1];
            J_tcp[5][i] = kJointChain[i].axis_sign * z_joint[i][2];
        }
    }

    /**
    * @brief 构造 6D Weighted-DLS 的正规方程
    *        A = J^T * Wx * J + lambda^2 * Wq
    *        b = J^T * Wx * rhs
    */
    static void BuildWeightedDLSNormalEquation6D(const float (&J6)[6][6], const float (&twist_target)[6],
                                                 const float (&Wx)[6], const float (&Wq)[6],
                                                 float damping, float (&A)[6][6], float (&b)[6])
    {
        const float lambda = damping * damping;

        MWL_Matrix::Matrix_SetZero(A);
        MWL_Matrix::Vector_SetZero(b);

        // 计算 A = J^T * Wx * J + lambda^2 * Wq
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                float sum = 0.0f;

                for (int k = 0; k < 6; k++)
                {
                    sum += J6[k][i] * Wx[k] * J6[k][j];
                }

                A[i][j] = sum;
            }
            A[i][i] += lambda * Wq[i];
        }

        // 计算 b = J^T * Wx * twist_target
        for (int i = 0; i < 6; i++)
        {
            float sum = 0.0f;

            for (int k = 0; k < 6; k++)
            {
                sum += J6[k][i] * Wx[k] * twist_target[k];
            }

            b[i] = sum;
        }
    }
}


/**
* @brief 运动学正解
*/
void Class_Kinematics::FK(const float (&q)[6], float (&pos)[3]) const
{
    ComputeFk(q, pos);
}


/**
* @brief 计算雅可比矩阵
*/
void Class_Kinematics::Calc_Jacobian(const float (&q)[6], float (&J)[3][6]) const
{
    float pos_end[3];
    ComputeFkAndJacobian(q, pos_end, J);
}


/**
* @brief 计算正运动学 + 雅可比矩阵
*/
void Class_Kinematics::FK_and_Jacobian(const float (&q)[6], float (&pos)[3], float (&J)[3][6]) const
{
    ComputeFkAndJacobian(q, pos, J);
}


/**
* @brief 运动学正解-TCP位姿 6d
*/
void Class_Kinematics::FK_TCP(const float (&q)[6], float (&p_tcp)[3], float (&R_tcp)[3][3]) const
{
    float J_tcp_dummy[6][6];
    ComputeFkTcpAndJacobian6D(q, p_tcp, R_tcp, J_tcp_dummy);
}

/**
* @brief 计算TCP正运动学 + 6D雅可比矩阵
*/
void Class_Kinematics::FK_TCP_and_Jacobian6D(const float (&q)[6], float (&p_tcp)[3],
                           float (&R_tcp)[3][3], float (&J_tcp)[6][6]) const
{
    ComputeFkTcpAndJacobian6D(q, p_tcp, R_tcp, J_tcp);
}


/**
* @brief  求解笛卡尔空间速度
*/
void Class_Kinematics::Calc_CartesianVel(const float (&q)[6], const float (&q_dot)[6], float (&x_dot)[3]) const
{
    float J[3][6];
    float pos_end[3];

    ComputeFkAndJacobian(q, pos_end, J);

    for (int row = 0; row < 3; ++row)
    {
        x_dot[row] = 0.0f;
        for (int col = 0; col < 6; ++col)
        {
            x_dot[row] += J[row][col] * q_dot[col];
        }
    }
}


/**
    * @brief  求解笛卡尔空间下 TCP的twist
    */
void Class_Kinematics::Calc_TCP_Twist(const float (&J_tcp)[6][6], const float (&q_dot)[6], float (&twist_tcp)[6]) const
{
    MWL_Matrix::MatVec_Mul(J_tcp, q_dot, twist_tcp);
}


/**
* @brief  基于log3 计算基坐标系下的姿态误差表达
*/
void Class_Kinematics::Calc_OrientationError_World(const float (&R_des)[3][3], const float (&R_cur)[3][3], float (&e_rot)[3]) const
{
    MWL_SO3::OrientationErrorWorld(R_des, R_cur, e_rot);
}


/**
* @brief  求解笛卡尔空间下 TCP的6D位姿误差表达 位置误差 + 基坐标系下的姿态误差
*/
void Class_Kinematics::Calc_PoseError_6D(const float (&p_des)[3], const float (&p_cur)[3],
                       const float (&R_des)[3][3], const float (&R_cur)[3][3], float (&e_pose)[6]) const
{
    float e_pos[3];
    float e_rot[3];

    // 位置误差
    MWL_Matrix::Vector_Sub(p_des, p_cur, e_pos);
    // 姿态误差
    MWL_SO3::OrientationErrorWorld(R_des, R_cur, e_rot);

    // 组合位置误差和姿态误差
    e_pose[0] = e_pos[0];
    e_pose[1] = e_pos[1];
    e_pose[2] = e_pos[2];
    e_pose[3] = e_rot[0];
    e_pose[4] = e_rot[1];
    e_pose[5] = e_rot[2];
}

/**
* @brief        求解关节空间速度  阻尼最小二乘法 仅位置雅可比矩阵 3x6
* @param[in]    J               位置雅可比矩阵
* @param[in]    xdot_target     笛卡尔空间目标速度
* @param[in]    damping         阻尼系数
* @param[out]   qdot_target     关节空间目标速度
* @note         1. 阻尼最小二乘法在雅可比矩阵接近奇异时能够提供更稳定的解，避免过大的关节速度。
*               2. 阻尼系数的选择需要根据具体机械臂的运动范围和任务需求进行调整，过大可能导致响应过慢，过小可能无法有效处理奇异情况。
*               3. 该函数仅使用位置雅可比矩阵进行计算
*/
void Class_Kinematics::Solve_JointVel_DLS(const float (&J)[3][6], const float (&xdot_target)[3], float damping, float (&qdot_target)[6]) const
{
    float Jt[6][3];     // 雅可比矩阵转置
    float JJt[3][3];    // J * J^T
    float A[3][3];      // J * J^T + lambda^2 * I
    float y[3];         // y = A^-1 * xdot_target    或者 y = J * qdot

    const float lambda2 = damping * damping;

    // J^T
    MWL_Matrix::Matrix_Transpose(J, Jt);

    // JJt = J * J^T
    MWL_Matrix::Matrix_Mul(J, Jt, JJt);

    // A = J*J^T + lambda^2 * I
    MWL_Matrix::Matrix_AddIdentityScaled3(JJt, lambda2, A);

    // 解方程 A * y = xdot_target
    if(!MWL_Matrix::SolveSymmetricPositiveDefinite3x3LDLT(A, xdot_target, y))
    {
        MWL_Matrix::Vector_SetZero(qdot_target);
        return;
    }
    // qdot = J^T * y
    MWL_Matrix::MatVec_Mul(Jt, y, qdot_target);
}

/**
* @brief          求解关节空间速度  加权阻尼最小二乘法 6x6
* @param[in]      J6               6*6雅可比矩阵
* @param[in]      twist_target     笛卡尔空间目标twist
* @param[in]      Wx               笛卡尔空间权重
* @param[in]      Wq               关节空间权重
* @param[in]      damping          阻尼系数
* @param[out]     qdot_target      关节空间目标速度
* @note         1. 阻尼最小二乘法在雅可比矩阵接近奇异时能够提供更稳定的解，避免过大的关节速度。
*               2. 阻尼系数的选择需要根据具体机械臂的运动范围和任务需求进行调整，过大可能导致响应过慢，过小可能无法有效处理奇异情况。
*               3. 该函数仅使用位置雅可比矩阵进行计算
*/
void Class_Kinematics::Solve_JointVel_WeightedDLS_6D(const float (&J6)[6][6], const float (&twist_target)[6],
                                   const float (&Wx)[6], const float (&Wq)[6],
                                   float damping, float (&qdot_target)[6]) const
{
    float A[6][6];      // A = J^T * Wx * J + lambda^2 * Wq * I
    float b[6];         // b = J^T * Wx * twist_target
    const float lambda2 = damping * damping;

    MWL_Matrix::Matrix_SetZero(A);
    MWL_Matrix::Vector_SetZero(b);

    // 构造W-DLS的正规方程
    BuildWeightedDLSNormalEquation6D(J6, twist_target, Wx, Wq, damping, A, b);

    // LDLT分解求解线性方程 A * qdot_target = b
    if (!MWL_Matrix::SolveSymmetricPositiveDefinite6x6LDLT(A, b, qdot_target))
    {
        MWL_Matrix::Vector_SetZero(qdot_target);
        return;
    }

}


/**
* @brief          6D W-DLS + 零空间软投影
* @param[in]      J6               TCP 6D Jacobian
* @param[in]      twist_target     主任务目标 twist
* @param[in]      Wx               笛卡尔空间对角权重
* @param[in]      Wq               关节空间对角权重
* @param[in]      damping          阻尼系数
* @param[in]      qdot_null_raw    零空间原始偏好速度
* @param[out]     qdot_target      合成后的关节目标速度
* @note           数学形式：
*                 qdot = qdot_primary + (I - J#J) qdot_null_raw
*                 为避免显式构造投影矩阵，实际实现为：
*                 qdot = qdot_primary + qdot_null_raw - J#(J qdot_null_raw)
*/
void Class_Kinematics::Solve_JointVel_WeightedDLS_NullSpace_6D(const float (&J6)[6][6], const float (&twist_target)[6],
                                             const float (&Wx)[6],const float (&Wq)[6], float damping,
                                             const float (&qdot_null_raw)[6], float (&qdot_target)[6]) const
{
    float A[6][6];          // A = J^T * Wx * J
    float b_task[6];        // b = J^T * Wx * twist_target
    float qdot_primary[6];  // 主任务的关节速度解

    /* 1.求解主任务 qdot_primary = J# @ twist_target */
    BuildWeightedDLSNormalEquation6D(J6, twist_target, Wx, Wq, damping, A, b_task);
    if( !MWL_Matrix::SolveSymmetricPositiveDefinite6x6LDLT(A, b_task, qdot_primary) )
    {
        MWL_Matrix::Vector_SetZero(qdot_primary);
    }

    /* 2.计算 J @ qdot_null_raw */
    float twist_null[6];
    MWL_Matrix::MatVec_Mul(J6, qdot_null_raw, twist_null);

    /* 3.求解J# @ (J @ qdot_null_raw) */
    float b_null[6];
    float qdot_null[6];
    BuildWeightedDLSNormalEquation6D(J6, twist_null, Wx, Wq, damping, A, b_null);
    if( !MWL_Matrix::SolveSymmetricPositiveDefinite6x6LDLT(A, b_null, qdot_null) )
    {
        // 求解失败 退化为仅主任务求解
        MWL_Matrix::Vector_Copy(qdot_primary, qdot_target);
        return;
    }

    /* 4.合成最终解 qdot = qdot_primary + qdot_null_raw - J#(J @ qdot_null_raw) */
    for (int i = 0; i < 6; i++)
    {
        qdot_target[i] = qdot_primary[i] + qdot_null_raw[i] - qdot_null[i];
    }

}


/**
  * @brief          对输入向量进行限幅
  * @param[in]      vector      输入向量
  *                 vector_min   向量最小值
  *                 vector_max   向量最大值
  * @retval         限幅后的结果
  */
float MWL_Vector_Clamp(float vector, float vector_min, float vector_max)
{
    assert_param(vector_min < vector_max);

    if (vector_min >= vector_max)
    {
        return vector;
    }

    if (vector < vector_min)
    {
        vector = vector_min;
    }

    if (vector > vector_max)
    {
        vector = vector_max;
    }

    return vector;
}