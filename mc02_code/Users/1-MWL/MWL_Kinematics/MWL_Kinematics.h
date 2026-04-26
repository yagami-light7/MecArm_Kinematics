/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_Kinematics.h
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
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"


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


class Class_Kinematics
{
public:

    /**
    * @brief 运动学正解-Link6位置 3d
    */
    void FK(const float (&q)[6], float (&pos)[3]) const;

    /**
    * @brief  计算雅可比矩阵
    */
    void Calc_Jacobian(const float (&q)[6], float (&J)[3][6]) const;

    /**
    * @brief 计算正运动学 + 雅可比矩阵
    */
    void FK_and_Jacobian(const float (&q)[6], float (&pos)[3], float (&J)[3][6]) const;

    /**
    * @brief  求解笛卡尔空间速度-3d 仅位置
    */
    void Calc_CartesianVel(const float (&q)[6], const float (&q_dot)[6], float (&x_dot)[3]) const;

    /**
   * @brief 运动学正解-TCP位姿 6d
   */
    void FK_TCP(const float (&q)[6], float (&p_tcp)[3], float (&R_tcp)[3][3]) const;

    /**
    * @brief 计算TCP正运动学 + 6D雅可比矩阵
    */
    void FK_TCP_and_Jacobian6D(const float (&q)[6], float (&p_tcp)[3],
                               float (&R_tcp)[3][3], float (&J_tcp)[6][6]) const;

    /**
    * @brief  求解笛卡尔空间下 TCP的twist
    */
    void Calc_TCP_Twist(const float (&J_tcp)[6][6], const float (&q_dot)[6], float (&twist_tcp)[6]) const;

    /**
    * @brief  基于log3/exp3 计算基坐标系下的姿态误差表达
    */
    void Calc_OrientationError_World(const float (&R_des)[3][3], const float (&R_cur)[3][3], float (&e_rot)[3]) const;

    /**
    * @brief  求解笛卡尔空间下 TCP的6D位姿误差表达 位置误差 + 基坐标系下的姿态误差
    */
    void Calc_PoseError_6D(const float (&p_des)[3], const float (&p_cur)[3],
                           const float (&R_des)[3][3], const float (&R_cur)[3][3], float (&e_pose)[6]) const;

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
    void Solve_JointVel_DLS(const float (&J)[3][6], const float (&xdot_target)[3], float damping, float (&qdot_target)[6]) const;


    /**
    * @brief          求解关节空间速度  加权阻尼最小二乘法 6x6
    * @param[in]      J6               TCP 6D Jacobian
    * @param[in]      twist_target     笛卡尔空间目标twist
    * @param[in]      Wx               笛卡尔空间权重
    * @param[in]      Wq               关节空间权重
    * @param[in]      damping          阻尼系数
    * @param[out]     qdot_target      关节空间目标速度
    * @note         1. 阻尼最小二乘法在雅可比矩阵接近奇异时能够提供更稳定的解，避免过大的关节速度。
    *               2. 阻尼系数的选择需要根据具体机械臂的运动范围和任务需求进行调整，过大可能导致响应过慢，过小可能无法有效处理奇异情况。
    *               3. 该函数仅使用位置雅可比矩阵进行计算
    */
    void Solve_JointVel_WeightedDLS_6D(const float (&J6)[6][6], const float (&twist_target)[6],
                                       const float (&Wx)[6], const float (&Wq)[6],
                                       float damping, float (&qdot_target)[6]) const;


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
    void Solve_JointVel_WeightedDLS_NullSpace_6D(const float (&J6)[6][6], const float (&twist_target)[6],
                                                 const float (&Wx)[6],const float (&Wq)[6], float damping,
                                                 const float (&qdot_null_raw)[6], float (&qdot_target)[6]) const;


};


#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


/**
* @brief          对输入向量进行限幅
*/
extern float MWL_Vector_Clamp(float vector, float vector_min, float vector_max);


#ifdef __cplusplus
}
#endif
