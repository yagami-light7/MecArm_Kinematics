/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       
  * @brief      
  * @note       
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
#include "MWL_Trajectory.h"
#include "MWL_Matrix.h"
#include "MWL_SO3.h"

namespace
{
    static const float kMinJerkPeakScale = 1.875f;
    const float kCartesianTrajMinTime = 0.10f;  // 轨迹规划最短时间 s
    const float kCartesianTrajPeakLinearSpeed = 0.08f;   // 轨迹规划最大线速度 m/s
    const float kCartesianTrajPeakAngularSpeed = 0.10f;  // 轨迹规划最大角速度 rad/s
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

}

namespace MWL_Trajectory
{
    void InitCartesianPoseTrajectory(CartesianPoseTrajectory_t &traj)
    {
        traj.active = 0U;
        traj.elapsed = 0.0f;
        traj.duration = 0.0f;

        MWL_Matrix::Vector_SetZero(traj.p_start);
        MWL_Matrix::Vector_SetZero(traj.p_goal);
        MWL_Matrix::Vector_SetZero(traj.p_ref);
        MWL_Matrix::Vector_SetZero(traj.v_ff);
        MWL_Matrix::Vector_SetZero(traj.rotvec_total_local);
        MWL_Matrix::Vector_SetZero(traj.w_ff);
        MWL_Matrix::Vector_SetZero(traj.twist_ff);

        MWL_Matrix::MatIdentity3(traj.R_start);
        MWL_Matrix::MatIdentity3(traj.R_goal);
        MWL_Matrix::MatIdentity3(traj.R_ref);
    }

    void MinJerkTimeScaling(float t, float total_time, float &s, float &s_dot)
    {
        if (total_time <= 1.0e-6f)
        {
            s = 1.0f;
            s_dot = 0.0f;
            return;
        }

        const float tau = ClampScalar(t / total_time, 0.0f, 1.0f);
        const float tau2 = tau * tau;
        const float tau3 = tau2 * tau;
        const float tau4 = tau3 * tau;
        const float tau5 = tau4* tau;

        s = 10.0f * tau3 - 15 * tau4 + 6 * tau5;
        s_dot = (30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4) / total_time;
    }

    /**
    * @brief 估计笛卡尔空间位姿轨迹所需的时间，考虑位置增量和姿态增量，并根据给定的线速度和角速度峰值进行计算
    */
    float EstimateCartesianPoseDuration(const float (&p_start)[3], const float (&p_goal)[3],
                                        const float (&R_start)[3][3], const float (&R_goal)[3][3],
                                        float peak_linear_speed, float peak_angular_speed, float min_duration)
    {
        float dp[3];
        float R_start_T[3][3];
        float R_delta[3][3];
        float rotvec_delta[3];

        // 计算位置增量和轴角增量
        MWL_Matrix::Vector_Sub(p_goal, p_start, dp);
        MWL_Matrix::Matrix_Transpose(R_start, R_start_T);
        MWL_Matrix::Matrix_Mul(R_start_T, R_goal, R_delta);
        MWL_SO3::Log3(R_delta, rotvec_delta);

        // 计算位置增量和轴角增量各自范数
        const float pos_distance = MWL_SO3::Norm3(dp);
        const float rot_distance = MWL_SO3::Norm3(rotvec_delta);

        // 估计线速度和角速度所需的时间，并取较大值作为总时间
        float t_pos = 0.0f;
        float t_rot = 0.0f;

        if(peak_linear_speed > 1.0e-6f)
        {
            t_pos = kMinJerkPeakScale * pos_distance / peak_linear_speed;
        }

        if(peak_angular_speed > 1.0e-6f)
        {
            t_rot = kMinJerkPeakScale * rot_distance / peak_angular_speed;
        }

        return MaxScalar(min_duration, MaxScalar(t_pos, t_rot));
    }

    void StartCartesianPoseTrajectory(CartesianPoseTrajectory_t &traj,
                                      const float (&p_start)[3], const float (&p_goal)[3],
                                      const float (&R_start)[3][3], const float (&R_goal)[3][3],
                                      float duration)
    {
        float dp[3];
        float R_start_T[3][3];
        float R_delta[3][3];

        MWL_Matrix::Vector_Copy(p_start, traj.p_start);
        MWL_Matrix::Vector_Copy(p_goal, traj.p_goal);
        MWL_Matrix::Vector_Copy(p_start, traj.p_ref);
        MWL_Matrix::Vector_SetZero(traj.v_ff);

        MWL_Matrix::Matrix_Copy(R_start, traj.R_start);
        MWL_Matrix::Matrix_Copy(R_goal, traj.R_goal);
        MWL_Matrix::Matrix_Copy(R_start, traj.R_ref);
        MWL_Matrix::Vector_SetZero(traj.w_ff);

        MWL_Matrix::Vector_SetZero(traj.twist_ff);

        // 计算位置增量和轴角增量
        MWL_Matrix::Vector_Sub(p_goal, p_start, dp);
        MWL_Matrix::Matrix_Transpose(R_start, R_start_T);
        MWL_Matrix::Matrix_Mul(R_start_T, R_goal, R_delta);
        MWL_SO3::Log3(R_delta, traj.rotvec_total_local);

        traj.elapsed = 0.0f;

        if (duration > 1.0e-6f)
        {
            traj.duration = duration;
        }
        else
        {
            traj.duration = EstimateCartesianPoseDuration(p_start, p_goal, R_start, R_goal,
                    kCartesianTrajPeakLinearSpeed,kCartesianTrajPeakAngularSpeed,kCartesianTrajMinTime);
        }


        if(MWL_SO3::Norm3(dp) < 1.0e-6f && MWL_SO3::Norm3(traj.rotvec_total_local) < 1.0e-6f)
        {
            traj.active = 0U;
        }
        else
        {
            traj.active = 1U;
        }

    }

    void UpdateCartesianPoseTrajectory(CartesianPoseTrajectory_t &traj, float dt)
    {
        // 轨迹未激活，直接返回
        if (!traj.active)
        {
            MWL_Matrix::Vector_SetZero(traj.v_ff);
            MWL_Matrix::Vector_SetZero(traj.w_ff);
            MWL_Matrix::Vector_SetZero(traj.twist_ff);
            return;
        }

        traj.elapsed += dt;
        if (traj.elapsed > traj.duration)
        {
            traj.elapsed = traj.duration;
        }

        float s = 0.0f;
        float s_dot = 0.0f;
        MinJerkTimeScaling(traj.elapsed, traj.duration, s, s_dot);

        /* 位置 &&  线速度前馈 */
        float dp[3];
        MWL_Matrix::Vector_Sub(traj.p_goal,  traj.p_start, dp);


        for (int i = 0; i < 3; i++)
        {
            traj.p_ref[i] = traj.p_start[i] + s * dp[i];
            traj.v_ff[i] = s_dot * dp[i];
        }


        /* 旋转 &&  角速度前馈 */
        float phi_ref_local[3];     // 局部坐标系参考旋转增量
        float w_ff_local[3];        // 局部坐标系前馈角速度
        float R_delta_ref[3][3];    // 参考旋转增量的旋转矩阵表达 局部坐标系

        for (int i = 0; i < 3; i++)
        {
            phi_ref_local[i] = s * traj.rotvec_total_local[i];  // 参考旋转增量的局部表达
            w_ff_local[i] = s_dot * traj.rotvec_total_local[i]; // 前馈角速度的局部表达
        }

        MWL_SO3::Exp3(phi_ref_local, R_delta_ref);
        MWL_Matrix::Matrix_Mul(traj.R_start, R_delta_ref, traj.R_ref);  // 参考旋转矩阵表达
        MWL_Matrix::MatVec_Mul(traj.R_ref, w_ff_local, traj.w_ff);  // 前馈角速度的世界坐标系表达

        /* 组合线速度和角速度前馈 */
        for (int i = 0; i < 6; ++i)
        {
            traj.twist_ff[i] = (i < 3) ? traj.v_ff[i] : traj.w_ff[i - 3];
        }

        if (traj.elapsed >= traj.duration)
        {
            traj.active = 0U;
            MWL_Matrix::Vector_Copy(traj.p_goal, traj.p_ref);
            MWL_Matrix::Matrix_Copy(traj.R_goal, traj.R_ref);

            MWL_Matrix::Vector_SetZero(traj.v_ff);
            MWL_Matrix::Vector_SetZero(traj.w_ff);
            MWL_Matrix::Vector_SetZero(traj.twist_ff);

        }

    }

}