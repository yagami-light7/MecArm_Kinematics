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

namespace MWL_Trajectory
{
    // 笛卡尔空间位姿轨迹结构体
    typedef struct
    {
        uint8_t active; // 轨迹激活标志

        float elapsed;  // 轨迹运行时间
        float duration; // 轨迹总时长

        float p_start[3];   //  轨迹起始位置
        float p_goal[3];    //  轨迹目标位置
        float p_ref[3];     // 当前轨迹参考位置
        float v_ff[3];      // 当前轨迹前馈线速度

        float R_start[3][3];    // 轨迹起始姿态
        float R_goal[3][3];     // 轨迹目标姿态
        float R_ref[3][3];      // 当前轨迹参考姿态

        float rotvec_total_local[3];    // 轨迹总旋转增量R_start^T * R_goal的log3局部表达
        float w_ff[3];                  // 当前轨迹前馈角速度

        float twist_ff[6];  // 当前轨迹前馈twist = [v_ff; w_ff]

    } CartesianPoseTrajectory_t;

    void InitCartesianPoseTrajectory(CartesianPoseTrajectory_t &traj);

    void MinJerkTimeScaling(float t, float total_time, float &s, float &s_dot);

    float EstimateCartesianPoseDuration(const float (&p_start)[3], const float (&p_goal)[3],
                                        const float (&R_start)[3][3], const float (&R_goal)[3][3],
                                        float peak_linear_speed, float peak_angular_speed, float min_duration);

    void StartCartesianPoseTrajectory(CartesianPoseTrajectory_t &traj,
                                const float (&p_start)[3], const float (&p_goal)[3],
                                const float (&R_start)[3][3], const float (&R_goal)[3][3],
                                float duration);

    void UpdateCartesianPoseTrajectory(CartesianPoseTrajectory_t &traj, float dt);

}

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
