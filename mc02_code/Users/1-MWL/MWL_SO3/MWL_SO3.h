/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_SO3.h
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

#endif

namespace MWL_SO3
{
    float Norm3(const float (&v)[3]);

    void Log3(const float (&R)[3][3], float (&phi)[3]);

    void Exp3(const float (&phi)[3], float (&R)[3][3]);

    void OrientationErrorWorld(const float (&R_des)[3][3], const float (&R_cur)[3][3], float (&e_rot)[3]);
}


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
