/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       CallBack_FDCAN.h
  * @brief      电机 FDCAN 回调入口
  * @note       Application Layer
  *             1. 本层只放 HAL FDCAN 的电机接收回调
  *             2. 回调读取 FIFO 后调用 HDL_Motor_Rx 完成反馈分发
  *             3. 电机对象初始化、总线绑定和控制命令下发仍由任务层完成
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "fdcan.h"

/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
