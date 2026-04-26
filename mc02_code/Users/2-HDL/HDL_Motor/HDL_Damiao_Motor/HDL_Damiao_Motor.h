/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Damiao_Motor.h
  * @brief      DM4310 电机驱动层头文件
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 基于 MIT 电机公共基类封装 DM4310
  *             2. 负责提供 DM4310 的控制量程和温度解释
  *             3. 发送使能、失能、设置零点等公共 MIT 控制由基类实现
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *  V1.1.0     Apr-10-2026     Codex           2. align style with project
 ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "HDL_MIT_Motor_Base.h"

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

/**
 * @brief 达妙 DM4310 电机类
 */
class Class_Damiao_Motor : public Class_MIT_Motor_Base
{
public:
    /**
     * @brief 构造函数
     */
    Class_Damiao_Motor();

protected:
    float PositionMin() const override;
    float PositionMax() const override;
    float VelocityMin() const override;
    float VelocityMax() const override;
    float KpMax() const override;
    float KdMax() const override;
    float TorqueMin() const override;
    float TorqueMax() const override;
    void DecodeTemperature(const uint8_t *data, uint32_t length) override;
};

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
