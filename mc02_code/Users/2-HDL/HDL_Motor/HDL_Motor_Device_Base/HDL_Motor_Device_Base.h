/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Motor_Device_Base.h
  * @brief      电机设备统一接口头文件
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 为不同品牌电机对象提供统一的反馈解析入口
  *             2. 物理 CAN 总线只依赖本接口，不再关心具体品牌类型
  *             3. 任意挂载到总线上的设备对象都应实现 ParseFeedback 接口
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "HAL_FDCAN.h"

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

class Class_Motor_Device_Base
{
public:
    /**
     * @brief 虚析构函数
     */
    virtual ~Class_Motor_Device_Base() {}

    /**
     * @brief 解析一帧反馈数据
     * @param[in] header FDCAN 接收头
     *            data   数据区
     *            length 数据长度
     * @retval true  当前帧属于本设备并完成解析
     *         false 当前帧不属于本设备
     */
    virtual bool ParseFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length) = 0;
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
