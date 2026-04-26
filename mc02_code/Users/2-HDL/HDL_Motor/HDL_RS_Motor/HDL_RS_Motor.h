/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_RS_Motor.h
  * @brief      RS03 电机驱动层头文件
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 基于 MIT 电机公共基类封装 RS03
  *             2. 只保留经典 CAN 标准帧 MIT 协议
  *             3. 包含使能、失能、设置零点、清故障、改 ID 等消息帧
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
 * @brief 灵足 RS03 电机类
 * @note 本类不使用厂商扩展私有协议，只处理经典 CAN 的 MIT 协议收发
 */
class Class_RS_Motor : public Class_MIT_Motor_Base
{
public:
    /**
     * @brief 构造函数
     */
    Class_RS_Motor();

    /**
     * @brief 清除电机故障
     * @param[in] command 清故障命令字，默认 0xFF
     * @retval true  发送成功
     *         false 发送失败
     */
    bool ClearFault(uint8_t command = 0xFFU);

    /**
     * @brief 修改电机 CAN ID
     * @param[in] new_can_id 新的标准帧 CAN ID
     * @retval true  发送成功
     *         false 发送失败
     */
    bool ChangeCanId(uint8_t new_can_id);

    /**
     * @brief 修改主控接收 ID
     * @param[in] new_master_id 新的主控回读 ID
     * @retval true  发送成功
     *         false 发送失败
     */
    bool ChangeMasterId(uint8_t new_master_id);

protected:
    float PositionMin() const override;
    float PositionMax() const override;
    float VelocityMin() const override;
    float VelocityMax() const override;
    float KpMax() const override;
    float KdMax() const override;
    float TorqueMin() const override;
    float TorqueMax() const override;
    uint8_t DecodeCanId(const uint8_t *data) const override;
    uint8_t DecodeErrState(const uint8_t *data) const override;
    uint32_t DecodeFaultCode(const uint8_t *data, uint32_t length) const override;
    void DecodeTemperature(const uint8_t *data, uint32_t length) override;

private:
    /**
     * @brief 发送 RS03 的特殊命令帧
     * @note RS03 的特殊帧格式为前 6 字节 0xFF，最后 2 字节携带命令参数
     */
    bool SendPayloadFrame(uint8_t payload6, uint8_t payload7);
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
