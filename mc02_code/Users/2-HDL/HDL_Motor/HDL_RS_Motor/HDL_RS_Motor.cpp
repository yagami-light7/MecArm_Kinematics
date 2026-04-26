/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_RS_Motor.cpp
  * @brief      RS03 电机驱动层实现
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 基于 MIT 电机公共基类实现 RS03 参数量程
  *             2. 兼容 RS03 回读帧中 Byte0 直接表示 CAN ID 的格式
  *             3. 只使用经典 CAN 标准帧，不接入扩展私有协议
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *  V1.1.0     Apr-10-2026     Codex           2. align style with project
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HDL_RS_Motor.h"

#include "MWL_Motor.h"

#include <string.h>

/**
  * @brief          构造函数
  * @retval         none
  * @note           在公共基类中完成品牌、型号与初始状态设置
  */
Class_RS_Motor::Class_RS_Motor()
    : Class_MIT_Motor_Base(MWL_MOTOR_BRAND_RS)
{
}

/**
  * @brief          清除电机故障
  * @param[in]      command 清故障命令字
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool Class_RS_Motor::ClearFault(uint8_t command)
{
    return SendPayloadFrame(command, 0xFBU);
}

/**
  * @brief          修改电机 CAN ID
  * @param[in]      new_can_id 新的 CAN ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool Class_RS_Motor::ChangeCanId(uint8_t new_can_id)
{
    if (SendPayloadFrame(new_can_id, 0xFAU))
    {
        state_.can_id = new_can_id;
        return true;
    }

    return false;
}

/**
  * @brief          修改主控接收 ID
  * @param[in]      new_master_id 新的主控 ID
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool Class_RS_Motor::ChangeMasterId(uint8_t new_master_id)
{
    if (SendPayloadFrame(new_master_id, 0x01U))
    {
        master_id_ = new_master_id;
        return true;
    }

    return false;
}

/**
  * @brief RS03 位置量程下限
  */
float Class_RS_Motor::PositionMin() const
{
    return -12.57f;
}

/**
  * @brief RS03 位置量程上限
  */
float Class_RS_Motor::PositionMax() const
{
    return 12.57f;
}

/**
  * @brief RS03 速度量程下限
  */
float Class_RS_Motor::VelocityMin() const
{
    if (state_.model == MWL_MOTOR_MODEL_EL05)
    {
        return -50.0f;
    }

    return -20.0f;
}

/**
  * @brief RS03 速度量程上限
  */
float Class_RS_Motor::VelocityMax() const
{
    if (state_.model == MWL_MOTOR_MODEL_EL05)
    {
        return 50.0f;
    }

    return 20.0f;
}

/**
  * @brief RS03 刚度量程上限
  */
float Class_RS_Motor::KpMax() const
{
    if (state_.model == MWL_MOTOR_MODEL_EL05)
    {
        return 500.0f;
    }

    return 5000.0f;
}

/**
  * @brief RS03 阻尼量程上限
  */
float Class_RS_Motor::KdMax() const
{
    if (state_.model == MWL_MOTOR_MODEL_EL05)
    {
        return 5.0f;
    }

    return 100.0f;
}

/**
  * @brief RS03 力矩量程下限
  */
float Class_RS_Motor::TorqueMin() const
{
    if (state_.model == MWL_MOTOR_MODEL_EL05)
    {
        return -6.0f;
    }

    return -60.0f;
}

/**
  * @brief RS03 力矩量程上限
  */
float Class_RS_Motor::TorqueMax() const
{
    if (state_.model == MWL_MOTOR_MODEL_EL05)
    {
        return 6.0f;
    }

    return 60.0f;
}

/**
  * @brief          从回读帧中解析电机 ID
  * @param[in]      data 回读数据区
  * @retval         当前电机反馈 ID
  * @note           RS03 的 Byte0 直接是完整 CAN ID，不和错误码共用同一字节
  */
uint8_t Class_RS_Motor::DecodeCanId(const uint8_t *data) const
{
    return data[0];
}

/**
  * @brief          从回读帧中解析错误状态
  * @param[in]      data 回读数据区
  * @retval         错误状态
  * @note           当前使用的 RS03 MIT 标准反馈中未单独提供 err_state 字段
  */
uint8_t Class_RS_Motor::DecodeErrState(const uint8_t *data) const
{
    (void)data;
    return 0U;
}

/**
  * @brief          从回读帧中解析故障码
  * @param[in]      data   回读数据区
  *                 length 数据长度
  * @retval         故障码
  * @note           经典 CAN MIT 回读中未读到单独故障字，因此此处返回 0
  */
uint32_t Class_RS_Motor::DecodeFaultCode(const uint8_t *data, uint32_t length) const
{
    (void)data;
    (void)length;
    return 0U;
}

/**
  * @brief          解析温度字段
  * @param[in]      data   回读数据区
  *                 length 数据长度
  * @retval         none
  * @note           RS03 温度按 0.1 摄氏度分辨率回读
  */
void Class_RS_Motor::DecodeTemperature(const uint8_t *data, uint32_t length)
{
    if (length < 8U)
    {
        return;
    }

    state_.raw_temperature = MWL_Motor_Read_BE_U16(&data[6]);
    state_.temperature = (float)state_.raw_temperature * 0.1f;
    state_.auxiliary_temperature = 0.0f;
}

/**
  * @brief          发送 RS03 特殊命令帧
  * @param[in]      payload6 第 7 字节载荷
  *                 payload7 第 8 字节载荷
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool Class_RS_Motor::SendPayloadFrame(uint8_t payload6, uint8_t payload7)
{
    uint8_t tx_data[8];

    if (FDCAN_Manage_Object_ == NULL || state_.can_id == 0U)
    {
        return false;
    }

    memset(tx_data, 0xFF, sizeof(tx_data));
    tx_data[6] = payload6;
    tx_data[7] = payload7;

    return FDCAN_Send_Std_Data(FDCAN_Manage_Object_, state_.can_id, tx_data, sizeof(tx_data));
}
