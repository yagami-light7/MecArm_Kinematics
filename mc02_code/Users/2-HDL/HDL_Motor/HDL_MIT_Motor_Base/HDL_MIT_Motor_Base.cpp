/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_MIT_Motor_Base.cpp
  * @brief      MIT 协议电机公共基类实现
  * @note       Hardware Driver Layer 硬件驱动层
  *             本文件实现 MIT 电机共有的数据打包、特殊控制帧发送、
  *             反馈匹配和公共字段解析逻辑。
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#include "HDL_MIT_Motor_Base.h"
#include <string.h>

/**
  * @brief          构造函数
  * @param[in]      brand 电机品牌
  * @retval         none
  * @note           创建对象时先把状态清零，后续再由 AttachPort 绑定总线参数
  */
Class_MIT_Motor_Base::Class_MIT_Motor_Base(MWL_Motor_Brand_e brand)
    : FDCAN_Manage_Object_(NULL), master_id_(kDefaultMasterId)
{
    memset(&state_, 0, sizeof(state_));
    state_.brand = brand;
    state_.model = MWL_MOTOR_MODEL_UNKNOWN;
    state_.run_mode = MWL_MOTOR_RUN_MODE_DISABLED;
}

/**
  * @brief          初始化电机驱动对象
  */
void Class_MIT_Motor_Base::Init(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint8_t can_id, uint16_t master_id)
{
    FDCAN_Manage_Object_ = FDCAN_Manage_Object;
    state_.can_id = can_id;
    master_id_ = master_id;
}

void Class_MIT_Motor_Base::SetMotorModel(MWL_Motor_Model_e model)
{
    state_.model = model;
}

/**
  * @brief          使能电机
  * @retval         true  发送成功
  *                 false 发送失败
  */
bool Class_MIT_Motor_Base::Enable()
{
    if (SendSpecialFrame(0xFCU))
    {
        state_.run_mode = MWL_MOTOR_RUN_MODE_MIT;
        return true;
    }

    return false;
}

/**
  * @brief          失能电机
  */
bool Class_MIT_Motor_Base::Disable()
{
    if (SendSpecialFrame(0xFDU))
    {
        state_.run_mode = MWL_MOTOR_RUN_MODE_DISABLED;
        return true;
    }

    return false;
}

/**
  * @brief          设置当前位置为零点
  */
bool Class_MIT_Motor_Base::SetZero()
{
    return SendSpecialFrame(0xFEU);
}

/**
  * @brief          下发 MIT 五参数控制帧
  * @param[in]      position 目标位置
  *                 velocity 目标速度
  *                 kp       刚度
  *                 kd       阻尼
  *                 torque   前馈力矩
  * @retval         true  发送成功
  *                 false 发送失败
  * @note           量程由子类提供，因此同一套代码可复用于不同品牌电机
  */
bool Class_MIT_Motor_Base::SendMITCommand(float position, float velocity, float kp, float kd, float torque)
{
    uint8_t tx_data[8] = {};
    const uint16_t pos_uint = MWL_Motor_Float_To_Uint(position, PositionMin(), PositionMax(), 16U);
    const uint16_t vel_uint = MWL_Motor_Float_To_Uint(velocity, VelocityMin(), VelocityMax(), 12U);
    const uint16_t kp_uint = MWL_Motor_Float_To_Uint(kp, KpMin(), KpMax(), 12U);
    const uint16_t kd_uint = MWL_Motor_Float_To_Uint(kd, KdMin(), KdMax(), 12U);
    const uint16_t tor_uint = MWL_Motor_Float_To_Uint(torque, TorqueMin(), TorqueMax(), 12U);

    if (FDCAN_Manage_Object_ == NULL || state_.can_id == 0U)
    {
        return false;
    }

    MWL_Motor_Write_BE_U16(&tx_data[0], pos_uint);
    tx_data[2] = (uint8_t)(vel_uint >> 4);
    tx_data[3] = (uint8_t)(((vel_uint & 0x0FU) << 4) | (kp_uint >> 8));
    tx_data[4] = (uint8_t)(kp_uint & 0xFFU);
    tx_data[5] = (uint8_t)(kd_uint >> 4);
    tx_data[6] = (uint8_t)(((kd_uint & 0x0FU) << 4) | (tor_uint >> 8));
    tx_data[7] = (uint8_t)(tor_uint & 0xFFU);

    state_.last_command_kp = kp;
    state_.last_command_kd = kd;
    state_.run_mode = MWL_MOTOR_RUN_MODE_MIT;

    return FDCAN_Send_Std_Data(FDCAN_Manage_Object_, state_.can_id, tx_data, sizeof(tx_data));
}

/**
  * @brief          解析一帧反馈数据
  * @retval         true  该帧属于当前电机并完成了解析
  *                 false 该帧不属于当前电机
  */
bool Class_MIT_Motor_Base::ParseFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length)
{
    if (!MatchesFeedback(header, data, length))
    {
        return false;
    }

    UpdateMITState(data);
    DecodeTemperature(data, length);
    state_.online = 1U;
    state_.update_count++;

    return true;
}

/**
  * @brief          判断反馈帧是否属于当前电机
  * @note           先看是否是标准帧，再看是否回给当前主机，最后再比较电机 ID
  */
bool Class_MIT_Motor_Base::MatchesFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length) const
{
    if (data == NULL || length < 8U)
    {
        return false;
    }

    // 检查标准帧 检查主控ID
    if (header.IdType != FDCAN_STANDARD_ID || header.Identifier != master_id_)
    {
        return false;
    }

    // 判断电机注册表中电机id是否与反馈帧id相匹配
    return (DecodeCanId(data) == state_.can_id);
}

/**
  * @brief          获取电机状态结构体
  */
const MWL_Motor_State_t &Class_MIT_Motor_Base::GetState() const
{
    return state_;
}

/**
  * @brief          获取当前电机 CAN ID
  */
uint8_t Class_MIT_Motor_Base::GetCanId() const
{
    return state_.can_id;
}

/**
  * @brief          默认 Kp 下限
  */
float Class_MIT_Motor_Base::KpMin() const
{
    return 0.0f;
}

/**
  * @brief          默认 Kd 下限
  */
float Class_MIT_Motor_Base::KdMin() const
{
    return 0.0f;
}

/**
  * @brief          发送 MIT 协议特殊控制帧
  * @param[in]      tail_code 尾字节命令码
  * @retval         true  发送成功
  *                 false 发送失败
  * @note           典型命令包括 0xFC 使能、0xFD 失能、0xFE 标零
  */
bool Class_MIT_Motor_Base::SendSpecialFrame(uint8_t tail_code)
{
    uint8_t tx_data[8];

    if (FDCAN_Manage_Object_ == NULL || state_.can_id == 0U)
    {
        return false;
    }

    memset(tx_data, 0xFF, sizeof(tx_data));
    tx_data[7] = tail_code;

    return FDCAN_Send_Std_Data(FDCAN_Manage_Object_, state_.can_id, tx_data, sizeof(tx_data));
}

/**
  * @brief          解析 MIT 回读中的公共字段
  * @note           位置、速度、力矩三项的量程解释由子类提供
  */
void Class_MIT_Motor_Base::UpdateMITState(const uint8_t *data)
{
    state_.err_state = DecodeErrState(data);
    state_.raw_position = MWL_Motor_Read_BE_U16(&data[1]);
    state_.raw_velocity = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    state_.raw_torque = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);
    state_.position = MWL_Motor_Uint_To_Float(state_.raw_position, PositionMin(), PositionMax(), 16U);
    state_.velocity = MWL_Motor_Uint_To_Float(state_.raw_velocity, VelocityMin(), VelocityMax(), 12U);
    state_.torque = MWL_Motor_Uint_To_Float(state_.raw_torque, TorqueMin(), TorqueMax(), 12U);
    state_.fault_code = DecodeFaultCode(data, 8U);
}

/**
  * @brief          默认从低 4 位中解析电机 ID
  * @note           该格式适用于达妙类 MIT 回读，RS 会在子类中覆写
  */
uint8_t Class_MIT_Motor_Base::DecodeCanId(const uint8_t *data) const
{
    return (uint8_t)(data[0] & 0x0FU);
}

/**
  * @brief          默认从高 4 位中解析错误状态
  */
uint8_t Class_MIT_Motor_Base::DecodeErrState(const uint8_t *data) const
{
    return (uint8_t)(data[0] >> 4);
}

/**
  * @brief          默认故障码解析
  * @note           若品牌没有单独故障字段，则默认返回 err_state
  */
uint32_t Class_MIT_Motor_Base::DecodeFaultCode(const uint8_t *data, uint32_t length) const
{
    (void)data;
    (void)length;
    return state_.err_state;
}
