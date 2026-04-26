/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Damiao_Motor.cpp
  * @brief      DM4310 电机驱动层实现
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 基于 MIT 电机公共基类实现 DM4310 参数量程
  *             2. 保持与项目原有 HDL 模块一致的注释与函数风格
  *             3. 达妙电机特殊帧仍复用基类提供的 MIT 控制接口
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *  V1.1.0     Apr-10-2026     Codex           2. align style with project
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HDL_Damiao_Motor.h"

/**
  * @brief          构造函数
  * @retval         none
  * @note           在公共基类中完成品牌、型号与默认状态初始化
  */
Class_Damiao_Motor::Class_Damiao_Motor()
    : Class_MIT_Motor_Base(MWL_MOTOR_BRAND_DAMIAO)
{
}

/**
  * @brief DM4310 位置量程下限
  */
float Class_Damiao_Motor::PositionMin() const
{
    return -12.5f;
}

/**
  * @brief DM4310 位置量程上限
  */
float Class_Damiao_Motor::PositionMax() const
{
    return 12.5f;
}

/**
  * @brief DM4310 速度量程下限
  */
float Class_Damiao_Motor::VelocityMin() const
{
    return -30.0f;
}

/**
  * @brief DM4310 速度量程上限
  */
float Class_Damiao_Motor::VelocityMax() const
{
    return 30.0f;
}

/**
  * @brief DM4310 刚度量程上限
  */
float Class_Damiao_Motor::KpMax() const
{
    return 500.0f;
}

/**
  * @brief DM4310 阻尼量程上限
  */
float Class_Damiao_Motor::KdMax() const
{
    return 5.0f;
}

/**
  * @brief DM4310 力矩量程下限
  */
float Class_Damiao_Motor::TorqueMin() const
{
    return -10.0f;
}

/**
  * @brief DM4310 力矩量程上限
  */
float Class_Damiao_Motor::TorqueMax() const
{
    return 10.0f;
}

/**
  * @brief          解析温度字段
  * @param[in]      data   回读数据区
  *                 length 数据长度
  * @retval         none
  * @note           达妙回读中 Byte6 和 Byte7 分别可看作 MOS 与电机温度
  */
void Class_Damiao_Motor::DecodeTemperature(const uint8_t *data, uint32_t length)
{
    if (length < 8U)
    {
        return;
    }

    state_.temperature = (float)data[6];
    state_.auxiliary_temperature = (float)data[7];
    state_.raw_temperature = (uint16_t)(((uint16_t)data[6] << 8) | data[7]);
}
