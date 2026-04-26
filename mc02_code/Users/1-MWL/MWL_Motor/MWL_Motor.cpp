/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_Motor.cpp
  * @brief      电机中间件公共工具函数实现
  * @note       Middleware Layer 中间件层
  *             1. 统一实现电机协议中常见的映射函数
  *             2. 统一实现不同字节序的读写函数
  *             3. 给 HDL 层提供纯数据处理能力
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "MWL_Motor.h"

#include <string.h>

/**
  * @brief          对输入值进行限幅
  * @param[in]      value       输入值
  *                 min_value   最小值
  *                 max_value   最大值
  * @retval         限幅后的结果
  */
float MWL_Motor_Clamp(float value, float min_value, float max_value)
{
    assert_param(min_value < max_value);

    if (min_value >= max_value)
    {
        return value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    if (value > max_value)
    {
        return max_value;
    }

    return value;
}

/**
  * @brief          将浮点数映射为无符号整数
  * @param[in]      value       输入浮点数
  *                 min_value   量程最小值
  *                 max_value   量程最大值
  *                 bits        目标位宽
  * @retval         映射后的无符号整数
  * @note           该函数会先执行限幅，避免超量程输入导致溢出
  */
uint16_t MWL_Motor_Float_To_Uint(float value, float min_value, float max_value, uint32_t bits)
{
    const float span = max_value - min_value;
    const float clamped = MWL_Motor_Clamp(value, min_value, max_value);
    const uint32_t full_scale = (uint32_t)((1UL << bits) - 1UL);

    return (uint16_t)((clamped - min_value) * (float)full_scale / span);
}

/**
  * @brief          将无符号整数反解为浮点数
  */
float MWL_Motor_Uint_To_Float(uint32_t value, float min_value, float max_value, uint32_t bits)
{
    const float span = max_value - min_value;
    const uint32_t full_scale = (uint32_t)((1UL << bits) - 1UL);

    return ((float)value) * span / (float)full_scale + min_value;
}

/**
  * @brief          从字节数组中读取大端 16 位数值
  */
uint16_t MWL_Motor_Read_BE_U16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

/**
  * @brief          向字节数组写入大端 16 位数值
  */
void MWL_Motor_Write_BE_U16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);
}

/**
  * @brief          向字节数组写入小端 16 位数值
  */
void MWL_Motor_Write_LE_U16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)(value >> 8);
}

/**
  * @brief          从字节数组中读取小端 32 位数值
  */
uint32_t MWL_Motor_Read_LE_U32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

/**
  * @brief          按小端格式写入单精度浮点数
  */
void MWL_Motor_Write_LE_Float(uint8_t *data, float value)
{
    memcpy(data, &value, sizeof(value));
}

/**
  * @brief          按小端格式读取单精度浮点数
  */
float MWL_Motor_Read_LE_Float(const uint8_t *data)
{
    float value = 0.0f;
    memcpy(&value, data, sizeof(value));
    return value;
}

/**
  * @brief          将 FDCAN DLC 编码转换为真实字节长度
  * @param[in]      dlc FDCAN 数据长度码
  * @retval         实际的字节长度
  * @note           当前项目只用到经典 CAN 0~8 字节，因此这里只处理到 8 字节
  */
uint32_t MWL_Motor_Dlc_To_Length(uint32_t dlc)
{
    switch (dlc)
    {
        case FDCAN_DLC_BYTES_0:
            return 0U;
        case FDCAN_DLC_BYTES_1:
            return 1U;
        case FDCAN_DLC_BYTES_2:
            return 2U;
        case FDCAN_DLC_BYTES_3:
            return 3U;
        case FDCAN_DLC_BYTES_4:
            return 4U;
        case FDCAN_DLC_BYTES_5:
            return 5U;
        case FDCAN_DLC_BYTES_6:
            return 6U;
        case FDCAN_DLC_BYTES_7:
            return 7U;
        case FDCAN_DLC_BYTES_8:
            return 8U;
        default:
            return 8U;
    }
}
