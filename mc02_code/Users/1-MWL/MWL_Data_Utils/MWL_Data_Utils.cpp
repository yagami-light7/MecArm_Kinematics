/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       MWL_Data_Utils.cpp
  * @brief      本文件提供底层通用数据处理模块
  * @note       Middleware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-28-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "MWL_Data_Utils.h"

_Static_assert(sizeof(float) == 4, "Float size must be 4 bytes");

#define DEFINE_UINT8_TO_TYPE_FUNC(TYPE, FUNC_NAME)                        \
void FUNC_NAME(const uint8_t *uint8_data, TYPE *typed_data, int count)   \
{                                                                         \
    for (int i = 0; i < count; i++)                                       \
    {                                                                     \
        memcpy(&typed_data[i], uint8_data + i * sizeof(TYPE), sizeof(TYPE)); \
    }                                                                     \
}

#define DEFINE_TYPE_TO_UINT8_FUNC(TYPE, FUNC_NAME)                          \
void FUNC_NAME(const TYPE *typed_data, uint8_t *uint8_data, int count)     \
{                                                                           \
    for (int i = 0; i < count; i++)                                         \
    {                                                                       \
        memcpy(uint8_data + i * sizeof(TYPE), &typed_data[i], sizeof(TYPE));\
    }                                                                       \
}



/**
 * @brief          将uint8_t数组转化为float数组
 *
 * @param[in]      uint8_data   需要转化的uint8_t数组
 *                 float_data   转化后的float数组
 *                 num_floats   float数组元素个数
 *
 * @retval         none
 */
void uint8_to_float(uint8_t *uint8_data, float *float_data, int num_floats)
{
    for (int i = 0; i < num_floats; i++)
    {
        float_data[i] = *( (float *)(uint8_data + i * sizeof(float)) );
    }
}

//// 生成多个转换函数
//DEFINE_UINT8_TO_TYPE_FUNC(float, uint8_to_float)
//DEFINE_UINT8_TO_TYPE_FUNC(int32_t, uint8_to_int32)
//DEFINE_UINT8_TO_TYPE_FUNC(uint16_t, uint8_to_uint16)
//DEFINE_UINT8_TO_TYPE_FUNC(double, uint8_to_double)
//
//DEFINE_TYPE_TO_UINT8_FUNC(float, float_to_uint8)
//DEFINE_TYPE_TO_UINT8_FUNC(int32_t, int32_to_uint8)
//DEFINE_TYPE_TO_UINT8_FUNC(uint16_t, uint16_to_uint8)