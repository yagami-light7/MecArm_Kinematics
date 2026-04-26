#pragma once

#include "main.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

//#define DECLARE_UINT8_TO_TYPE_FUNC(TYPE, FUNC_NAME) \
//    void FUNC_NAME(const uint8_t *uint8_data, TYPE *typed_data, int count);


//// 声明函数
//DECLARE_UINT8_TO_TYPE_FUNC(float, uint8_to_float)
//DECLARE_UINT8_TO_TYPE_FUNC(int32_t, uint8_to_int32)
//DECLARE_UINT8_TO_TYPE_FUNC(uint16_t, uint8_to_uint16)
//DECLARE_UINT8_TO_TYPE_FUNC(double, uint8_to_double)

/**
 * @brief          将uint8_t数组转化为float数组
 *
 * @param[in]      uint8_data   需要转化的uint8_t数组
 *                 float_data   转化后的float数组
 *                 num_floats   float数组元素个数
 *
 * @retval         none
 */
extern void uint8_to_float(uint8_t *uint8_data, float *float_data, int num_floats);

#ifdef __cplusplus
}
#endif