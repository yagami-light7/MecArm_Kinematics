/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_CRC.h
  * @brief      CRC8 与 CRC16 公共校验接口
  * @note       Middleware Layer 中间件层
  *             1. 提供新框架统一使用的 CRC8/CRC16 计算接口
  *             2. 保留旧框架同名函数，方便旧模块平滑迁移
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-25-2026     Codex           1. migrate from legacy framework
  *
  @verbatim
  ==============================================================================
  * 本文件统一收口 CRC 相关逻辑。新代码应优先使用 MWL_CRC_* 接口，
  * 旧框架函数名仅作为兼容层保留。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief CRC8 默认初值
 */
#define MWL_CRC8_INIT_VALUE   0xFFU

/**
 * @brief CRC16 默认初值
 */
#define MWL_CRC16_INIT_VALUE  0xFFFFU

/**
 * @brief 计算 CRC8 校验值
 * @param message 输入数据指针
 * @param length  数据长度
 * @param init    CRC8 初值
 * @return CRC8 校验值
 */
uint8_t MWL_CRC8_GetCheckSum(const uint8_t *message, uint32_t length, uint8_t init);

/**
 * @brief 校验一段带 CRC8 尾字节的数据
 * @param message 输入数据指针
 * @param length  数据总长度，包含最后 1 字节 CRC8
 * @return true 校验成功
 * @return false 校验失败
 */
bool MWL_CRC8_Verify(const uint8_t *message, uint32_t length);

/**
 * @brief 计算并写入 CRC8 到数据尾部
 * @param message 输入输出数据指针
 * @param length  数据总长度，包含最后 1 字节 CRC8
 */
void MWL_CRC8_Append(uint8_t *message, uint32_t length);

/**
 * @brief 计算 CRC16 校验值
 * @param message 输入数据指针
 * @param length  数据长度
 * @param init    CRC16 初值
 * @return CRC16 校验值
 */
uint16_t MWL_CRC16_GetCheckSum(const uint8_t *message, uint32_t length, uint16_t init);

/**
 * @brief 校验一段带 CRC16 尾字节的数据
 * @param message 输入数据指针
 * @param length  数据总长度，包含最后 2 字节 CRC16
 * @return true 校验成功
 * @return false 校验失败
 */
bool MWL_CRC16_Verify(const uint8_t *message, uint32_t length);

/**
 * @brief 计算并写入 CRC16 到数据尾部
 * @param message 输入输出数据指针
 * @param length  数据总长度，包含最后 2 字节 CRC16
 */
void MWL_CRC16_Append(uint8_t *message, uint32_t length);

/**
 * @brief 旧框架 CRC8 接口兼容层
 */
uint8_t get_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);
void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

/**
 * @brief 旧框架 CRC16 接口兼容层
 */
uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);
void append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#ifdef __cplusplus
}
#endif
