/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_FDCAN.h
  * @brief      FDCAN 外设再封装
  * @note       Hardware Abstract Layer 硬件抽象层
  *             1. 采用与 HAL_USART 相同的“管理对象 + 接口函数”风格
  *             2. 统一标准帧、扩展帧发送入口
  *             3. 统一接收读取入口，供上层电机驱动复用
  *             4. 将原 Board_FDCAN 的总线初始化与发送接口收敛到 HAL 层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *  V1.1.0     Apr-10-2026     Codex           2. merge board fdcan apis
  *
  @verbatim
  ==============================================================================
  * 本文件只负责 FDCAN 的收发抽象与基础初始化，不处理具体协议字段含义。
  * 电机品牌协议解析应放在 2-HDL 层实现。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FDCAN 管理对象
 */
typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;               /*!< 绑定的 FDCAN 外设句柄 */
    uint32_t rx_fifo;                          /*!< 默认接收 FIFO */
} FDCAN_Manage_Object_t;

/**
 * @brief 变量外部声明
 */
extern FDCAN_Manage_Object_t FDCAN1_Manage_Object;
extern FDCAN_Manage_Object_t FDCAN2_Manage_Object;
extern FDCAN_Manage_Object_t FDCAN3_Manage_Object;

/**
 * @brief          FDCAN 总线初始化
 * @param[in]      none
 * @retval         none
 */
extern void FDCAN_Filter_Init(void);

/**
 * @brief          FDCAN 管理对象初始化
 * @param[in]      FDCAN_Manage_Object FDCAN 管理对象
 *                 hfdcan              FDCAN 外设句柄
 *                 rx_fifo             默认接收 FIFO
 * @retval         none
 */
extern void FDCAN_Init(FDCAN_Manage_Object_t *FDCAN_Manage_Object, FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo);

/**
 * @brief          发送标准帧数据
 * @param[in]      FDCAN_Manage_Object FDCAN 管理对象
 *                 identifier          标准帧 ID
 *                 data                数据区指针
 *                 length              数据长度
 * @retval         true  发送成功
 *                 false 发送失败
 */
extern bool FDCAN_Send_Std_Data(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint16_t identifier, uint8_t *data, uint32_t length);

/**
 * @brief          发送扩展帧数据
 */
extern bool FDCAN_Send_Ext_Data(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint32_t identifier, uint8_t *data, uint32_t length);

/**
 * @brief          从默认 FIFO 读取一帧消息
 */
extern bool FDCAN_Read_Data(FDCAN_Manage_Object_t *FDCAN_Manage_Object, FDCAN_RxHeaderTypeDef *header, uint8_t *data);

/**
 * @brief          从指定 FIFO 读取一帧消息
 */
extern bool FDCAN_Read_Data_From_FIFO(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint32_t fifo, FDCAN_RxHeaderTypeDef *header, uint8_t *data);

#ifdef __cplusplus
}
#endif
