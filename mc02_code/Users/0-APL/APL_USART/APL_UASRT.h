/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       APL_USART.cpp
  * @brief      串口回调函数
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-29-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"


/**
 * @brief 宏定义
 */
#define UART_TX_BUFFER_SIZE     256     // 串口TX缓冲区字节长度
#define UART_RX_BUFFER_SIZE     512     // 串口RX缓冲区字节长度
#define QUEUE_RX_ITEM_NUM       30      // 接收队列中数据段数目


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