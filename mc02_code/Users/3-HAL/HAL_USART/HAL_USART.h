/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_USART.h
  * @brief      USART外设再封装
  * @note       Hardware Abstract Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-29-2025     Light            1. done
  *  V1.1.0     Apr-13-2026     Codex            1. add DR16 double DMA buffer APIs
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考中科大2024年工程机器人电控代码开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

#pragma once

/**
 * @brief 头文件
 */
#include <stdbool.h>
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

/**
 * @brief 宏定义
 */
#define UART_TX_BUFFER_SIZE     256U    // 串口TX缓冲区字节长度
#define UART_RX_BUFFER_SIZE     512U    // 串口RX缓冲区字节长度
#define QUEUE_RX_ITEM_NUM       3U      // 接收队列中数据段数目_注意不要超出堆栈configTOTAL_HEAP_SIZE

#define CUSTOM_ROBOT_DATA_SIZE  30U
#define REMOTE_ROBOT_DATA_SIZE  12U
#define VT_RC_ROBOT_DATA_SIZE   21U

#define DR16_FRAME_LENGTH       18U     // DR16单帧有效字节长度
#define DR16_DMA_BUFFER_SIZE    36U     // DR16单个DMA缓冲区长度，需大于单帧长度以等待空闲中断

/**
 * @brief 结构体
 */
typedef struct
{
    UART_HandleTypeDef *huart;                          // 串口句柄
    uint8_t tx_buffer[UART_TX_BUFFER_SIZE];             // 通用发送缓冲区
    uint8_t rx_buffer[UART_RX_BUFFER_SIZE];             // 通用接收缓冲区
    uint16_t rx_data_size;                              // 当前接收数据长度
} UART_Manage_Object_t;

/**
 * @brief 变量外部声明
 */
extern UART_Manage_Object_t UART5_Manage_Object;
extern UART_Manage_Object_t UART7_Manage_Object;
extern UART_Manage_Object_t UART10_Manage_Object;

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

/**
* @brief          UART串口初始化
* @param[in]      UART_Manage_Object   串口实例对象
*                 rx_data_size         接收数据长度
* @retval         none
*/
extern void UART_Init(UART_Manage_Object_t *UART_Manage_Object, uint16_t rx_data_size);

/**
 * @brief          UART使用空闲中断+双DMA缓冲区启动接收
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 rx_buffer_0          DMA缓冲区0
 *                 rx_buffer_1          DMA缓冲区1
 *                 rx_data_size         单个DMA缓冲区长度
 * @retval         true                 启动成功
 *                 false                启动失败
 */
extern bool UART_Init_Double_Buffer(UART_Manage_Object_t *UART_Manage_Object, uint8_t *rx_buffer_0,
                                    uint8_t *rx_buffer_1, uint16_t rx_data_size);

/**
 * @brief          UART发送数据
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 data                 数据
 *                 length               数据长度
 * @retval         发送结果
 */
extern bool UART_Send_Data(UART_Manage_Object_t *UART_Manage_Object, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif
