#ifndef ENGINEERING_DEV_REFEREE_H
#define ENGINEERING_DEV_REFEREE_H

#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "UC_Protocol.h"
#include "UC_Referee.h"
#include "Sp_FIFO.h"

#define USART_RX_BUF_LENGHT      512
#define REFEREE_FIFO_BUF_LENGTH  1024

//#define REFEREE_QUEUE_ITEM_NUM    50   // 消息队列中存放的数据个数
//#define REFEREE_QUEUE_ITEM_SIZE  sizeof(_referee_buf_) // 消息队列中存放的数据大小
//#define REFEREE_QUEUE_ITEM_TYPE  typeof(_referee_buf_) // 消息队列中存放的数据类型
//
///* 定义长度为64字节的缓冲区作为队列中的数据 */
//static uint8_t _referee_buf_[USART_RX_BUF_LENGHT]; // Only Example 模板数组

#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief          裁判系统初始化
  * @param[in]      本函数将会在FreeRTOS中的初始化环节被调用
  * @retval         none
  */
extern void Referee_Init(void);

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
extern QueueHandle_t xGet_Referee_Queue(void);

/**
  * @brief          串口中断服务函数
  *                 将在HAL_UARTEx_RxEventCallback串口10的空闲中断回调函数中被调用
  * @param[in]      huart: 对应的串口句柄
  *                 Size : 数据长度
  *@retval          none
  */
extern void UART10_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size);

/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口10的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
extern void UART10_Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
