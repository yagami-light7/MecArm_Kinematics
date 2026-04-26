#ifndef BSP_USART_H
#define BSP_USART_H

#include "usart.h"
#include "main.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "Dev_Custom.h"
#include "Dev_CV.h"

#define UART_TX_BUFFER_SIZE            256         // 串口TX缓冲区字节长度
#define UART_RX_BUFFER_SIZE            512         // 串口RX缓冲区字节长度

/**
 * @brief UART处理结构体
 */
struct Struct_UART_Manage_Object
{
    UART_HandleTypeDef * huart;
    uint8_t Tx_Buffer[UART_TX_BUFFER_SIZE];
    uint8_t Rx_Buffer[UART_RX_BUFFER_SIZE];
    uint16_t Rx_Data_Size;
};

/**
 * @brief          裁判系统串口发送函数
 * @param[in]      none
 * @retval         none
 */

extern void Referee_Tx_Dma_Enable(uint8_t *data, uint16_t len);

/**
 * @brief          裁判系统串口初始化发送函数
 * @param[in]      none
 * @retval         none
 */
extern void Referee_UART_Dma_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

#endif
