/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       
  * @brief      
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-2-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

#include "Dev_Referee.h"

/* 变量声明 */
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern unpack_data_t referee_unpack_obj;

static uint8_t referee_rx_buf[USART_RX_BUF_LENGHT];
//QueueHandle_t xReferee_Queue;

/**
  * @brief          裁判系统初始化
  * @param[in]      本函数将会在FreeRTOS中的初始化环节被调用
  * @retval         none
  */
void Referee_Init(void)
{
    /* 串口不定长接收+DMA初始化 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buf, sizeof(referee_rx_buf));

//    /* 消息队列初始化 */
//    xReferee_Queue = xQueueCreate(REFEREE_QUEUE_ITEM_NUM, REFEREE_QUEUE_ITEM_SIZE);
}

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
//QueueHandle_t xGet_Referee_Queue(void)
//{
//    return xReferee_Queue;
//}


/**
  * @brief          串口中断服务函数
  *                 将在HAL_UARTEx_RxEventCallback串口10的空闲中断回调函数中被调用
  * @param[in]      huart: 对应的串口句柄
  *                 Size : 数据长度
  *@retval          none
  */
void UART10_ISR_Handler(UART_HandleTypeDef *huart,uint16_t Size)
{
    if (huart->Instance == USART10)
    {
        static uint16_t this_time_rx_len = 0;
        this_time_rx_len = sizeof(referee_rx_buf) - __HAL_DMA_GET_COUNTER(huart10.hdmarx);

        if (Size <= sizeof(referee_rx_buf))
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buf, sizeof(referee_rx_buf)); // 重新开启接收下一次串口中断数据
//            /* 将数据写入队列 */
//            QueueHandle_t xReferee_Queue = xGet_Referee_Queue();
//            REFEREE_QUEUE_ITEM_TYPE referee_info_W;
//
//            memcpy(&referee_info_W, referee_rx_buf, REFEREE_QUEUE_ITEM_SIZE); // 将字节数组复制
//            xQueueSendFromISR(xReferee_Queue, &referee_info_W, NULL);

            fifo_s_puts(&referee_fifo, (char*)referee_rx_buf, this_time_rx_len);
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buf, sizeof(referee_rx_buf));
            memset(referee_rx_buf, 0, sizeof(referee_rx_buf));
        }
    }
}


/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口10的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
void UART10_Error_Handler(void)
{
    __HAL_UNLOCK(&huart10);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, referee_rx_buf, sizeof(referee_rx_buf));
    memset(referee_rx_buf, 0, sizeof(referee_rx_buf));
}