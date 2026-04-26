/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Dev_Custom.c
  * @brief      图传链路数据Dev层，包括0x302、0x304数据接收
  * @note       本文件只涉及数据的接收与搬运，数据解析交给App层，使用RTOS队列实现解耦
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-31-2025     Light            1. done
  *  v1.1.0     Mar-14-2025     Light            2. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include <string.h>
#include "Dev_Custom.h"

/* 变量声明 */
extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart7_rx;

QueueHandle_t xCustom_Queue; // 图传链路自定义控制器队列句柄
QueueHandle_t xRC_Queue; // 图传链路键鼠信息队列句柄
/**
  * @brief          自定义控制器接收数据初始化
  * @param[in]      本函数将会在FreeRTOS中的初始化环节被调用
  * @retval         none
  */
void Custom_Control_Init(void)
{
    /* 串口不定长接收+DMA初始化 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, custom_rx_buf, CUSTOM_QUEUE_ITEM_SIZE);

    /* 消息队列初始化 */
    xCustom_Queue = xQueueCreate(CUSTOM_QUEUE_ITEM_NUM, CUSTOM_QUEUE_ITEM_SIZE); // 0x302
    xRC_Queue = xQueueCreate(RC_QUEUE_ITEM_NUM, CUSTOM_QUEUE_ITEM_SIZE); // 0x304
}

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
QueueHandle_t xGet_Custom_Queue(void)
{
    return xCustom_Queue;
}

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
QueueHandle_t xGet_RC_Queue(void)
{
    return xRC_Queue;
}

/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口7的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
void UART7_Error_Handler(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, custom_rx_buf, CUSTOM_QUEUE_ITEM_SIZE);
    memset(custom_rx_buf, 0, CUSTOM_QUEUE_ITEM_SIZE);
}

/**
  * @brief          串口中断服务函数
  *                 将在HAL_UARTEx_RxEventCallback串口7的空闲中断回调函数中被调用
  * @param[in]      huart: 对应的串口句柄
  *                 Size : 数据长度
  *@retval          none
  */
void UART7_ISR_Handler(UART_HandleTypeDef *huart,uint16_t Size)
{ 
    if (huart->Instance == UART7)
    {
        if (Size <= CUSTOM_QUEUE_ITEM_SIZE || Size <= RC_QUEUE_ITEM_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, custom_rx_buf, CUSTOM_QUEUE_ITEM_SIZE); // 重新开启接收下一次串口中断数据

            /* 将数据写入队列 */
            QueueHandle_t xCustom_Queue = xGet_Custom_Queue();
            Custom_Data_t custom_info_W;

            QueueHandle_t xRC_Queue = xGet_RC_Queue();
            Custom_Data_t RC_info_W;

            memcpy(&custom_info_W, custom_rx_buf, sizeof(Custom_Data_t)); // 将字节数组复制到结构体
            xQueueSendFromISR(xCustom_Queue, &custom_info_W, NULL);

            memcpy(&RC_info_W, custom_rx_buf, sizeof(Custom_Data_t)); // 将字节数组复制到结构体
            xQueueSendFromISR(xRC_Queue, &RC_info_W, NULL);
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart7, custom_rx_buf, CUSTOM_QUEUE_ITEM_SIZE);
            memset(custom_rx_buf, 0, CUSTOM_QUEUE_ITEM_SIZE);
        }
    }
}