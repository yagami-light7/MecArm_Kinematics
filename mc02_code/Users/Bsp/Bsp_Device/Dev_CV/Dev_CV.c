/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       App_CV_Control.c
  * @brief      Light
  * @note       完成与上位机进行通信获取视觉下发的编码值
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-12-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "Dev_CV.h"
#include <string.h>

/* 变量声明 */
extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;

static uint8_t cv_rx_buf[CV_QUEUE_ITEM_SIZE];
QueueHandle_t xCv_Queue;

/**
  * @brief          自定义控制器接收数据初始化
  * @param[in]      本函数将会在FreeRTOS中的初始化环节被调用
  * @retval         none
  */
void CV_Control_Init(void)
{
    /* 串口不定长接收+DMA初始化 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, cv_rx_buf, CV_QUEUE_ITEM_SIZE);

    /* 消息队列初始化 */
    xCv_Queue = xQueueCreate(CV_QUEUE_ITEM_NUM, CV_QUEUE_ITEM_SIZE);
}

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
QueueHandle_t xGet_CV_Queue(void)
{
    return xCv_Queue;
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
    HAL_UARTEx_ReceiveToIdle_DMA(&huart10, cv_rx_buf, CV_QUEUE_ITEM_SIZE);
    memset(cv_rx_buf, 0, CV_QUEUE_ITEM_SIZE);
}

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
        if (Size <= CV_QUEUE_ITEM_SIZE)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, cv_rx_buf, CV_QUEUE_ITEM_SIZE); // 重新开启接收下一次串口中断数据
            /* 将数据写入队列 */
            QueueHandle_t xCV_Queue = xGet_CV_Queue();
            CV_Data_t cv_info_W;

            memcpy(&cv_info_W, cv_rx_buf, sizeof(CV_Data_t)); // 将字节数组复制到结构体
            xQueueSendFromISR(xCV_Queue, &cv_info_W, NULL);
        }
        else
        {
            HAL_UARTEx_ReceiveToIdle_DMA(&huart10, cv_rx_buf, CV_QUEUE_ITEM_SIZE);
            memset(cv_rx_buf, 0, CV_QUEUE_ITEM_SIZE);
        }
    }
}
