#ifndef ENGINEERING_DEV_CV_H
#define ENGINEERING_DEV_CV_H

#include "main.h"
#include "cmsis_os.h"

#define CV_QUEUE_ITEM_NUM    50   // 消息队列中存放的数据个数
#define CV_QUEUE_ITEM_SIZE  sizeof(CV_Data_t) // 消息队列中存放的数据大小

typedef __packed struct
{
    uint8_t  sof;                 // 起始字节，固定值为0xA5
    __packed uint8_t  data[30];   // 自定义控制器的数据帧,存放6个float类型数据
    __packed uint16_t frame_tail; // 帧尾CRC16校验
} CV_Data_t;            // 自定义控制器数据包

/**
  * @brief          视觉信息接收数据初始化
  * @param[in]      本函数将会在FreeRTOS中的初始化环节被调用
  * @retval         none
  */
extern void CV_Control_Init(void);

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
extern QueueHandle_t xGet_CV_Queue(void);

/**
  * @brief          串口中断服务函数
  *                 将在HAL_UARTEx_RxEventCallback串口10的空闲中断回调函数中被调用
  * @param[in]      huart: 对应的串口句柄
  *                 Size : 数据长度
  * @retval          none
  */
extern void UART10_ISR_Handler(UART_HandleTypeDef *huart,uint16_t Size);

/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口10的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
extern void UART10_Error_Handler(void);

#endif
