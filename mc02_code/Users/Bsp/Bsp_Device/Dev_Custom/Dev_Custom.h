#ifndef ENGINEERING_DEV_CUSTOM_H
#define ENGINEERING_DEV_CUSTOM_H

#include "main.h"
#include "cmsis_os.h"

#define CUSTOM_QUEUE_ITEM_NUM    30   // 消息队列中存放的数据个数
#define CUSTOM_QUEUE_ITEM_SIZE  sizeof(Custom_Data_t) // 消息队列中存放的数据大小

#define RC_QUEUE_ITEM_NUM    30   // 消息队列中存放的数据个数
#define RC_QUEUE_ITEM_SIZE  sizeof(Remote_Data_t) // 消息队列中存放的数据大小

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

#pragma pack(push, 1)  // 开始紧凑对齐

typedef __packed struct
{
    __packed struct
    {
        uint8_t  sof;             // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t  seq;             // 包序号
        uint8_t  crc8;            // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t  data[30];   // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
} Custom_Data_t;            // 自定义控制器数据包

// 图传链路键鼠数据结构体
typedef __packed struct
{
    __packed struct
    {
        uint8_t  sof;             // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t  seq;             // 包序号
        uint8_t  crc8;            // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码

    // 原始数据部分
    uint8_t data[12];

    __packed uint16_t frame_tail; // 帧尾CRC16校验

}Remote_Data_t;

#pragma pack(pop)     // 恢复默认对齐

static uint8_t custom_rx_buf[CUSTOM_QUEUE_ITEM_SIZE]; // 串口接收缓冲区


#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief          自定义控制器接收数据初始化
  * @param[in]      本函数将会在FreeRTOS中的初始化环节被调用
  * @retval         none
  */
extern void Custom_Control_Init(void);

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
extern QueueHandle_t xGet_Custom_Queue(void);

/**
  * @brief          返回消息队列的句柄
  * @param[in]      none
  * @retval         none
  */
extern QueueHandle_t xGet_RC_Queue(void);

/**
  * @brief          串口中断服务函数
  *                 将在HAL_UARTEx_RxEventCallback串口7的空闲中断回调函数中被调用
  * @param[in]      huart: 对应的串口句柄
  *                 Size : 数据长度
  * @retval          none
  */
extern void UART7_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size);

/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口7的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
extern void UART7_Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif
