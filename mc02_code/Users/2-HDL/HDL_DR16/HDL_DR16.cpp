/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_DR16.cpp
  * @brief      DR16接收驱动
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 使用UART空闲中断+双DMA缓冲区接收原始DR16字节流
  *             2. 中断中只做原始帧拷贝入队，不在中断中解析摇杆与键鼠
  *             3. 解析工作交给APL_RC_Hub线程完成
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-13-2026     Codex           1. done
  *
  @verbatim
  ==============================================================================
  * DR16每帧有效负载固定为18字节，因此中断中仅接受18字节有效空闲帧
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "HDL_DR16.h"

#include <string.h>

/**
 * @brief DMA双缓冲区
 * @note  必须放置到DMA可访问的RAM_D1中，不能落在默认DTCM里
 */
__attribute__((section(".RAM_D1"))) static uint8_t dr16_dma_buffer[2][DR16_DMA_BUFFER_SIZE];

/**
 * @brief 创建DR16对象
 */
Class_DR16 DR16_Module;

/**
 * @brief 构造函数
 */
Class_DR16::Class_DR16()
    : UART_Manage_Obj(NULL),
      is_init_(0U)
{
    memset(&dr16_robot, 0, sizeof(dr16_robot));
}

/**
 * @brief          初始化DR16驱动对象
 * @param[in]      _UART_Manage_Obj_ 串口实例对象
 * @retval         none
 */
void Class_DR16::Init(UART_Manage_Object_t *_UART_Manage_Obj_)
{
    if (_UART_Manage_Obj_ == NULL)
    {
        return;
    }

    UART_Manage_Obj = _UART_Manage_Obj_;

    if (dr16_robot.xdr16_queue == NULL)
    {
        dr16_robot.xdr16_queue = xQueueCreate(QUEUE_RX_ITEM_NUM, DR16_FRAME_LENGTH);
    }

    (void)RestartReceive();
    is_init_ = 1U;
}

/**
 * @brief          DR16串口接收事件处理
 * @param[in]      huart 串口句柄
 *                 Size  本次接收字节数
 * @retval         none
 */
void Class_DR16::DR16_Data_Processing(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t *current_buffer = NULL;

    if (is_init_ == 0U || UART_Manage_Obj == NULL || huart == NULL)
    {
        return;
    }

    if (huart != UART_Manage_Obj->huart)
    {
        return;
    }

    /* 仅处理空闲中断事件，避免HT/TC事件误触发上层解析链路 */
    if (HAL_UARTEx_GetRxEventType(huart) != HAL_UART_RXEVENT_IDLE)
    {
        return;
    }

    current_buffer = Get_Current_DMA_Buffer();
    if (current_buffer != NULL && Size == DR16_FRAME_LENGTH && dr16_robot.xdr16_queue != NULL)
    {
        xQueueSendFromISR(dr16_robot.xdr16_queue, current_buffer, 0);
//        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    (void)RestartReceive();
}

/**
 * @brief          DR16串口错误处理
 * @param[in]      huart 串口句柄
 * @retval         none
 */
void Class_DR16::DR16_Error_Processing(UART_HandleTypeDef *huart)
{
    if (UART_Manage_Obj == NULL || huart == NULL)
    {
        return;
    }

    if (huart != UART_Manage_Obj->huart)
    {
        return;
    }

    (void)RestartReceive();
}

/**
 * @brief          重启DR16接收
 * @param[in]      none
 * @retval         true  重启成功
 *                 false 重启失败
 */
bool Class_DR16::RestartReceive(void)
{
    if (UART_Manage_Obj == NULL)
    {
        return false;
    }

    Clear_DMA_Buffer();

    return UART_Init_Double_Buffer(UART_Manage_Obj,
                                   dr16_dma_buffer[0],
                                   dr16_dma_buffer[1],
                                   DR16_DMA_BUFFER_SIZE);
}

/**
 * @brief 获取当前DMA正在写入的缓冲区地址
 * @retval 缓冲区首地址
 */
uint8_t *Class_DR16::Get_Current_DMA_Buffer(void)
{
    DMA_Stream_TypeDef *dma_stream = NULL;

    if (UART_Manage_Obj == NULL || UART_Manage_Obj->huart == NULL || UART_Manage_Obj->huart->hdmarx == NULL)
    {
        return NULL;
    }

    dma_stream = (DMA_Stream_TypeDef *)UART_Manage_Obj->huart->hdmarx->Instance;
    if (dma_stream == NULL)
    {
        return NULL;
    }

    if ((dma_stream->CR & DMA_SxCR_CT) == 0U)
    {
        return dr16_dma_buffer[0];
    }

    return dr16_dma_buffer[1];
}

/**
 * @brief 清空DMA缓冲区
 * @retval none
 */
void Class_DR16::Clear_DMA_Buffer(void)
{
    memset(dr16_dma_buffer, 0, sizeof(dr16_dma_buffer));
}
