/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HAL_USART.cpp
  * @brief      USART外设再封装
  * @note       Hardware Abstract Layer硬件抽象层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Sep-29-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考中科大2024年工程机器人电控代码开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "HAL_USART.h"

/**
 * @brief  UART管理对象实例化
 */
UART_Manage_Object_t UART5_Manage_Object  = {&huart5, {0}, {0}, 0U};
UART_Manage_Object_t UART7_Manage_Object  = {&huart7, {0}, {0}, 0U};
UART_Manage_Object_t UART10_Manage_Object = {&huart10, {0}, {0}, 0U};

/**
 * @brief          UART串口初始化
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 rx_data_size         接收数据长度
 * @retval         none
 */
void UART_Init(UART_Manage_Object_t *UART_Manage_Object, uint16_t rx_data_size)
{
    UART_Manage_Object->rx_data_size = rx_data_size;
    HAL_UARTEx_ReceiveToIdle_DMA(UART_Manage_Object->huart, UART_Manage_Object->rx_buffer, rx_data_size);
}

/**
 * @brief          双缓冲DMA错误回调
 * @param[in]      hdma DMA句柄
 * @retval         none
 * @note           当前仅将DMA错误上抛给HAL弱定义错误回调入口
 */
static void UART_Double_Buffer_DMA_Error_Callback(DMA_HandleTypeDef *hdma)
{
    UART_HandleTypeDef *huart = NULL;

    if (hdma == NULL)
    {
        return;
    }

    huart = (UART_HandleTypeDef *)hdma->Parent;
    if (huart == NULL)
    {
        return;
    }

    huart->ErrorCode |= HAL_UART_ERROR_DMA;
    HAL_UART_ErrorCallback(huart);
}

/**
 * @brief          启动空闲中断+双DMA缓冲区接收
 * @param[in]      huart         串口句柄
 *                 rx_buffer_0   DMA缓冲区0
 *                 rx_buffer_1   DMA缓冲区1
 *                 rx_data_size  单个DMA缓冲区长度
 * @retval         true          启动成功
 *                 false         启动失败
 * @note           该接口面向UART5/DR16场景，特点如下：
 *                 1. 依然使用HAL弱定义的IDLE回调作为中断入口
 *                 2. DMA采用双缓冲区模式，但每次空闲中断后都会重新拉起DMA，
 *                    以保证下一帧继续从缓冲区起始地址开始写入
 *                 3. 关闭HT/TC中断，仅保留IDLE和错误链路，避免半包事件干扰上层逻辑
 */
static bool UART_Start_Idle_Double_Buffer(UART_HandleTypeDef *huart, uint8_t *rx_buffer_0, uint8_t *rx_buffer_1,
                                          uint16_t rx_data_size)
{
    DMA_HandleTypeDef *hdma = NULL;

    if (huart == NULL || rx_buffer_0 == NULL || rx_buffer_1 == NULL || rx_data_size == 0U)
    {
        return false;
    }

    hdma = huart->hdmarx;
    if (hdma == NULL)
    {
        return false;
    }

    /* 重新启动前先停掉旧的DMA接收状态，保证NDTR和CT位回到可配置状态 */
    (void)HAL_UART_DMAStop(huart);

    huart->pRxBuffPtr = rx_buffer_0;
    huart->RxXferSize = rx_data_size;
    huart->RxXferCount = rx_data_size;
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    huart->RxState = HAL_UART_STATE_BUSY_RX;
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    huart->RxEventType = HAL_UART_RXEVENT_TC;

    hdma->XferCpltCallback = NULL;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferErrorCallback = UART_Double_Buffer_DMA_Error_Callback;
    hdma->XferAbortCallback = NULL;

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_IDLEF | UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);

    if (HAL_DMAEx_MultiBufferStart_IT(hdma,
                                      (uint32_t)&huart->Instance->RDR,
                                      (uint32_t)rx_buffer_0,
                                      (uint32_t)rx_buffer_1,
                                      rx_data_size) != HAL_OK)
    {
        huart->RxState = HAL_UART_STATE_READY;
        huart->ReceptionType = HAL_UART_RECEPTION_STANDARD;
        return false;
    }

    /* DR16只关心IDLE帧结束事件，关闭HT/TC中断避免空队列和重复解析 */
    __HAL_DMA_DISABLE_IT(hdma, DMA_IT_HT | DMA_IT_TC);

    if (huart->Init.Parity != UART_PARITY_NONE)
    {
        ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_PEIE);
    }

    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
    ATOMIC_SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    __HAL_UART_CLEAR_IDLEFLAG(huart);
    ATOMIC_SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);

    return true;
}

/**
 * @brief          UART使用空闲中断+双DMA缓冲区启动接收
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 rx_buffer_0          DMA缓冲区0
 *                 rx_buffer_1          DMA缓冲区1
 *                 rx_data_size         单个DMA缓冲区长度
 * @retval         true                 启动成功
 *                 false                启动失败
 */
bool UART_Init_Double_Buffer(UART_Manage_Object_t *UART_Manage_Object, uint8_t *rx_buffer_0, uint8_t *rx_buffer_1,
                             uint16_t rx_data_size)
{
    if (UART_Manage_Object == NULL)
    {
        return false;
    }

    UART_Manage_Object->rx_data_size = rx_data_size;

    return UART_Start_Idle_Double_Buffer(UART_Manage_Object->huart, rx_buffer_0, rx_buffer_1, rx_data_size);
}


/**
 * @brief          UART发送数据
 * @param[in]      UART_Manage_Object   串口实例对象
 *                 data                 数据
 *                 length               数据长度
 * @retval         发送结果
 */
bool UART_Send_Data(UART_Manage_Object_t *UART_Manage_Object, uint8_t *data, uint16_t length)
{
    return (HAL_UART_Transmit_DMA(UART_Manage_Object->huart, data, length) == HAL_OK);
}
