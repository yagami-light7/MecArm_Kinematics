/**
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  * @file       Board_RC.c
  * @brief      使用DMA双缓冲区+IDLE串口空闲中断驱动UART5 接收DT7数据
  * @note       本文件参考 辽宁科技大学——王草凡 开源
  *             仿照c板例程代码改写
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-29-2024     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  */
#include "Board_RC.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef  hdma_uart5_rx;

/**
  * @brief  USART5 DMA双缓冲区初始化
  * @param  UART_HandleTypeDef *huart：接收哪个串口数据的结构体指针。
  *         Fir_MemAddress： 第一个缓冲区的地址
  *         Sec_MemAddress： 第二个缓冲区的地址
  *         DataLength：     接收数据的长度
  * @retval None
  *
  * */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength);

/**
  * @brief          片上串口DMA初始化
  * @param[in]      rx1_buf         内存缓冲区1
  *                 rx2_buf         内存缓冲区2
  *                 dma_buf_num     数据长度(单个缓冲区的长度)
  *                 指定将DMA传输的数据存入rx1_buf与rx2_buf中，之后直接读取rx1_buf/rx2_buf即可
  * @retval         none
  */
//void RC_Init(uint32_t *SrcAddress, uint8_t *rx1_buf, uint8_t *rx2_buf, uint8_t dma_buf_num)
//{
//    USART_RxDMA_MultiBuffer_Init(&huart5, (uint32_t *)rx1_buf, (uint32_t *)rx2_buf, dma_buf_num);
//}

/**
  * @brief          失能串口
  * @param[in]      none
  * @retval         none
  */
void RC_Disable(void)
{
    __HAL_UART_DISABLE(&huart5);
}

/**
  * @brief          重启串口，以保证热插拔的稳定性
  * @param[in]      none
  * @retval         none
  */
void RC_Restart(uint16_t dma_buf_num)
{
    __HAL_UART_DISABLE(&huart5);
    __HAL_DMA_DISABLE(&hdma_uart5_rx);

    ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->NDTR = dma_buf_num;

    __HAL_DMA_ENABLE(&hdma_uart5_rx);
    __HAL_UART_ENABLE(&huart5);
}


/**
  * @brief  USART5 DMA双缓冲区初始化
  * @param  UART_HandleTypeDef *huart：接收哪个串口数据的结构体指针。
  *         Fir_MemAddress： 第一个缓冲区的地址
  *         Sec_MemAddress： 第二个缓冲区的地址
  *         DataLength：     接收数据的长度
  * @retval None
  *
  * */
static void USART_RxDMA_MultiBuffer_Init(UART_HandleTypeDef *huart, uint32_t *DstAddress, uint32_t *SecondMemAddress, uint32_t DataLength)
{
    huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;

    huart->RxXferSize    = DataLength * 2;

    SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);

    do{
        __HAL_DMA_DISABLE(huart->hdmarx);
    }while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);

    /* Configure the source memory Buffer address  */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->RDR;

    /* Configure the destination memory Buffer address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;

    /* Configure DMA Stream destination address */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;

    /* Configure the length of data to be transferred from source to destination */
    ((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

    /* Enable double memory buffer */
    SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

    /* Enable DMA */
    __HAL_DMA_ENABLE(huart->hdmarx);

}

