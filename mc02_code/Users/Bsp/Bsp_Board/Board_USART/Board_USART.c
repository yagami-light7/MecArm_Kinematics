#include "Board_USART.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

///**
// * @brief          不定长接收中断
// * @param[in]      none
// * @retval         none
// */
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
//{
//    if(huart == &huart5)
//    {
//        // DT7
//        UART5_ISR_Handler(huart, Size);
//    }
//    else if(huart == &huart7)
//    {
//        // 裁判系统 图传链路
//        UART7_ISR_Handler(huart, Size);
//    }
//    else if(huart == &huart10)
//    {
//        // CV视觉 之后优化为USB虚拟串口，本次比赛用作接收裁判系统常规链路
//        UART10_ISR_Handler(huart, Size);
//    }
//}
//
//
///**
// * @brief          错误处理中断
// * @param[in]      none
// * @retval         none
// */
//void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
//{
//    if (huart->Instance == UART5)
//    {
//        UART5_Error_Handler();
//    }
//    else if(huart->Instance == UART7)
//    {
//        UART7_Error_Handler();
//    }
//    else if(huart->Instance == USART10)
//    {
//        UART10_Error_Handler();
//    }
//}

/**
 * @brief          裁判系统串口初始化发送函数
 * @param[in]      none
 * @retval         none
 */
void Referee_UART_Dma_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //使能DMA串口接收和发送
    SET_BIT(huart10.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart10.Instance->CR3, USART_CR3_DMAT);

    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart10, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart10_rx);

    while(((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart10_rx, DMA_LISR_TCIF1);

    ((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->PAR = (uint32_t) & (USART10->RDR);

    //内存缓冲区1
    ((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->M0AR = (uint32_t)(rx1_buf);

    //内存缓冲区2
    ((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->M1AR = (uint32_t)(rx2_buf);

    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_usart10_rx, dma_buf_num);

    //使能双缓冲区
    SET_BIT(((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart10_rx);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart10_rx);

    while(((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_rx);
    }

    ((DMA_Stream_TypeDef   *)hdma_usart10_rx.Instance)->PAR = (uint32_t) & (USART10->RDR);

}


/**
 * @brief          裁判系统串口发送函数
 * @param[in]      none
 * @retval         none
 */

void Referee_Tx_Dma_Enable(uint8_t *data, uint16_t len)
{
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart10_tx);

    while( ( (DMA_Stream_TypeDef   *)hdma_usart10_tx.Instance)->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart10_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart10_tx, DMA_HISR_TCIF6);

    ( (DMA_Stream_TypeDef   *)hdma_usart10_tx.Instance)->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart10_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart10_tx);
}