/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       CallBack_USART.cpp
  * @brief      串口回调函数
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-13-2026     Codex           1. reconstruct from APL_USART
  *
  @verbatim
  ==============================================================================
  * 本文件作为HAL弱定义串口回调的统一落点，只负责按照串口实例做分发
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "CallBack_USART.h"

#include "HDL_DR16.h"
#include "HDL_VT.h"
#include "HAL_USART.h"

extern "C" __attribute__((weak)) void UART10_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size)
{
    (void)huart;
    (void)Size;
}

extern "C" __attribute__((weak)) void UART10_Error_Handler(void)
{
}

/**
 * @brief          不定长接收中断
 * @param[in]      huart 串口句柄
 *                 Size  接收字节数
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        DR16_Module.DR16_Data_Processing(huart, Size);
    }
    else if (huart->Instance == UART7)
    {
//        VT_Module.VT_Data_Processing(UART7_Manage_Object.rx_buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.rx_buffer, UART7_Manage_Object.rx_data_size);
    }
    else if (huart->Instance == USART10)
    {
        UART10_ISR_Handler(huart, Size);
    }
}

/**
 * @brief          串口错误处理中断
 * @param[in]      huart 串口句柄
 * @retval         none
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        DR16_Module.DR16_Error_Processing(huart);
    }
    else if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.rx_buffer, UART7_Manage_Object.rx_data_size);
    }
    else if (huart->Instance == USART10)
    {
        UART10_Error_Handler();
    }
}
