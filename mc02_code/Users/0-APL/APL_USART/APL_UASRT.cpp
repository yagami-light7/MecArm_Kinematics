/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       APL_USART.cpp
  * @brief      串口回调函数
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-29-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "APL_UASRT.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "Dev_Referee.h"
#include "HDL_VT.h"
#include "HAL_USART.h"

/**
 * @brief          不定长接收中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{
    if(huart->Instance == UART5)        // DR16
    {
        // DR16
        UART5_ISR_Handler(huart, Size);
    }
    else if(huart->Instance == UART7)
    {
        // 图传链路
        VT_Module.VT_Data_Processing(UART7_Manage_Object.rx_buffer, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.rx_buffer, UART7_Manage_Object.rx_data_size);
    }
    else if(huart->Instance == USART10) // 常规链路
    {
        // 常规链路
        UART10_ISR_Handler(huart, Size);
    }
}


/**
 * @brief          错误处理中断
 * @param[in]      none
 * @retval         none
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        UART5_Error_Handler();
    }
    else if(huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART7_Manage_Object.rx_buffer, UART7_Manage_Object.rx_data_size);
    }
    else if(huart->Instance == USART10)
    {
        UART10_Error_Handler();
    }
}