/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       CallBack_FDCAN.cpp
  * @brief      电机 FDCAN 回调入口
  * @note       Application Layer
  *             1. 统一承接 HAL 的 FDCAN FIFO0/FIFO1 中断回调
  *             2. 回调中持续搬运 FIFO 中所有接收帧
  *             3. 每一帧都交给 HDL_Motor_Rx 做品牌驱动分发
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "CallBack_FDCAN.h"

#include "HDL_Motor_Rx.h"

/**
  * @brief          电机 FDCAN FIFO 回调处理函数
  * @param[in]      hfdcan 触发中断的 FDCAN 句柄
  *                 fifo   当前接收 FIFO
  * @retval         none
  * @note           FIFO 中可能在一次中断里堆积多帧，因此这里循环读取直到清空
  */
static void Motor_Rx_FIFO_Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, fifo) > 0U)
    {
        Motor_Rx.DispatchRx(hfdcan, fifo);
    }
}

/**
  * @brief          FDCAN FIFO0 接收中断回调函数
  * @param[in]      hfdcan     FDCAN 句柄指针
  *                 RxFifo0ITs FIFO0 中断标志位
  * @retval         none
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        // 后续如需接入统计或告警，可在此补充
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U ||
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0U ||
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_WATERMARK) != 0U)
    {
        Motor_Rx_FIFO_Callback(hfdcan, FDCAN_RX_FIFO0);
    }
}

/**
  * @brief          FDCAN FIFO1 接收中断回调函数
  * @param[in]      hfdcan     FDCAN 句柄指针
  *                 RxFifo1ITs FIFO1 中断标志位
  * @retval         none
  */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_MESSAGE_LOST) != 0U)
    {
        // 后续如需接入统计或告警，可在此补充
    }

    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != 0U ||
        (RxFifo1ITs & FDCAN_IT_RX_FIFO1_FULL) != 0U ||
        (RxFifo1ITs & FDCAN_IT_RX_FIFO1_WATERMARK) != 0U)
    {
        Motor_Rx_FIFO_Callback(hfdcan, FDCAN_RX_FIFO1);
    }
}
