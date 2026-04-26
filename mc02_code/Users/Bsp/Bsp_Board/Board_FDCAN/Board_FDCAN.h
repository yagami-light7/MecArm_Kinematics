/**
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  * @file       Board_FDCAN.c
  * @brief      在H7系列开发板下，将FDCAN配置成经典CAN模式并进行初始化，并封装了FDCAN发送函数
  * @note       在工程机器人框架       FDCAN1:底盘DJI电机
  *                                 FDCAN2:大关节XIAO_MI电机
  *                                 FDCAN3:小关节DJI电机
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-28-2024     Light           1. done
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考了达妙mc02开发板开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  */
#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
  * @brief          FDCAN总线初始化
  * @param[in]      none
  * @retval         none
  */
extern void FDCAN_Filter_Init(void);

/**
 * @brief           FDCAN发送函数
 * @param[in]       hfdcan         FDCAN控制器的句柄
 * @param[in]       hfdcan_id      CAN消息的标识符
 * @param[in]       mes_data      指向要发送的数据的指针
 * @param[in]       mes_len       要发送的数据长度
 * @retval          uint8_t 返回状态，0表示成功
 */
extern uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t hfdcan_id, uint8_t *mes_data, uint32_t mes_len);

#endif
