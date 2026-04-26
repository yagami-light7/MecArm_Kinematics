/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.0.0     Nov-11-2019     RM              1. support development board tpye c
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef INFANTRY_ROBOT_DEV_REMOTE_CONTROL_H
#define INFANTRY_ROBOT_DEV_REMOTE_CONTROL_H

#include "Board_RC.h"
#include "string.h"
#include "cmsis_os.h"
#include "HAL_USART.h"
#include "APL_RC_Hub.h"

#define SBUS_RX_BUF_NUM         DR16_DMA_BUFFER_SIZE
#define RC_FRAME_LENGTH         DR16_FRAME_LENGTH

#ifndef RC_CH_VALUE_MIN
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#endif

#ifndef RC_CH_VALUE_OFFSET
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#endif

#ifndef RC_CH_VALUE_MAX
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
#endif

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_SHOOT_CHANNEL_READYSHOOT							((int16_t)  650)
#define RC_SHOOT_CHANNEL_READYSHOOT_CONTINUE		        ((int16_t) -650)
#define SHOOT_CHANNEL_IS_READY(ch)							(ch > RC_SHOOT_CHANNEL_READYSHOOT || ch < RC_SHOOT_CHANNEL_READYSHOOT_CONTINUE)
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
/* ----------------------- Data Struct ------------------------------------- */

///* 遥控器控制变量 */
//extern dr16_control_t rc_ctrl;

#ifdef __cplusplus
extern "C"{
#endif


/**
  * @brief          遥控器初始化
  *                 本函数将会在FreeRTOS中的初始化环节被调用
  * @param[in]      none
  * @retval         none
  */
extern void Remote_Control_Init(void);

/**
  * @brief          获取遥控器DT7数据指针
  * @param[in]      none
  * @retval         none
  */
extern const dr16_control_t *Get_Remote_Control_Point(void);

/**
  * @brief          串口中断服务函数
  *                 将在HAL_UARTEx_RxEventCallback串口5的空闲中断回调函数中被调用
  * @param[in]      huart: 对应的串口句柄
  *                 Size : 数据长度
  *@retval          none
  */
extern void UART5_ISR_Handler(UART_HandleTypeDef *huart,uint16_t Size);

/**
  * @brief          串口中断错误处理函数
  *                 将在HAL_UART_ErrorCallback串口5的中断错误回调函数中被调用
  * @param[in]      none
  * @retval         none
  */
extern void UART5_Error_Handler(void);

extern uint8_t RC_Data_Is_Error(void);

extern void Slove_RC_Lost(void);

extern void Slove_Data_Error(void);

extern void Sbus_to_Usart1(uint8_t *sbus);


#ifdef __cplusplus
}
#endif

#endif

