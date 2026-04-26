/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       Dev_Remote_Control.cpp
  * @brief      DR16兼容接口层
  * @note       1. 保留旧设备层接口名称，避免其他模块链接失败
  *             2. 真正的数据接收已迁移到HDL_DR16，解析迁移到APL_RC_Hub
  *             3. 本文件仅负责兼容旧接口调用
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-13-2026     Codex           1. reconstruct as compatibility layer
  *
  @verbatim
  ==============================================================================
  * 当前工程中DR16接收链路为：
  * HAL弱回调 -> CallBack_USART -> HDL_DR16 -> Queue -> APL_RC_Hub
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#include "Dev_Remote_Control.h"
#include "HDL_DR16.h"

/**
 * @brief 遥控器出错数据上限
 */
#define RC_CHANNEL_ERROR_VALUE 700

/**
 * @brief          绝对值函数
 * @param[in]      value 输入值
 * @retval         绝对值
 */
static int16_t RC_abs(int16_t value)
{
    if (value >= 0)
    {
        return value;
    }

    return (int16_t)(-value);
}

/**
 * @brief          遥控器初始化
 * @param[in]      none
 * @retval         none
 * @note           保留旧接口，内部转调新的DR16 HDL对象
 */
void Remote_Control_Init(void)
{
    DR16_Module.Init(&UART5_Manage_Object);
}

/**
 * @brief          获取遥控器DT7数据指针
 * @param[in]      none
 * @retval         遥控器数据结构体地址
 */
const dr16_control_t *Get_Remote_Control_Point(void)
{
    return &RC_Hub.dr16_control;
}

/**
 * @brief          串口中断服务函数
 * @param[in]      huart 串口句柄
 *                 Size  接收字节数
 * @retval         none
 * @note           保留旧接口，内部转调新的DR16 HDL对象
 */
void UART5_ISR_Handler(UART_HandleTypeDef *huart, uint16_t Size)
{
    DR16_Module.DR16_Data_Processing(huart, Size);
}

/**
 * @brief          串口中断错误处理函数
 * @param[in]      none
 * @retval         none
 */
void UART5_Error_Handler(void)
{
    DR16_Module.DR16_Error_Processing(UART5_Manage_Object.huart);
}

/**
 * @brief          判断遥控器数据是否出错
 * @param[in]      none
 * @retval         1 数据异常
 *                 0 数据正常
 */
uint8_t RC_Data_Is_Error(void)
{
    dr16_control_t *rc_ctrl = &RC_Hub.dr16_control;

    if (RC_abs(rc_ctrl->rc.ch[0]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[1]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[2]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[3]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[4]) > RC_CHANNEL_ERROR_VALUE ||
        rc_ctrl->rc.s[0] == 0 ||
        rc_ctrl->rc.s[1] == 0)
    {
        rc_ctrl->rc.ch[0] = 0;
        rc_ctrl->rc.ch[1] = 0;
        rc_ctrl->rc.ch[2] = 0;
        rc_ctrl->rc.ch[3] = 0;
        rc_ctrl->rc.ch[4] = 0;
        rc_ctrl->rc.s[0] = RC_SW_DOWN;
        rc_ctrl->rc.s[1] = RC_SW_DOWN;
        rc_ctrl->mouse.x = 0;
        rc_ctrl->mouse.y = 0;
        rc_ctrl->mouse.z = 0;
        rc_ctrl->mouse.press_l = 0;
        rc_ctrl->mouse.press_r = 0;
        rc_ctrl->key.v = 0;
        return 1U;
    }

    return 0U;
}

/**
 * @brief          解决遥控器掉线问题
 * @param[in]      none
 * @retval         none
 */
void Slove_RC_Lost(void)
{
    (void)DR16_Module.RestartReceive();
}

/**
 * @brief          解决遥控器数据异常问题
 * @param[in]      none
 * @retval         none
 */
void Slove_Data_Error(void)
{
    (void)DR16_Module.RestartReceive();
}

/**
 * @brief          透传SBUS数据到其它串口
 * @param[in]      sbus sbus数据指针
 * @retval         none
 * @note           当前工程未使用该接口，保留空实现仅用于兼容旧声明
 */
void Sbus_to_Usart1(uint8_t *sbus)
{
    (void)sbus;
}
