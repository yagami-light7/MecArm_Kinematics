/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_DR16.h
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
  * 该文件用于将DT7/DR16的接收链路从旧设备层中剥离出来
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#pragma once

/**
 * @brief 头文件
 */
#include "HAL_USART.h"

/**
 * @brief 宏定义
 */

/**
 * @brief 结构体
 */
typedef struct
{
    QueueHandle_t xdr16_queue;      // DR16原始帧队列，队列元素固定为18字节原始数据
} dr16_robot_t;

/**
 * @brief 变量外部声明
 */

/**
 * @brief CPP部分
 */
#ifdef __cplusplus

class Class_DR16
{
public:
    /**
     * @brief 构造函数
     */
    Class_DR16();

    /**
     * @brief          初始化DR16驱动对象
     * @param[in]      _UART_Manage_Obj_ 串口实例对象
     * @retval         none
     */
    void Init(UART_Manage_Object_t *_UART_Manage_Obj_);

    /**
     * @brief          DR16串口接收事件处理
     * @param[in]      huart 串口句柄
     *                 Size  本次接收字节数
     * @retval         none
     * @note           中断中只做入队和重启DMA，不做协议解析
     */
    void DR16_Data_Processing(UART_HandleTypeDef *huart, uint16_t Size);

    /**
     * @brief          DR16串口错误处理
     * @param[in]      huart 串口句柄
     * @retval         none
     */
    void DR16_Error_Processing(UART_HandleTypeDef *huart);

    /**
     * @brief          重启DR16接收
     * @param[in]      none
     * @retval         true  重启成功
     *                 false 重启失败
     */
    bool RestartReceive(void);

    dr16_robot_t dr16_robot;        // DR16原始接收队列

private:
    /**
     * @brief 获取当前DMA正在写入的缓冲区地址
     */
    uint8_t *Get_Current_DMA_Buffer(void);

    /**
     * @brief 清空DMA缓冲区
     */
    void Clear_DMA_Buffer(void);

    UART_Manage_Object_t *UART_Manage_Obj;    // 绑定的UART管理对象
    uint8_t is_init_;                         // 初始化标志位
};

extern Class_DR16 DR16_Module;

#endif

/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
