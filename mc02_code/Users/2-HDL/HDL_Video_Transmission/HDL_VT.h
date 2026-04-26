/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_VT.cpp
  * @brief      Video Transmission Module图传模块串口通信驱动
  * @note       在串口空闲中断中调用处理函数，处理函数拷贝原始数据之后写入队列
  *             上层解析函数接收队列数据，在任务完成解析
  *             Hardware Driver Layer 硬件驱动层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-27-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考中科大2024年工程机器人电控代码开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include <string.h>
#include "main.h"
#include "HAL_USART.h"

#pragma pack(push, 1)  // 开始紧凑对齐
/**
 * @brief 帧头结构体
 */
typedef __packed struct
{
    uint8_t SOF;
    uint16_t data_length;
    uint8_t seq;
    uint8_t CRC8;
}frame_header_t;

#pragma pack(pop)     // 恢复默认对齐

/**
 * @brief 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz 0x0302
 */
typedef struct
{
    QueueHandle_t xcustom_queue;
}custom_robot_t;


/**
 * @brief 键鼠遥控数据，固定 30Hz 频率发送 0x0304
 */
typedef struct
{
    QueueHandle_t xremote_queue;
}remote_robot_t;


/**
 * @brief VT03 键鼠数据 + 遥控数据 每14ms发送
 */
typedef struct
{
    QueueHandle_t xrc_vt_queue;
}vt_rc_robot_t;

#ifdef __cplusplus
/**
 * @brief 图传链路类
 */
class Class_Video_Transmission
{
public:
    /* Functions */
    void Init(UART_Manage_Object_t *_UART_Manage_Obj_);
    void VT_Data_Processing(uint8_t *data,uint16_t length); //裁判系统数据处理

//    QueueHandle_t xGet_Custom_Queue(void);          // 获取自定义控制器0x302的队列句柄
//    QueueHandle_t xGet_RC_Queue(void);              // 获取键鼠数据0x304的队列句柄
//    QueueHandle_t xGet_RC_VT_Queue(void);           // 获取新图传键鼠+摇杆数据的队列句柄

    /* Variables */
    frame_header_t Frame_header;                    //帧头
    uint16_t cmd_id;                                //命令码ID

    custom_robot_t custom_robot;                    //自定义控制器与机器人交互数据 0x0302
    remote_robot_t remote_robot;                    //键鼠遥控数据 0x0304
    vt_rc_robot_t  vt_rc_robot;                     //新图传键鼠+摇杆数据

    uint8_t crc8_check;                             //CRC8校验,1为成功，0为失败
    uint8_t crc16_check;                            //CRC16校验,1为成功，0为失败

private:
    UART_Manage_Object_t * UART_Mangae_Obj;    /*!< 绑定的UART */
};

/**
 * @brief 变量外部声明
 */
extern Class_Video_Transmission VT_Module;

#endif
