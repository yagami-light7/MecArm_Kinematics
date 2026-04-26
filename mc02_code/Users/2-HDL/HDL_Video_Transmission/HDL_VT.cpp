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
#include "HDL_VT.h"
#include "MWL_CRC.h"
#include "App_Detect_Task.h"

/**
 * @brief 创建图传对象
 */
Class_Video_Transmission VT_Module;


/**
 * @brief          初始化图传串口、创建队列
 * @param[in]      *UART_Manage_Obj 串口实例对象
 * @retval         none
 */

void Class_Video_Transmission::Init(UART_Manage_Object_t *_UART_Manage_Obj_)
{
    this->UART_Mangae_Obj = _UART_Manage_Obj_;
    // 应在RTOS内核调度启动后创建
    this->custom_robot.xcustom_queue = xQueueCreate(QUEUE_RX_ITEM_NUM, CUSTOM_ROBOT_DATA_SIZE);
    this->remote_robot.xremote_queue = xQueueCreate(QUEUE_RX_ITEM_NUM, REMOTE_ROBOT_DATA_SIZE);
    this->vt_rc_robot.xrc_vt_queue   = xQueueCreate(QUEUE_RX_ITEM_NUM, VT_RC_ROBOT_DATA_SIZE );
    // 启动串口中断
    UART_Init(this->UART_Mangae_Obj, 512);
}


/**
 * @brief          图传链路数据处理
 * @param[in]      *data    串口数据
 * @retval         1:解析成功
 *                 0:解析失败
 */
void Class_Video_Transmission::VT_Data_Processing(uint8_t *data, uint16_t length)
{
    auto data_head = data;

    uint8_t custom_data[30];
    uint8_t remote_data[12];
    uint8_t vt_rc_data[21];

    while ((data - data_head) < length)
    {
        // ---------- 图传遥控帧 ----------
        if ((data[0] == 0xA9 && data[1] == 0x53) && ((data - data_head) + 21 <= length))
        {
            memcpy(vt_rc_data, data, sizeof(vt_rc_data));
            xQueueSendFromISR(vt_rc_robot.xrc_vt_queue, vt_rc_data, 0);
//            detect_hook(VT_TOE);
            data += 21;
            continue;
        }

        // ---------- 自定义控制帧或键鼠帧 ----------
        if ((data[0] == 0xA5) && ((data - data_head) + sizeof(Frame_header) + sizeof(cmd_id) <= length))
        {
            memcpy(&Frame_header, data, sizeof(Frame_header));
            memcpy(&cmd_id, data + sizeof(Frame_header), sizeof(cmd_id));

            // 判断帧是否完整（含CRC16）
            uint16_t full_frame_len = sizeof(Frame_header) + sizeof(cmd_id) + Frame_header.data_length + 2;
            if ((data - data_head) + full_frame_len <= length)
            {

                crc8_check = MWL_CRC8_Verify(data, 5U);
                crc16_check = MWL_CRC16_Verify(data, (uint32_t)Frame_header.data_length + 9U); // header+cmd+payload

                // 通过检验
                if ((Frame_header.SOF == 0xA5) && crc8_check && crc16_check)
                {
                    const auto payload = data + sizeof(Frame_header) + sizeof(cmd_id);

                    switch (cmd_id)
                    {
                        case 0x0302: // 自定义控制器
                            if (Frame_header.data_length == sizeof(custom_data))
                            {
                                memcpy(custom_data, payload, sizeof(custom_data));
                                xQueueSendFromISR(custom_robot.xcustom_queue, custom_data, 0);
//                                detect_hook(VT_TOE);
                            }
                            break;

                        case 0x0304: // 键鼠数据
                            if (Frame_header.data_length == sizeof(remote_data))
                            {
                                memcpy(remote_data, payload, sizeof(remote_data));
                                xQueueSendFromISR(remote_robot.xremote_queue, remote_data, 0);
//                                detect_hook(VT_TOE);
                            }
                            break;

                        default:
                            break;
                    }

                    data += full_frame_len;
                    continue;
                }
            }
        }

        // ---------- 未识别帧头或帧无效，跳过一个字节 ----------
        data++;
    }
}
