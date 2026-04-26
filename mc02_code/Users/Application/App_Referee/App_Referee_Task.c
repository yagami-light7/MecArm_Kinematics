/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       App_Referee_Task.c
  * @brief      裁判系统回传数据解析任务
  * @note       串口不定长接收+DMA搬运来自mc_02喵板的UART10数据，进行解析得到比赛中来自常规链路的数据
  * @history
  *  Version    Date               Author          Modification
  *  V1.0.0     March-2nd-2025     Light           1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

#include "App_Referee_Task.h"
#include "cmsis_os.h"
#include "Sp_FIFO.h"
#include "ui_interface.h"

fifo_s_t referee_fifo;
uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
uint8_t referee_uart_buf[2][USART_RX_BUF_LENGHT];
unpack_data_t referee_unpack_obj;

/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void);

/**
 * @brief          裁判系统通信任务
 * @param[in]      none
 * @retval         none
 */
void Referee_Task(void *pvParameters)
{
    /* 延迟一段时间以待底层初始化 */
    vTaskDelay(Referee_TASK_INIT_TIME);

    /* 初始化 */
    init_referee_struct_data();
//    Referee_UART_Dma_Init(referee_uart_buf[0], referee_uart_buf[1], USART_RX_BUF_LENGHT);
    fifo_s_init(&referee_fifo, referee_fifo_buf, REFEREE_FIFO_BUF_LENGTH);

    while(1)
    {
        get_robot_id(&ui_self_id, &client_id);
        referee_unpack_fifo_data();
        vTaskDelay(Referee_TASK_CONTROL_TIME);
    }
}


/**
  * @brief          单字节解包
  * @param[in]      void
  * @retval         none
  */
void referee_unpack_fifo_data(void)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF;
    unpack_data_t *p_obj = &referee_unpack_obj;

    while ( fifo_s_used(&referee_fifo) )
    {
        byte = fifo_s_get(&referee_fifo);
        switch(p_obj->unpack_step)
        {
            case STEP_HEADER_SOF:
            {
                if(byte == sof)
                {
                    p_obj->unpack_step = STEP_LENGTH_LOW;
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                else
                {
                    p_obj->index = 0;
                }
            }break;

            case STEP_LENGTH_LOW:
            {
                p_obj->data_len = byte;
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_LENGTH_HIGH;
            }break;

            case STEP_LENGTH_HIGH:
            {
                p_obj->data_len |= (byte << 8);
                p_obj->protocol_packet[p_obj->index++] = byte;

                if(p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
                {
                    p_obj->unpack_step = STEP_FRAME_SEQ;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }break;
            case STEP_FRAME_SEQ:
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
                p_obj->unpack_step = STEP_HEADER_CRC8;
            }break;

            case STEP_HEADER_CRC8:
            {
                p_obj->protocol_packet[p_obj->index++] = byte;

                if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
                {
                    if ( verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE) )
                    {
                        p_obj->unpack_step = STEP_DATA_CRC16;
                    }
                    else
                    {
                        p_obj->unpack_step = STEP_HEADER_SOF;
                        p_obj->index = 0;
                    }
                }
            }break;

            case STEP_DATA_CRC16:
            {
                if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->protocol_packet[p_obj->index++] = byte;
                }
                if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;

                    if ( verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len) )
                    {
                        referee_data_solve(p_obj->protocol_packet);
                    }
                }
            }break;

            default:
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }break;
        }
    }
}