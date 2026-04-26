/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       App_Custom_Task.c
  * @brief      自定义控制器通信接收与解析任务
  * @note       该任务负责解析自定义控制器发送过来的数据
  *             使用队列与任务进行通信实现解耦
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-31-2025     Light            1. done
  *  V1.1.0     Apr-27-2025     Light            2. 串口附录更新
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "App_Custom_Task.h"
#include "Board_CRC8_CRC16.h"
#include "App_Mechanical_Arm_Task.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;

extern mec_arm_control_t mec_arm_control;

/* 发送给自定义控制器的数据 */
Data_Frame_t tx_data;
uint8_t info_data[DATA_LENGTH]; // 数据段数组


/* 函数声明 */
/**
 * @brief          自定义控制器通信接收与解析任务初始化
 * @param[in]      none
 * @retval         none
 */
static void Custom_Init(void);


/**
 * @brief          发送控制信息给自定义控制器，主要是模式切换，替换掉原先的DT7控制
 * @param[in]      none
 * @retval         custom_controller的指针
 */
static void custom_data_transmit(void);


/**
 * @brief          自定义控制器通信数据解析
 * @param[in]      Custom_Data_t data
 * @retval         1:解析成功
 *                 0:解析失败
 */
static bool custom_data_solve(Custom_Data_t *data);


/**
 * @brief          图传链路键鼠通信数据解析
 * @param[in]      Remote_Data_t *rc
 * @retval         1:解析成功
 *                 0:解析失败
 */
static bool rc_data_solve(Custom_Data_t *rc);


/* 变量声明 */
Custom_Controller_t custom_controller;
Remote_Controller_t RC_controller; // 解析完毕，图传链路数据结构体


/**
 * @brief          自定义控制器通信任务
 * @param[in]      none
 * @retval         none
 */
void Custom_Controller_Task(void *pvParameters)
{
    /* 延迟一段时间以待底层初始化 */
    vTaskDelay(CUSTOM_TASK_INIT_TIME);

    /* 初始化 */
    Custom_Init();

    static TickType_t time_tick = 0;
    time_tick = xTaskGetTickCount(); // 获取时间戳

    /* 是否阻塞该任务 */

#if _CUSTOM_USED_ // 使用自定义控制器
    // 不进行操作
#else
    // 删除任务
    vTaskDelete(NULL);
#endif

    while(1)
    {
        /* 读取队列数据 */
        QueueHandle_t xCustom_Queue = xGet_Custom_Queue();
        Custom_Data_t custom_info_R;

        QueueHandle_t xRC_Queue = xGet_RC_Queue();
        Custom_Data_t rc_info_R;

        /* 数据发送，主要用于更改小臂模式 */
        custom_data_transmit();

        /* 进行数据解析 */
        if(pdPASS == xQueueReceive(xCustom_Queue, &custom_info_R, 0))
        {
            if(custom_data_solve(&custom_info_R) == 1)
            {
                /* 解析成功进行角度值同步设定 */
                // 为提升代码内聚性，这一步代码放在其他任务中执行
            }
        }

        if(pdPASS == xQueueReceive(xRC_Queue, &rc_info_R, 0))
        {
            if(rc_data_solve(&rc_info_R) == 1)
            {
                // 标志位
                RC_controller.rc_analysis_flag++;
            }
        }

        vTaskDelay(CUSTOM_TASK_CONTROL_TIME);
    }
}


/**
 * @brief          自定义控制器通信接收与解析任务初始化
 * @param[in]      none
 * @retval         none
 */
static void Custom_Init(void)
{
    for (int i = 0; i < JOINT_NUM; i++)
    {
        custom_controller.custom_angle_set[i] = 0;
    }

}


/**
 * @brief          将uint8_t数组转化为float数组，用于解析串口通信数据，得到角度数据
 * @param[in]      none
 * @retval         none
 */
static void uint8_to_float(uint8_t *uint8_data, float *float_data, int num_floats)
{
    for (int i = 0; i < num_floats; i++)
    {
        float_data[i] = *((float *)(uint8_data + i * sizeof(float)));
    }
}


/**
 * @brief          自定义控制器角度数据解析
 * @param[in]      custom_angle    自定义控制器角度原始数据（小臂角度值）
 *                 custom_rx_data  串口收到的数据
 * @retval         none
 */
static void custom_data_analysis(float *custom_angle, uint8_t *custom_rx_data)
{
    /* 进行转化 */
    uint8_to_float(custom_rx_data, custom_angle, JOINT_NUM);
}


/**
 * @brief          图传链路0x304键鼠信息解析
 * @param[in]      rc    Remote_Controller_t结构体数据
 *                 custom_rx_data  串口收到的数据
 * @retval         none
 */
static void rc_data_analysis(Remote_Controller_t * rc, uint8_t *custom_rx_data)
{
    rc->last_left_button_down = rc->left_button_down;
    rc->last_right_button_down = rc->right_button_down;
    rc->last_keyboard_value = rc->keyboard_value;

    memcpy(&rc->mouse_x, &custom_rx_data[0], sizeof(rc->mouse_x));
    memcpy(&rc->mouse_y, &custom_rx_data[2], sizeof(rc->mouse_y));
    memcpy(&rc->mouse_z, &custom_rx_data[4], sizeof(rc->mouse_z));
    memcpy(&rc->left_button_down, &custom_rx_data[6], sizeof(rc->keyboard_value));
    memcpy(&rc->right_button_down, &custom_rx_data[7], sizeof(rc->keyboard_value));
    memcpy(&rc->keyboard_value, &custom_rx_data[8], sizeof(rc->keyboard_value));
    // 12以后的数据由于缓冲区大小不匹配，请不要读取使用
}

/**
 * @brief          自定义控制器角度数据重新映射，这个函数由于大臂与小臂限位并不相同而存在
 *                 同时这一步也有限幅的的效果
 *                 已经在下位机处理完毕，此函数不再使用
 * @param[in]      custom_angle_set  大臂角度设定值
 *                 custom_data_raw  自定义控制器角度原始数据（小臂角度值）
 * @retval         none
 */
static void custom_angle_mapping(float custom_angle_set[6], float custom_data_raw[6])
{
    /* 进行转化 */

}


/**
 * @brief          自定义控制器通信数据解析
 * @param[in]      Custom_Data_t *custom
 * @retval         1:解析成功
 *                 0:解析失败
 */
//static bool custom_data_solve(Custom_Data_t *custom)
//{
//    /* 检查帧头 */
//    if(custom->frame_header.sof == 0xA5)
//    {
//        /* 检查数据帧长度 */
//        if(custom->frame_header.data_length == 30)
//        {
//            /* 检查帧头帧尾CRC校验 */
//            if(verify_CRC8_check_sum((uint8_t *)(&custom->frame_header),sizeof(custom->frame_header)) &&
//               verify_CRC16_check_sum((uint8_t *)(&custom),CUSTOM_QUEUE_ITEM_SIZE) )
//            {
//                /* 角度数据解析 */
//                custom_data_analysis(custom_controller.custom_angle_set,custom->data);
//
//                return 1;
//            }
//            else
//            {
//                return 0;
//            }
//        }
//        else
//        {
//            return 0;
//        }
//    }
//    else
//    {
//        return 0;
//    }
//}


/**
 * @brief          自定义控制器通信数据解析
 * @param[in]      Custom_Data_t *custom
 * @retval         1:解析成功
 *                 0:解析失败
 */
static bool custom_data_solve(Custom_Data_t *custom)
{
    /* 检查帧头 */
    if(custom->frame_header.sof == 0xA5)
    {
        /* 检查数据帧长度 */
        if(custom->frame_header.data_length == 30)
        {
            /* 检查命令码 */
            if(custom->cmd_id == 0x0302)
            {
                /* 角度数据解析 */
                custom_data_analysis(custom_controller.custom_angle_set,custom->data);

                /* 角度数据重新映射 */
//                custom_angle_mapping(custom_controller.custom_angle_set,custom_controller.custom_data_raw);
                return 1;
            }

        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}


/**
 * @brief          图传链路键鼠通信数据解析
 * @param[in]      Remote_Data_t *rc
 * @retval         1:解析成功
 *                 0:解析失败
 */
static bool rc_data_solve(Custom_Data_t *rc)
{
    /* 检查帧头 */
    if(rc->frame_header.sof == 0xA5)
    {
        /* 检查数据帧长度 */
        if(rc->frame_header.data_length == 12)
        {
            /* 检查命令码 */
            if(rc->cmd_id == 0x0304)
            {
                // 键鼠数据解析
                rc_data_analysis(&RC_controller, rc->data);
                return 1;
            }

        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}


/**
 * @brief 数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param data 数据段的数组指针
 * @param data_lenth 数据段长度
 */
static void Data_Concatenation(uint8_t *data, uint16_t data_lenth)
{
    static uint8_t seq = 0;
    /// 帧头数据
    tx_data.frame_header.sof = 0xA5;                              // 数据帧起始字节，固定值为 0xA5
    tx_data.frame_header.data_length = data_lenth;                // 数据帧中数据段的长度
    tx_data.frame_header.seq = seq++;                             // 包序号
    append_CRC8_check_sum((uint8_t *)(&tx_data.frame_header), 5); // 添加帧头 CRC8 校验位
    /// 命令码ID
    tx_data.cmd_id = CONTROLLER_CMD_ID;
    /// 数据段
    memcpy(tx_data.data, data, data_lenth);
    /// 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(&tx_data), DATA_FRAME_LENGTH);
}


/**
 * @brief          发送控制信息给自定义控制器，主要是模式切换，替换掉原先的DT7控制
 * @param[in]      none
 * @retval         custom_controller的指针
 */
static void custom_data_transmit(void)
{
    /* 自定义控制器进入MEC_ARM_MASTER_MODE模式 */
    if(mec_arm_control.mec_arm_mode == MEC_ARM_CUSTOM_MODE)
    {
        info_data[0] = 1;
    }
    /* 自定义控制器进入MEC_ARM_ZERO_POSITION_MODE模式 */
    else
    {
        info_data[0] = 0;
//        memset(info_data, 0, sizeof(info_data));
    }
    Data_Concatenation(info_data, DATA_LENGTH);
    HAL_UART_Transmit_DMA(&huart7, (uint8_t *)(&tx_data), sizeof(tx_data));
//    osDelay(35);
}


/**
 * @brief          返回自定义控制器数组结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
Custom_Controller_t *Get_Custom_Controller_Point(void)
{
    return &custom_controller;
}


/**
 * @brief          返回图传链路数据结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
Remote_Controller_t *Get_Remote_Control_0x304_Point(void)
{
    return &RC_controller;
}