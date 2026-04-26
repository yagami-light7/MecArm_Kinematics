/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       App_CV_Control.c
  * @brief      Light
  * @note       与上位机进行通信获取视觉下发的编码值，进行视觉控制兑矿
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Feb-12-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

#include "App_CV_Control.h"
#include <stdio.h>

#include "Board_CRC8_CRC16.h"
#include "App_Gimbal_Task.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_usart5_rx;

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_usart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;

extern UART_HandleTypeDef huart10;
extern DMA_HandleTypeDef hdma_usart10_rx;
extern DMA_HandleTypeDef hdma_usart10_tx;


/* 函数声明 */
/**
 * @brief          视觉通信接收与解析任务初始化
 * @param[in]      none
 * @retval         none
 */
static void CV_Init(void);


/**
 * @brief          视觉通信数据解析
 * @param[in]      CV_Data_t *custom
 * @retval         1:解析成功
 *                 0:解析失败
 */
static bool cv_data_solve(CV_Data_t *custom);


/* 变量声明 */
CV_Controller_t cv_controller;


/**
 * @brief          自定义控制器通信任务
 * @param[in]      none
 * @retval         none
 */
void CV_Control_Task(void *pvParameters)
{
    /* 延迟一段时间以待底层初始化 */
    vTaskDelay(CUSTOM_TASK_INIT_TIME);

    /* 初始化 */
    CV_Init();

    /* 是否阻塞该任务 */

#if _CV_USED_ // 使用视觉兑矿
    // 不进行操作
#else
    // 删除任务
    vTaskDelete(NULL);
#endif

    while(1)
    {
        /* 读取队列数据 */
        QueueHandle_t xCV_Queue = xGet_CV_Queue();
        CV_Data_t cv_info_R;

        /* 进行数据解析 */
        if(pdPASS == xQueueReceive(xCV_Queue, &cv_info_R, 0))
        {
            if(cv_data_solve(&cv_info_R) == 1)
            {
                /* 解析成功进行角度值同步设定 */
                // 为提升代码内聚性，这一步代码放在其他任务中执行
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
static void CV_Init(void)
{
    for (int i = 0; i < JOINT_NUM; i++)
    {
        cv_controller.cv_angle_set[i] = 0;
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
 * @brief          视觉角度数据解析
 * @param[in]      custom_angle    自定义控制器角度原始数据（小臂角度值）
 *                 custom_rx_data  串口收到的数据
 * @retval         none
 */
static void cv_data_analysis(float *cv_angle, uint8_t *cv_rx_data)
{
    /* 进行转化 */
    uint8_to_float(cv_rx_data, cv_angle, JOINT_NUM);
}


/**
 * @brief          视觉通信数据解析
 * @param[in]      CV_Data_t *custom
 * @retval         1:解析成功
 *                 0:解析失败
 */
static bool cv_data_solve(CV_Data_t *cv)
{
    /* 检查帧头 */
    if(cv->sof == 0xA5)
    {
        /* 检查帧尾CRC校验 */
//        if(verify_CRC16_check_sum((uint8_t *)(&cv),CV_QUEUE_ITEM_SIZE) )
//        {
            /* 角度数据解析 */
            cv_data_analysis(cv_controller.cv_angle_set,cv->data);

//            return 1;
//        }
//        else
//        {
//            return 0;
//        }

    }
    else
    {
        return 0;
    }
}

/**
 * @brief          返回自定义控制器数组结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
CV_Controller_t *Get_CV_Controller_Pointer(void)
{
    return &cv_controller;
}




