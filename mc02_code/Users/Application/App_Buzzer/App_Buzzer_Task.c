/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       test_task.c/h
  * @brief      buzzer warning task.蜂鸣器报警任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "App_Buzzer_Task.h"
#include "cmsis_os.h"

#include "Board_Buzzer.h"

#include "App_Detect_Task.h"

static void buzzer_warn_error(uint8_t num);

static void play_startupsound(void);
static void low_power_warning(void);


const error_t *error_list_test_local;

/**
 * @brief          蜂鸣器任务 播放开机音乐 错误报警
 * @param[in]      pvParameters: NULL
 * @retval         none
 */
void Buzzer_Task(void *pvParameters)
{
    static uint8_t error, last_error;
    static uint8_t error_num;
//    error_list_test_local = get_error_list_point();

    /* 播放开机音乐 */
    play_startupsound();
    osDelay(1000);

    while (1)
    {
//        error = 0;
//
//        /* 电量低于 5% 时触发低电量报警 */
//         if (electricity_percentage < 0.05)
//         {
//             // low_power_warning();
//         }
//
//        /* 发现错误 */
//        for (error_num = 0; error_num < ERROR_LIST_LENGHT; error_num++)
//        {
//            if (error_list_test_local[error_num].error_exist)
//            {
//                error = 1;
//                break;
//            }
//        }
//
//        /* 无错误，蜂鸣器停止报警 */
//        if (error == 0 && last_error != 0)
//        {
//            Buzzer_OFF();
//        }
//
//        /* 有错误：蜂鸣器次数代表对应错误 */
//        if (error)
//        {
//            //buzzer_warn_error(error_num + 1);
//        }
//
//        last_error = error;
        osDelay(10);
    }
}

/**
 * @brief          使得蜂鸣器响
 * @param[in]      num:响声次数
 * @retval         none
 */
static void buzzer_warn_error(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if (show_num == 0 && stop_num == 0)
    {
        show_num = num;
        stop_num = 100;
    }
    else if (show_num == 0)
    {
        stop_num--;
        Buzzer_OFF();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if (tick < 50)
        {
            Buzzer_OFF();
        }
        else if (tick < 100)
        {
            Buzzer_On(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}

// 函数：播放开机音效
void play_startupsound(void)
{
    Buzzer_On(100, 1500);
    osDelay(200);

    Buzzer_OFF();

    Buzzer_On(200, 1500);
    osDelay(200);

    Buzzer_OFF();

    Buzzer_On(150, 1500);
    osDelay(200);

    Buzzer_OFF();

    Buzzer_On(100, 1500);
    osDelay(200);

    Buzzer_OFF();

    Buzzer_On(50, 1500);
    osDelay(200);

    Buzzer_OFF();

    osDelay(500);

    Buzzer_On(300, 1000);
    osDelay(200);

    Buzzer_OFF();
}

/* 低电量报警 */
void low_power_warning(void)
{
    Buzzer_On(1.0, 30000);
    osDelay(500);
    Buzzer_OFF();
    osDelay(500);
    Buzzer_On(0.5, 30000);
    osDelay(500);
    Buzzer_OFF();
    osDelay(500);
}
