/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       led_trigger_task.c/h
  * @brief      led RGB show.led RGB灯效。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. rgb led
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef INFANTRY_ROBOT_APP_LED_RGB_FLOW_TASK_H
#define INFANTRY_ROBOT_APP_LED_RGB_FLOW_TASK_H

#include "main.h"

#define LED_DELAY_TIME	10

/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void LED_RGB_Flow_Task(void *pvParameters);

#endif



