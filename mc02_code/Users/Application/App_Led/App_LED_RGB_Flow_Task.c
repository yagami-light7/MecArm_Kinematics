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
#include "App_LED_RGB_Flow_Task.h"
#include "Board_LED.h"
#include "cmsis_os.h"

/**
  * @brief          led RGB任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void LED_RGB_Flow_Task(void *pvParameters)
{
    while(1)
    {
        uint8_t red = 1;
        uint8_t green = 1;
        uint8_t blue = 1;

        while(1)
        {
            WS2812_Ctrl(red, green, blue);
            red++;
            green = 100 - red;
            blue += 100 - red - green;
            vTaskDelay(LED_DELAY_TIME);
            red++;
            green++;
            blue++;
        }
    }
}


