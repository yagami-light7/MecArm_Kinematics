/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       App_UI_Task.c
  * @brief      裁判系统自定义UI
  * @note       使用南航UI设计器
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Mar-16-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  * 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

#include "App_UI_Task.h"
#include "App_Gimbal_Task.h"
#include "App_Detect_Task.h"
#include "App_Custom_Task.h"


extern gimbal_control_t gimbal_control;
/**
 * @brief          初始化UI
 * @param[in]      none
 * @retval         none
 */
static void UI_init(void);

/**
 * @brief          更新UI 只需要更新动态元素UI
 * @param[in]      none
 * @retval         none
 */
static void UI_Update(void);

/**
 * @brief          重新初始化UI任务
 * @param[in]      none
 * @retval         none
 */
static void UI_Task_Reset(void);


/**
 * @brief          UI任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void UI_Task(void *pvParameters)
{
    /* 初始化UI */
    UI_init();

    while(1)
    {
        UI_Update();

        /* 手动刷新UI界面 */
        if(gimbal_control.vt_rc_control->key & KEY_PRESSED_OFFSET_CTRL) // CTRL
        {
            if( (gimbal_control.vt_rc_control->key & KEY_PRESSED_OFFSET_X) ) // X键按下
            {
                UI_Task_Reset();
            }
        }

        vTaskDelay(UI_CONTROL_TIME_MS);


    }

}

/**
 * @brief          初始化UI
 * @param[in]      none
 * @retval         none
 */
static void UI_init(void)
{
    ui_init_g_Dynamic();
    vTaskDelay(50);
    ui_init_g_Static();
    osDelay(50);
}

/**
 * @brief          更新UI 只需要更新动态元素UI
 * @param[in]      none
 * @retval         none
 */
static void UI_Update(void)
{
    /* 气泵 UI修改 */
    if(pump_master_is_on)
    {
        ui_g_Dynamic_sucker_master_on->color = GREEN_COLOR;
    }
    else
    {
        ui_g_Dynamic_sucker_master_on->color  = PINK_COLOR;
    }

    if (sucker_left_is_on)
    {
        ui_g_Dynamic_sucker_slave_left_on->color = GREEN_COLOR;
    }
    else
    {
        ui_g_Dynamic_sucker_slave_left_on->color = PINK_COLOR;
    }

    if (sucker_right_is_on)
    {
        ui_g_Dynamic_sucker_slave_right_on->color = GREEN_COLOR;
    }
    else
    {
        ui_g_Dynamic_sucker_slave_right_on->color = PINK_COLOR;
    }


    /* 机械臂模式UI修改 */
    if(gimbal_control.gimbal_motor_mode  == GIMBAL_CUSTOM_MODE)
    {
        ui_g_Dynamic_CUSTOM_ON->color = GREEN_COLOR;
        osDelay(2);
        strcpy(ui_g_Dynamic_CUSTOM_ON->string, "ON");
    }
    else
    {
        ui_g_Dynamic_CUSTOM_ON->color = PINK_COLOR;
        osDelay(2);
        strcpy(ui_g_Dynamic_CUSTOM_ON->string, "OFF");
    }

    if(gimbal_control.gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE || gimbal_control.gimbal_motor_mode  == GIMBAL_AUTO_GET_L_MINE_MODE)
    {
        ui_g_Dynamic_AUTO_ON->color = GREEN_COLOR;
        osDelay(2);
        strcpy(ui_g_Dynamic_AUTO_ON->string, "ON");
    }
    else
    {
        ui_g_Dynamic_CUSTOM_ON->color = PINK_COLOR;
        osDelay(2);
        strcpy(ui_g_Dynamic_AUTO_ON->string, "OFF");
    }

    if(toe_is_error(CUSTOM_TOE) == 0)
    {
        ui_g_Dynamic_CUSTOM_ENABLE->color = LIGHT_BLUE_COLOR;
        osDelay(2);
        strcpy(ui_g_Dynamic_CUSTOM_ENABLE->string, "CUSTOM_ENABLE");
    }
    else
    {
        ui_g_Dynamic_CUSTOM_ENABLE->color = PINK_COLOR;
        osDelay(2);
        strcpy(ui_g_Dynamic_CUSTOM_ENABLE->string, "CUSTOM_DISABLE");
    }

    /* DT7 UI修改 */
//    if(toe_is_error(DBUS_TOE))
//    {
//        ui_g_Dynamic_DT7->color = PINK_COLOR;
//        strcpy(ui_g_Dynamic_DT7->string, "DT7_DISCONNECTED");
//
//    }
//    else
//    {
//        ui_g_Dynamic_DT7->color = GREEN_COLOR;
//        strcpy(ui_g_Dynamic_DT7->string, "DT7_CONNECTED");
//    }

    ui_update_g_Dynamic();

}


/**
 * @brief          重新初始化UI任务
 * @param[in]      none
 * @retval         none
 */
static void UI_Task_Reset(void)
{
    UI_init();
    osDelay(50);
//    UI_Update();
//    osDelay(10);
}