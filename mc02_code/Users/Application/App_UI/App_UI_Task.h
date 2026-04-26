#ifndef ENGINEERING_APP_UI_TASK_H
#define ENGINEERING_APP_UI_TASK_H

#include "main.h"
#include "ui.h"
#include "cmsis_os.h"

#define UI_CONTROL_TIME_MS   2

#define PINK_COLOR              5
#define GREEN_COLOR             2
#define LIGHT_BLUE_COLOR        6
/**
 * @brief          UI任务
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void UI_Task(void *pvParameters);


#endif
