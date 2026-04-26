#ifndef ENGINEERING_APP_CV_CONTROL_H
#define ENGINEERING_APP_CV_CONTROL_H

#include "main.h"
#include "cmsis_os.h"
#include "Dev_CV.h"

#define CV_TASK_INIT_TIME    201 // 任务初始化时间
#define CV_TASK_CONTROL_TIME 1  // 任务控制时间

#define JOINT_NUM 6

typedef struct
{
    /* 原始 上位机回传数据 */
    float cv_data_raw[6];
    /* 最终 关节电机的角度设定值 */
    float cv_angle_set[6];

} CV_Controller_t;

/**
 * @brief          视觉控制任务
 * @param[in]      none
 * @retval         none
 */
extern void CV_Control_Task(void *pvParameters);

/**
 * @brief          返回视觉控制结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
extern CV_Controller_t *Get_CV_Controller_Pointer(void);

#endif