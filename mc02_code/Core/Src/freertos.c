/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"

#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "HAL_FDCAN.h"
#include "Board_Delay.h"
#include "Board_DWT.h"

#include "Dev_BMI088_Driver.h"
#include "Dev_Remote_Control.h"
#include "Dev_FDCAN.h"
#include "Dev_Custom.h"

#include "App_Buzzer_Task.h"
#include "App_Calibrate_Task.h"
#include "App_Chassis_Task.h"
#include "App_Detect_Task.h"
#include "App_Mechanical_Arm_Task.h"
#include "App_INS_Task.h"
#include "App_LED_RGB_Flow_Task.h"
#include "App_Custom_Task.h"
#include "App_CV_Control.h"
#include "App_Referee_Task.h"
#include "App_UI_Task.h"

#include "APL_RC_Hub.h"
#include "APL_MecArm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId testHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void test_task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
    *ppxTimerTaskStackBuffer = &xTimerStack[0];
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
    /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

    // 初始化FDCAN
    FDCAN_Filter_Init();

//    DWT_Init(480);
//    BMI088_Init(&hspi2, 0);
//    // 电机初始化
//    MIT_Motor_Init();
//#if _CHASSIS_RESET_ID_
//    FDCAN_Cmd_Chassis_Reset_ID();
//#endif
////    Cali_Param_Init();
//
//    Remote_Control_Init();
////    Custom_Control_Init();
////    CV_Control_Init();
//    Referee_Init();
//
//
//    /* 上电默认关闭气泵 */
//    pump_master_off;
//    pump_slave_off;
//    sucker_left_off;
//    sucker_right_off;

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of test */
  osThreadDef(test, test_task, osPriorityNormal, 0, 128);
  testHandle = osThreadCreate(osThread(test), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  // 创建机械臂线程
    xTaskCreate(_Thread_MecArm_, "_Thread_MecArm_", 512, NULL, osPriorityHigh, NULL);

//   FDCAN_Cmd_Chassis_Reset_ID();

    /* 创建检测任务 */
//    xTaskCreate(Detect_Task, "Detect_Task", 512, NULL, osPriorityHigh, NULL);
//
//    /* 创建操作任务 */
//    xTaskCreate(Operator_Task, "Operator_Task", 128, NULL, osPriorityHigh+1, NULL);

    /* 创建视觉控制任务 */
//    xTaskCreate(CV_Control_Task, "CV_Control_Task", 512, NULL, osPriorityNormal, NULL);

    /* 创键与自定义控制器通信任务 */
//    xTaskCreate(Custom_Controller_Task, "Custom_Controller_Task", 256, NULL, osPriorityAboveNormal, NULL);

//    /* 创建机械臂小关节任务 */
//    xTaskCreate(Mechanical_Arm_Task, "Mechanical_Arm_Task", 512, NULL, osPriorityHigh, NULL);
//
//    /* 创建底盘任务 */
//    xTaskCreate(Chassis_Task, "Chassis_Task", 512, NULL, osPriorityNormal, NULL);
//
//    /* 创建云台/机械臂大关节任务 */
//    xTaskCreate(Gimbal_Task, "Gimbal_Task", 512, NULL, osPriorityHigh, NULL);
//
//    /* 创建IMU解算任务 */
//    xTaskCreate(INS_Task, "INS_Task", 1024, NULL, osPriorityRealtime, NULL);
//
//    /* 创建校准任务 */
//    xTaskCreate(Calibrate_Task, "Calibrate_Task", 512, NULL, osPriorityNormal, NULL);
//
//    /* 创建LED闪烁任务 */
//    xTaskCreate(LED_RGB_Flow_Task, "LED_RGB_Flow_Task", 256, NULL, osPriorityNormal, NULL);
//
//    /* 创建蜂鸣器报警任务 */
//    xTaskCreate(Buzzer_Task, "Buzzer_Task", 128, NULL, osPriorityNormal, NULL);
//
//    /* 创建裁判系统数据解析任务 */
//    xTaskCreate(Referee_Task, "Referee_Task", 512, NULL, osPriorityNormal, NULL);
//
//    /* 创建UI任务 */
//    xTaskCreate(UI_Task, "UI_Task", 256, NULL, osPriorityAboveNormal, NULL);
//
    /* 创建数据解析任务 */
    xTaskCreate(_Thread_RC_Hub_, "_Thread_RC_Hub_", 256, NULL, osPriorityHigh, NULL);

//    /* 创建电压采样任务 */
//    xTaskCreate(Battery_Voltage_Task, "Battery_Voltage_Task", 128, NULL, osPriorityNormal, NULL);

  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
__weak void test_task(void const * argument)
{
  /* USER CODE BEGIN test_task */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
    }
  /* USER CODE END test_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
