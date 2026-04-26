/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"

#define JOINT1_CALI_TORQ       0.0f // 校准时给J1下发的扭矩
#define JOINT1_CALI_Velocity  -4.0f // 校准时给J1下发的速度

#define JOINT2_CALI_TORQ       0.0f // 校准时给J2下发的扭矩
#define JOINT2_CALI_Velocity   2.0f // 校准时给J2下发的速度

#define _CHASSIS_RESET_ID_  0 // 需要进行底盘ID重置1  不需要0
#define CAR_Omnidirectional 0 // 麦轮0 全向轮1
#define _CUSTOM_USED_ 1       // 机械臂交由 DT7控制0  自定义控制器控制1
#define _CV_USED_     0       // 是否使用视觉控制机械臂 使用1 不使用0

/* 控制主吸盘气路的电磁阀 电平与吸盘相反*/
#define contray_pump_master_control    GPIOA->BSRR = (GPIOE->ODR & GPIO_ODR_OD13) ? GPIO_BSRR_BR2 : GPIO_BSRR_BS2;


/* 控制吸盘 */
#define pump_master_on       HAL_GPIO_WritePin(pump_master_GPIO_Port, pump_master_Pin, GPIO_PIN_SET); \
                             contray_pump_master_control //打开吸盘


#define pump_master_off      HAL_GPIO_WritePin(pump_master_GPIO_Port, pump_master_Pin, GPIO_PIN_RESET);\
                             contray_pump_master_control //关闭吸盘

#define pump_master_toggle   HAL_GPIO_TogglePin(pump_master_GPIO_Port, pump_master_Pin);\
                             contray_pump_master_control //翻转吸盘电平

#define pump_master_is_on    GPIOE->ODR & GPIO_ODR_OD13

/* 控制气泵二 */
#define pump_slave_on        HAL_GPIO_WritePin(pump_slave_GPIO_Port, pump_slave_Pin, GPIO_PIN_SET)          //打开气泵
#define pump_slave_off       HAL_GPIO_WritePin(pump_slave_GPIO_Port, pump_slave_Pin, GPIO_PIN_RESET)        //关闭气泵
#define pump_slave_toggle    HAL_GPIO_TogglePin(pump_slave_GPIO_Port, pump_slave_Pin)

#define pump_slave_control   GPIOE->BSRR = ( (GPIOC->ODR & GPIO_ODR_OD13) | (GPIOC->ODR & GPIO_ODR_OD14) )? GPIO_BSRR_BS9 : GPIO_BSRR_BR9;

/* 控制两侧吸盘(电磁阀) */
#define sucker_left_on        HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_SET); \
                              pump_slave_control

#define sucker_right_on       HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_SET); \
                              pump_slave_control

#define sucker_left_off       HAL_GPIO_WritePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin, GPIO_PIN_RESET); \
                              pump_slave_control

#define sucker_right_off      HAL_GPIO_WritePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin, GPIO_PIN_RESET); \
                              pump_slave_control

#define sucker_left_toggle       HAL_GPIO_TogglePin(POWER_24V_1_GPIO_Port, POWER_24V_1_Pin); \
                                 pump_slave_control

#define sucker_right_toggle      HAL_GPIO_TogglePin(POWER_24V_2_GPIO_Port, POWER_24V_2_Pin); \
                                 pump_slave_control
#define sucker_left_control     GPIOC->BSRR = (GPIOE->ODR & GPIO_ODR_OD13) ? GPIO_BSRR_BR14 : GPIO_BSRR_BS14;

#define sucker_right_control    GPIOC->BSRR = (GPIOE->ODR & GPIO_ODR_OD13) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;

#define pump_slave_is_on      GPIOE->ODR & GPIO_ODR_OD9
#define sucker_left_is_on     GPIOC->ODR & GPIO_ODR_OD14
#define sucker_right_is_on    GPIOC->ODR & GPIO_ODR_OD13


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_24V_2_Pin GPIO_PIN_13
#define POWER_24V_2_GPIO_Port GPIOC
#define POWER_24V_1_Pin GPIO_PIN_14
#define POWER_24V_1_GPIO_Port GPIOC
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define CS1_ACCEL_Pin GPIO_PIN_0
#define CS1_ACCEL_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_3
#define CS1_GYRO_GPIO_Port GPIOC
#define sucker_right_Pin GPIO_PIN_0
#define sucker_right_GPIO_Port GPIOA
#define contray_pump_master_Pin GPIO_PIN_2
#define contray_pump_master_GPIO_Port GPIOA
#define ADC_BAT_Pin GPIO_PIN_4
#define ADC_BAT_GPIO_Port GPIOC
#define pump_slave_Pin GPIO_PIN_9
#define pump_slave_GPIO_Port GPIOE
#define INT1_ACCEL_Pin GPIO_PIN_10
#define INT1_ACCEL_GPIO_Port GPIOE
#define INT1_ACCEL_EXTI_IRQn EXTI15_10_IRQn
#define INT1_GYRO_Pin GPIO_PIN_12
#define INT1_GYRO_GPIO_Port GPIOE
#define INT1_GYRO_EXTI_IRQn EXTI15_10_IRQn
#define pump_master_Pin GPIO_PIN_13
#define pump_master_GPIO_Port GPIOE
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
