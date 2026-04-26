/**
  ****************************(C) COPYRIGHT 2025 ROBOT-Z****************************
  * @file       INS_task.c/h
  * @brief      2025赛季平衡步兵欧拉角计算任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dev-20-2024     冉文治           1. 添加卡尔曼滤波对零漂进行处理
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 ROBOT-Z****************************
  */

#ifndef INS_Task_H
#define INS_Task_H

#include "main.h"

#define ACCELLPF                        0.0085f

// 任务间隔
#define IMU_CONTROL_TIME_MS             2

#define INS_YAW_ADDRESS_OFFSET          0
#define INS_PITCH_ADDRESS_OFFSET        1
#define INS_ROLL_ADDRESS_OFFSET         2
#define INS_YAW_TOTAL_ADDRESS_OFFSET		3

#define INS_GYRO_X_ADDRESS_OFFSET 			0
#define INS_GYRO_Y_ADDRESS_OFFSET 			1
#define INS_GYRO_Z_ADDRESS_OFFSET 			2

#define INS_ACCEL_X_ADDRESS_OFFSET 			0
#define INS_ACCEL_Y_ADDRESS_OFFSET 			1
#define INS_ACCEL_Z_ADDRESS_OFFSET 			2

// BMI088控制温度的设置TIM的重载值，即给PWM最大为 BMI088_TEMP_PWM_MAX - 1
#define BMI088_TEMP_PWM_MAX 	    			    10000

// 温度控制PID
#define TEMPERATURE_PID_KP 							1600.0f
#define TEMPERATURE_PID_KI 							0.2f
#define TEMPERATURE_PID_KD 							0.0f
#define TEMPERATURE_PID_MAX_OUT   			        9500.0f
#define TEMPERATURE_PID_MAX_IOUT 				    9400.0f


#ifdef __cplusplus
extern "C"{
#endif

/**
  * @brief          imu任务, 初始化 bmi088, 计算欧拉角
  * @author         冉文治
  * @param          pvParameters: NULL
  * @retval         none
  */
extern void INS_Task(void *pvParameters);

/**
  * @brief          获取四元数
  * @author         冉文治
  * @param          none
  * @retval         INS_quat的指针
  */
extern const float *get_INS_quat_point(void);

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴, 3:total_yaw 单位 rad/s
  * @author         冉文治
  * @param          none
  * @retval         INS_gyro的指针
  */
extern const float *get_INS_angle_point(void);

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         INS_accel的指针
  */
extern const float *get_gyro_data_point(void);

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         INS_accel的指针
  */
extern const float *get_accel_data_point(void);

#ifdef __cplusplus
}
#endif


#endif
