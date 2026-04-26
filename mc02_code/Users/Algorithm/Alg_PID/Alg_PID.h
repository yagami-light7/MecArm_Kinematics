/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid实现函数，包括初始化，PID计算函数，
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef __ALG_PID_H__
#define __ALG_PID_H__

#include "main.h"

typedef enum PID_Mode
{
	PID_POSITION = 0,
	PID_Incremental = 1
	
} PID_Mode_e;

typedef struct 
{
	uint8_t mode;
	//PID 三参数
	float Kp;
	float Ki;
	float Kd;

	float max_out;  //最大输出
	float max_iout; //最大积分输出

	float set;
	float fdb;

	float out;
	float Pout;
	float Iout;
	float Dout;
	float Dbuf[3];  //微分项 0最新 1上一次 2上上次
	float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef_t;//


#ifdef __cplusplus
extern "C"{
#endif

extern void PID_init(PidTypeDef_t *pid, uint8_t mode, const float PID[3], float max_out, float max_iout);

extern float PID_Calc(PidTypeDef_t *pid, float ref, float set);

extern void PID_Change(PidTypeDef_t *pid, float Kp, float Ki, float Kd);

extern void PID_clear(PidTypeDef_t *pid);

extern float IMU_PID_Calc(PidTypeDef_t* pid, float ref, float set, float error_delta);

#ifdef __cplusplus
}
#endif

#endif
