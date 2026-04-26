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

#include "Alg_PID.h"
#include "Alg_UserLib.h"

/*---------------------------------------------------------------
 * Function:        这其实是个宏
 * Description:     对需要的值进行限幅操作，这里是I的限幅和PID控制器最后输出的限幅
 * Input:           input				想要进行限幅的值  
                    max					限幅值
 * Output:          NULL
---------------------------------------------------------------*/
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/*---------------------------------------------------------------
 * Function: 		PID_init(...)
 * Description:     PID初始化函数
 * Input:           pid				    相应PID控制器的结构体指针
                    mode				0：Position位置式PID  1：delta增量式PID
                    max_out				对PID控制器最后输出的限幅值
                    max_iout			对PID控制器运算中Iout的限幅值
 * Output:          NULL
---------------------------------------------------------------*/
void PID_init(PidTypeDef_t *pid, uint8_t mode, const float PID[3], float max_out, float max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}

/*---------------------------------------------------------------
 * Function:        float PID_Calc(...)
 * Description:     PID控制器运算函数
 * Input:           pid						相应PID控制器的结构体指针
                    ref						当前回传的实际值
                    set						设定值/期望值
 * Output:          PID控制器的输出值
---------------------------------------------------------------*/
float PID_Calc(PidTypeDef_t *pid, float ref, float set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
		
		
    if (pid->mode == PID_POSITION)			//采用位移式PID
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_Incremental)		//采用增量式PID
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
		
    return pid->out;
}

/*---------------------------------------------------------------
 * Function:        float IMU_PID_Calc(...)
 * Description:     PID控制器运算函数(针对角度环)
 * Input:           pid						相应PID控制器的结构体指针
										ref						当前回传的实际值
										set						设定值/期望值
										error_delta   两次误差之差：此处是角速度 
 * Output:          PID控制器的输出值
---------------------------------------------------------------*/
float IMU_PID_Calc(PidTypeDef_t* pid, float ref, float set, float error_delta)
{
	float err;
	
	if (pid == NULL)
	{
        return 0.0f;
	}
	
	pid->fdb = ref;
	pid->set = set;

	err = set - ref;
	pid->error[0] = rad_format(err); //利用循环限幅函数将误差限制在[-PI，PI]
	pid->Pout = pid->Kp * pid->error[0];
	pid->Iout += pid->Ki * pid->error[0];
	pid->Dout = pid->Kd * error_delta;
	abs_limit(&pid->Iout, pid->max_iout);
	pid->out = pid->Pout + pid->Iout + pid->Dout;
	abs_limit(&pid->out, pid->max_out);
	return pid->out;
}

/*---------------------------------------------------------------
 * Function:        PID_clear(PidTypeDef_t *pid)
 * Description:     PID复位（清零）函数
 * Input:           pid	相应PID控制器的结构体指针
 * Output:          NULL
---------------------------------------------------------------*/
void PID_clear(PidTypeDef_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

//修改pid数值
void PID_Change(PidTypeDef_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}