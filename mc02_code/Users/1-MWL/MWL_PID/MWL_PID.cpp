/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_PID.cpp
  * @brief      PID 中间件实现文件
  * @note       Middleware Layer 中间件层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-13-2026     Codex           1. create
  *
  @verbatim
  ==============================================================================
  * 本文件的算法逻辑保持与旧 Alg_PID.c 一致，只是改成了类封装形式，
  * 方便新框架在 APL/HDL 层直接持有 PID 对象。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#include "MWL_PID.h"

#include <math.h>
#include <string.h>

namespace
{
const float kPi = 3.14159265358979323846f;
const float kTwoPi = 6.28318530717958647692f;

float MWL_PID_Clamp(float input, float max_value)
{
    if (input > max_value)
    {
        return max_value;
    }

    if (input < -max_value)
    {
        return -max_value;
    }

    return input;
}

float MWL_PID_Rad_Format(float angle)
{
    while (angle > kPi)
    {
        angle -= kTwoPi;
    }

    while (angle < -kPi)
    {
        angle += kTwoPi;
    }

    return angle;
}
}

Class_PID::Class_PID()
{
    memset(&state_, 0, sizeof(state_));
}

void Class_PID::Init(uint8_t mode, const float pid[3], float max_out, float max_iout)
{
    if (pid == NULL)
    {
        return;
    }

    Init(mode, pid[0], pid[1], pid[2], max_out, max_iout);
}

void Class_PID::Init(uint8_t mode, float kp, float ki, float kd, float max_out, float max_iout)
{
    memset(&state_, 0, sizeof(state_));

    state_.mode = mode;
    state_.kp = kp;
    state_.ki = ki;
    state_.kd = kd;
    state_.max_out = max_out;
    state_.max_iout = max_iout;
}

float Class_PID::Calc(float ref, float set)
{
    state_.error[2] = state_.error[1];
    state_.error[1] = state_.error[0];
    state_.set = set;
    state_.fdb = ref;
    state_.error[0] = set - ref;

    if (state_.mode == MWL_PID_POSITION)
    {
        state_.pout = state_.kp * state_.error[0];
        state_.iout += state_.ki * state_.error[0];
        state_.dbuf[2] = state_.dbuf[1];
        state_.dbuf[1] = state_.dbuf[0];
        state_.dbuf[0] = state_.error[0] - state_.error[1];
        state_.dout = state_.kd * state_.dbuf[0];
        state_.iout = MWL_PID_Clamp(state_.iout, state_.max_iout);
        state_.out = state_.pout + state_.iout + state_.dout;
        state_.out = MWL_PID_Clamp(state_.out, state_.max_out);
    }
    else if (state_.mode == MWL_PID_INCREMENTAL)
    {
        state_.pout = state_.kp * (state_.error[0] - state_.error[1]);
        state_.iout = state_.ki * state_.error[0];
        state_.dbuf[2] = state_.dbuf[1];
        state_.dbuf[1] = state_.dbuf[0];
        state_.dbuf[0] = state_.error[0] - 2.0f * state_.error[1] + state_.error[2];
        state_.dout = state_.kd * state_.dbuf[0];
        state_.out += state_.pout + state_.iout + state_.dout;
        state_.out = MWL_PID_Clamp(state_.out, state_.max_out);
    }

    return state_.out;
}

float Class_PID::CalcRad(float ref, float set, float error_delta)
{
    float error = 0.0f;

    state_.fdb = ref;
    state_.set = set;
    error = MWL_PID_Rad_Format(set - ref);
    state_.error[0] = error;

    state_.pout = state_.kp * state_.error[0];
    state_.iout += state_.ki * state_.error[0];
    state_.dout = state_.kd * error_delta;
    state_.iout = MWL_PID_Clamp(state_.iout, state_.max_iout);
    state_.out = state_.pout + state_.iout + state_.dout;
    state_.out = MWL_PID_Clamp(state_.out, state_.max_out);

    return state_.out;
}

void Class_PID::Clear(void)
{
    state_.error[0] = 0.0f;
    state_.error[1] = 0.0f;
    state_.error[2] = 0.0f;

    state_.dbuf[0] = 0.0f;
    state_.dbuf[1] = 0.0f;
    state_.dbuf[2] = 0.0f;

    state_.out = 0.0f;
    state_.pout = 0.0f;
    state_.iout = 0.0f;
    state_.dout = 0.0f;
    state_.fdb = 0.0f;
    state_.set = 0.0f;
}

void Class_PID::Change(float kp, float ki, float kd)
{
    state_.kp = kp;
    state_.ki = ki;
    state_.kd = kd;
}

const MWL_PID_State_t &Class_PID::GetState(void) const
{
    return state_;
}
