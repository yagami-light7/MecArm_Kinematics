/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       Alg_Utilities.cpp
  * @brief      Pratical Algorithm Utilities 实用算法工具
  * @note       一些函数的实现，基于DSP库的arm_math.h库
  *             Implementation of some functions, based on DSP_arm_math.h library
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-3-2025      Light 卢俊烨     1. Not_done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */

#include "Alg_Utilities.hpp"
#include "arm_math.h"

/**
  * @brief          arcsin function
  * @param[in]      val
  * @retval         arc(val) uint:radian
  */
float Math::trig::arcsin(float val)
{

}

/**
  * @brief          arccos function
  * @param[in]      val
  * @retval         arccos(val) uint:radian
  */
float Math::trig::arccos(float val)
{

}

/**
  * @brief          arctan function
  * @param[in]      val
  * @retval         arctan(val) uint:radian
  */
float Math::trig::arctan(float val)
{

}


class Math::filter::LowPassFilter
{
public:
    /**
    * @brief          first-order low-pass filtering initialization
    * @param[in]      period    uint: seconds
    * @param[in]      _num_[1]  filtering parameter
    * @retval         none
    */
    void Lp_Init(float frame_period, float num[1])
    {
        /* clearing and initializing */
        this->frame_period = frame_period;
        this->num[0] = num[0];
        this->input = 0.0f;
        this->out = 0.0f;
    }

    /**
     * @brief          first-order low-pass filtering calculating
     * @param[in]      input
     * @retval         none
     */
    void Lp_Cal(float input)
    {
        this->input = input;
        this->out = this->num[0] / (this->num[0] + this->frame_period) * this->out + this->frame_period / (this->num[0] + this->frame_period) * this->input;
    }



private:
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
};



