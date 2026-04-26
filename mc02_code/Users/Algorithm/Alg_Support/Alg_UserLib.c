/* Includes ------------------------------------------------------------------*/
#include "Alg_UserLib.h"

//快速开方
float invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          斜波函数初始化
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      最大值
  * @param[in]      最小值
  * @retval         返回空
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          斜波函数计算，根据输入的值进行叠加， 输入单位为 /s 即一秒后增加输入的值
  * @author         RM
  * @param[in]      斜波函数结构体
  * @param[in]      输入值
  * @param[in]      滤波参数
  * @retval         返回空
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}

/**
  * @brief          斜坡函数初始化，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      斜坡函数结构体ramp
  * @param[in]      间隔时间,单位s
  * @retval         返回空
  */
void Ramp_Init(ramp_fun_t *init, float frame_period)
{
    /* 清零 */
    init->current = 0.0f;
    init->target  = 0.0f;
    init->slope   = 0.0f;

    /* 拷贝时间间隔值 */
    init->frame_period = frame_period;
}

/**
  * @brief          斜坡函数计算，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      斜坡函数结构体ramp
  * @param[in]      目标值target
  * @param[in]      斜率slope
  * @retval         返回空
  */
void Ramp_Calc(ramp_fun_t *calc, float target, float slope)
{
    float delta = target - calc->current; // 计算差值
    float step = fabsf(slope); // 取斜率的绝对值作为步长

    if (delta > 0)
    {
        // 正向逼近目标值
        calc->current += fminf(delta, step);
    }
    else if (delta < 0)
    {
        // 负向逼近目标值
        calc->current -= fminf(-delta, step);
    }

    // delta=0时无需调整
}

/**
  * @brief          五次多项式插值初始化，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      五次多项式插值函数结构体quintic
  * @param[in]      间隔时间,单位s
  * @retval         返回空
  */
void Quintic_Init(Quintic_fun_t *quintic_init, float coeffs[6], float frame_period)
{
    quintic_init->frame_period = frame_period;
    quintic_init->quintic_coeffs[0] = coeffs[0];
    quintic_init->quintic_coeffs[1] = coeffs[1];
    quintic_init->quintic_coeffs[2] = coeffs[2];
    quintic_init->quintic_coeffs[3] = coeffs[3];
    quintic_init->quintic_coeffs[4] = coeffs[4];
    quintic_init->quintic_coeffs[5] = coeffs[5];
}

/**
  * @brief          五次多项式插值计算，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      五次多项式插值函数结构体quintic
  * @param[in]      间隔时间,单位s
  * @retval         返回空
  */
void Quintic_Calc(Quintic_fun_t *calc)
{
    calc->pos = calc->quintic_coeffs[5] * powf(calc->frame_period, 5) + calc->quintic_coeffs[4] * powf(calc->frame_period,4)
              + calc->quintic_coeffs[3] * powf(calc->frame_period,3) + calc->quintic_coeffs[2] * powf(calc->frame_period,2)
              + calc->quintic_coeffs[1] * calc->frame_period + calc->quintic_coeffs[0];
    calc->vel = 5 * calc->quintic_coeffs[5] * powf(calc->frame_period,4) + 4 * calc->quintic_coeffs[4] * powf(calc->frame_period,3)
              + 3 * calc->quintic_coeffs[3] * powf(calc->frame_period,2) + 2 * calc->quintic_coeffs[2] * calc->frame_period
              + calc->quintic_coeffs[1];
    calc->acc = 20 * calc->quintic_coeffs[5] * powf(calc->frame_period,3) + 12 * calc->quintic_coeffs[4] * powf(calc->frame_period,2)
               + 6 * calc->quintic_coeffs[3] * calc->frame_period + 2 * calc->quintic_coeffs[2];
}

/**
  * @brief          一阶低通滤波初始化
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波参数
  * @retval         返回空
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @author         RM
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @retval         返回空
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
            first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//绝对限制
void abs_limit(float *num, float Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//判断符号位
float sign(float value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//浮点死区
float float_deadline(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//int26死区
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

/**
 * @brief 将输入值从原始区间线性映射到目标区间
 * @param input_value 原始输入值
 * @param input_min 原始区间最小值
 * @param input_max 原始区间最大值
 * @param output_min 目标区间最小值
 * @param output_max 目标区间最大值
 * @return 映射后的值（浮点数）
 *
 * @note 当输入区间无效（input_max == input_min）时，返回output_min
 */
float float_remap(float input_value, float input_min, float input_max,
            float output_min, float output_max)
{
    float scale = (output_max - output_min) / (input_max - input_min);
    float result = output_min + (input_value - input_min) * scale;

    if(result > output_max)
    {
        return output_max;
    }
    else if(result < output_min)
    {
        return output_min;
    }
    else
    {
        return result;
    }

}

//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//弧度格式化为-PI~PI

//角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

float angle_format_360(float angle)
{
    angle = fmod(angle, 360.0f);
    if (angle < 0)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
  * @brief          生成随机数
  * @author         罗威昊、Light
  * @param[in]      随机数种子
  * @param[in]      生成随机数的最小值
  * @param[in]      生成随机数的最大值
  * @retval         返回生成的随机数
  */
uint16_t customRand(unsigned int *seed)
{
    *seed = (*seed * 1103515245 + 12345) & 0x7fffffff;
    return *seed;
}

float Range_Number(float min, float max, const unsigned int *seed)
{
    float r = (float)customRand(seed) / 0x7fffffff; // 生成0到1之间的随机数
    return min + r * (max - min);
}

/**
  * @brief          线性插值 用于机械臂平滑运动
  * @author         Light卢俊烨
  * @param[in]      当前值     current
  * @param[in]      目标值     target
  * @param[in]      斜率       slope
  * @param[in]      间隔时间    frame_period(ms)
  * @retval         插值后的角度
  */
float Linear_Ramp(float current, float target, float slope, float frame_period)
{
    float delta = target - current; // 计算差值
    float step = fabsf(slope * frame_period); // 计算步长

    if (fabsf(delta) < step)
    {
        // 如果差值小于最大步进，直接到达目标
        return target;
    }
    else
    {
        // 逐渐逼近目标值
        return current + (delta > 0 ? step : -step);
    }

}