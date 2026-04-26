#ifndef USER_LIB_H
#define USER_LIB_H

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "arm_math.h"

typedef __packed struct
{
    float input;        //输入数据
    float out;          //输出数据
    float min_value;    //限幅最小值
    float max_value;    //限幅最大值
    float frame_period; //时间间隔
} ramp_function_source_t;

typedef struct
{
    float target;  // 目标值
    float current; // 当前值
    float frame_period; // 时间间隔
    float slope; // 变化斜率
    float out;
}ramp_fun_t;

/* 五次多项式插值 */
typedef struct
{
    /* 参数 */
    float quintic_coeffs[5]; // 定义五次多项式系数数组
    float frame_period;      // 间隔的时间 单位s

    /* 输出信息 */
    float pos; // 位置
    float vel; // 速度
    float acc; // 加速度

}Quintic_fun_t;

typedef __packed struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_order_filter_type_t;


#ifdef __cplusplus
extern "C"{
#endif


//快速开方
extern float invSqrt(float num);

//斜波函数初始化
void ramp_init(ramp_function_source_t *ramp_source_type, float frame_period, float max, float min);

//斜波函数计算
void ramp_calc(ramp_function_source_t *ramp_source_type, float input);

//一阶滤波初始化
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float frame_period, const float num[1]);

//一阶滤波计算
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float __packed input);

//绝对限制
extern void abs_limit(float *num, float Limit);

//判断符号位
extern float sign(float value);

//浮点死区
extern float float_deadline(float Value, float minValue, float maxValue);

//int26死区
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//限幅函数
extern float float_constrain(float Value, float minValue, float maxValue);

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
extern float float_remap(float input_value, float input_min, float input_max,
            float output_min, float output_max);

//限幅函数
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);

//循环限幅函数
extern float loop_float_constrain(float Input, float minValue, float maxValue);
//角度 °限幅 180 ~ -180
extern float theta_format(float Ang);

//将角度限制在0~360
extern float angle_format_360(float angle);

//弧度格式化为-PI~PI
#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

// 生成随机数
extern float Range_Number(float min, float max, const unsigned int *seed);


/**
  * @brief          斜坡函数初始化，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      斜坡函数结构体ramp
  * @param[in]      间隔时间,单位s
  * @retval         返回空
  */
extern void Ramp_Init(ramp_fun_t *init, float frame_period);


/**
  * @brief          斜坡函数计算，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      斜坡函数结构体ramp
  * @param[in]      目标值target
  * @param[in]      斜率slope
  * @retval         返回空
  */
extern void Ramp_Calc(ramp_fun_t *calc, float target, float slope);


/**
  * @brief          五次多项式插值初始化，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      五次多项式插值函数结构体quintic
  * @param[in]      间隔时间,单位s
  * @retval         返回空
  */
extern void Quintic_Init(Quintic_fun_t *quintic_init, float coeffs[6], float frame_period);


/**
  * @brief          五次多项式插值计算，用于机械臂角度平滑变化
  * @author         Light卢俊烨
  * @param[in]      五次多项式插值函数结构体quintic
  * @param[in]      间隔时间,单位s
  * @retval         返回空
  */
extern void Quintic_Calc(Quintic_fun_t *calc);


/**
  * @brief          线性插值 用于机械臂平滑运动
  * @author         Light卢俊烨
  * @param[in]      当前值     current
  * @param[in]      目标值     target
  * @param[in]      斜率       slope
  * @param[in]      间隔时间    frame_period(ms)
  * @retval         插值后的角度
  */
extern float Linear_Ramp(float current, float target, float slope, float frame_period);


#ifdef __cplusplus
}
#endif

#endif
