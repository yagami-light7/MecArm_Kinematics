/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_PID.h
  * @brief      PID 中间件头文件
  * @note       Middleware Layer 中间件层
  *             1. 将旧框架 Alg_PID 迁移到 MWL 层
  *             2. 采用面向对象方式统一管理位置式/增量式 PID
  *             3. 提供普通误差计算与角度环误差计算两类接口
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-13-2026     Codex           1. create
  *
  @verbatim
  ==============================================================================
  * 本文件只负责 PID 算法本身，不关心具体使用它的是电机、云台还是机械臂。
  * 上层只需要初始化参数，然后周期调用 Calc 或 CalcRad 即可。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#pragma once

/**
 * @brief 头文件
 */
#include "main.h"

#include <stdint.h>

/**
 * @brief 宏定义
 */

/**
 * @brief 结构体
 */
typedef enum
{
    MWL_PID_POSITION = 0,
    MWL_PID_INCREMENTAL = 1
} MWL_PID_Mode_e;

typedef struct
{
    uint8_t mode;

    float kp;
    float ki;
    float kd;

    float max_out;
    float max_iout;

    float set;
    float fdb;

    float out;
    float pout;
    float iout;
    float dout;

    float dbuf[3];
    float error[3];
} MWL_PID_State_t;

/**
 * @brief 变量外部声明
 */

/**
 * @brief CPP部分
 */
#ifdef __cplusplus

class Class_PID
{
public:
    /**
     * @brief 构造函数
     */
    Class_PID();

    /**
     * @brief 初始化 PID 参数
     * @param[in] mode     PID 模式
     * @param[in] pid      三参数数组，依次为 kp/ki/kd
     * @param[in] max_out  输出限幅
     * @param[in] max_iout 积分限幅
     */
    void Init(uint8_t mode, const float pid[3], float max_out, float max_iout);

    /**
     * @brief 初始化 PID 参数
     * @param[in] mode     PID 模式
     * @param[in] kp       比例系数
     * @param[in] ki       积分系数
     * @param[in] kd       微分系数
     * @param[in] max_out  输出限幅
     * @param[in] max_iout 积分限幅
     */
    void Init(uint8_t mode, float kp, float ki, float kd, float max_out, float max_iout);

    /**
     * @brief 常规 PID 计算
     * @param[in] ref 当前反馈值
     * @param[in] set 目标设定值
     * @retval PID 输出
     */
    float Calc(float ref, float set);

    /**
     * @brief 角度环 PID 计算
     * @note  用于处理周期角误差，将误差格式化到 [-PI, PI]
     * @param[in] ref         当前反馈角度
     * @param[in] set         目标角度
     * @param[in] error_delta 角速度或误差变化率
     * @retval PID 输出
     */
    float CalcRad(float ref, float set, float error_delta);

    /**
     * @brief 清空 PID 内部状态
     */
    void Clear(void);

    /**
     * @brief 动态修改 PID 参数
     */
    void Change(float kp, float ki, float kd);

    /**
     * @brief 获取 PID 当前状态
     */
    const MWL_PID_State_t &GetState(void) const;

private:
    MWL_PID_State_t state_;
};

#endif

/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
