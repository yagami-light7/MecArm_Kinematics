/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    如果要添加一个新的行为模式
    1.首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
    erum
    {
        ...
        ...
        GIMBAL_XXX_XXX, // 新添加的
    }gimbal_behaviour_e,

    2. 实现一个新的函数 gimbal_xxx_xxx_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 参数是云台运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    3.  在"gimbal_behavour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
        在gimbal_behaviour_mode_set函数最后，添加"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然后选择一种云台控制模式
        3种:
        GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"gimbal_behaviour_control_set" 函数的最后，添加
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef INFANTRY_ROBOT_APP_GIMBAL_BEHAVIOUR_H
#define INFANTRY_ROBOT_APP_GIMBAL_BEHAVIOUR_H

#include "main.h"
#include "App_Gimbal_Task.h"

typedef enum
{
    GIMBAL_ZERO_FORCE = 0,
    GIMBAL_INIT,
    GIMBAL_CALI,
    GIMBAL_ABSOLUTE_ANGLE,
    GIMBAL_RELATIVE_ANGLE,

    GIMBAL_ERROR,
    GIMBAL_CV_AUTO,  //辅助瞄准模式

    GIMBAL_ZERO_POSITION, // 大关节归零位
    GIMBAL_ANGLE_CONTROL, // 大关节角度控制
    GIMBAL_MOTIONLESS,    // 大关节无运动（位置锁住）
    GIMBAL_CUSTOM,        // 使用自定义控制器，大臂跟随小臂

    /* 自动模式 */
    GIMBAL_AUTO_GET_GND_MINE,  // 一键取小资源岛地矿
    GIMBAL_AUTO_GET_L_MINE,    // 一键取大资源岛矿石
} gimbal_behaviour_e;


#ifdef __cplusplus
extern "C"{
#endif

/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_pitch:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
extern void
gimbal_behaviour_control_set(float *add_yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

extern bool gimbal_cmd_to_chassis_stop(void);

/**
  * @brief          云台在某些行为下，需要射击停止
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
extern bool gimbal_cmd_to_shoot_stop(void);

#ifdef __cplusplus
}
#endif

#endif
