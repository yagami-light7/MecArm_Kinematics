/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      according to remote control, change the chassis behaviour.
  *             根据遥控器的值，决定底盘行为。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================
    如果要添加一个新的行为模式
    1.首先，在chassis_behaviour.h文件中， 添加一个新行为名字在 chassis_behaviour_e
    erum
    {
        ...
        ...
        CHASSIS_XXX_XXX, // 新添加的
    }chassis_behaviour_e,

    2. 实现一个新的函数 chassis_xxx_xxx_control(float *vx, float *vy, float *wz, Chassis_Move_t * chassis )
        "vx,vy,wz" 参数是底盘运动控制输入量
        第一个参数: 'vx' 通常控制纵向移动,正值 前进， 负值 后退
        第二个参数: 'vy' 通常控制横向移动,正值 左移, 负值 右移
        第三个参数: 'wz' 可能是角度控制或者旋转速度控制
        在这个新的函数, 你能给 "vx","vy",and "wz" 赋值想要的速度参数
    3.  在"Chassis_Behaviour_Mode_set"这个函数中，添加新的逻辑判断，给Chassis_Behaviour_Mode赋值成CHASSIS_XXX_XXX
        在函数最后，添加"else if(Chassis_Behaviour_Mode == CHASSIS_XXX_XXX)" ,然后选择一种底盘控制模式
        4种:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 云台和底盘的相对角度
        你可以命名成"xxx_angle_set"而不是'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'是速度控制， 'wz'是角度控制 底盘的陀螺仪计算出的绝对角度
        你可以命名成"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'是速度控制， 'wz'是旋转速度控制
        CHASSIS_VECTOR_RAW : 使用'vx' 'vy' and 'wz'直接线性计算出车轮的电流值，电流值将直接发送到can 总线上
    4.  在"chassis_behaviour_control_set" 函数的最后，添加
        else if(Chassis_Behaviour_Mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef INFANTRY_ROBOT_APP_CHASSIS_BEHAVIOUR_H
#define INFANTRY_ROBOT_APP_CHASSIS_BEHAVIOUR_H

#include "App_Chassis_Task.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"

#define CHASSIS_OPEN_RC_SCALE 10 				//在chassis_open 模型下，遥控器乘以该比例发送到can上
#define CHASSIS_SPIN_KEYBOARD   KEY_PRESSED_OFFSET_CTRL

//typedef enum
//{
////    CHASSIS_ZERO_SPEED,                   //底盘无力, 跟没上电那样
////    CHASSIS_NO_MOVE,                      //底盘保持不动
////    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //工程底盘角度控制底盘
////    CHASSIS_SPIN,   										  //小陀螺模式
//
//    CHASSIS_SENTINEL,                     //哨兵模式
//} Chassis_Behaviour_e;

#ifdef __cplusplus
extern "C"{
#endif

/**
  * @brief          通过逻辑判断，赋值"Chassis_Behaviour_Mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
extern void Chassis_Behaviour_Mode_Set(Chassis_Move_t *chassis_move_mode);

/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */
extern void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, Chassis_Move_t *chassis_move_rc_to_vector);

extern Chassis_Mode_e Chassis_Behaviour_Mode; // 声明底盘行为模式的全局变量

#ifdef __cplusplus
}
#endif


#endif //INFANTRY_ROBOT_APP_CHASSIS_BEHAVIOUR_H