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
#include "App_Chassis_Behaviour.h"
#include "cmsis_os.h"
#include "App_Chassis_Task.h"
#include "arm_math.h"
#include "Alg_UserLib.h"
#include <stdio.h>
#include "App_Mechanical_Arm_Task.h"
//#include "gimbal_behaviour.h"

#include "AHRS.h"
#include "App_Detect_Task.h"
#include "App_Gimbal_Task.h"

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void CHASSIS_zero_speed_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, Chassis_Move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector);

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_infantry_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, Chassis_Move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_engineer_follow_chassis_yaw_control(float *vx_set, float *vy_set, float *angle_set, Chassis_Move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_engineer_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector);


/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void chassis_open_set_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector);


/**
  * @brief          小陀螺模式 以初始值方向为正轴，在此行为下，云台地盘分开控制 保持云台的正轴方向不变。
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      angle_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void  chassis_gyroscope(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector);

/**
  * @brief          小陀螺模式变速部分生成
  * @author         Light
  * @param[in]      random_seed   随机数种子
  * @param[in]      range_min     生成随机数的最小值
  * @param[in]      range_max     生成随机数的最小值
  * @param[in]      range_const   正弦函数的基础值
  * @param[in]      sin_frequency 正弦函数的频率
  * @param[in]      range_const   正弦函数的基础值
  * @note           变速部分的幅值大小由一个（常数+随机数）* sin函数得到
  * @retval         返回空
  */
static float get_changeable_gyrospeed(const unsigned int random_seed, float range_min, float range_max, float range_const, float sin_frequency);

Chassis_Mode_e Chassis_Behaviour_Mode = CHASSIS_ZERO_SPEED;

extern uint16_t last_v;

/**
  * @brief          通过逻辑判断，赋值"Chassis_Behaviour_Mode"成哪种模式
  * @param[in]      chassis_move_mode: 底盘数据
  * @retval         none
  */
void Chassis_Behaviour_Mode_Set(Chassis_Move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    static int8_t last_s = RC_SW_UP;

    /* 遥控器设置模式部分:针对右侧按键开关 */

    if(toe_is_error(DBUS_TOE) == 0)
    {
        // 若右侧开关为上档
        if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_RIGHT]))
        {
            if(gimbal_control.gimbal_motor_mode == GIMBAL_ZERO_POSITION_MODE)
            {
                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
            }
            else if(gimbal_control.gimbal_motor_mode == GIMBAL_CUSTOM_MODE || gimbal_control.gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE)
            {
                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
            }
            else
            {
                Chassis_Behaviour_Mode = CHASSIS_ZERO_SPEED;
            }
//            if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_LEFT]) && ! switch_is_down(chassis_move_mode->chassis_RC->rc.last_s[CHASSIS_MODE_CHANNEL_LEFT]))
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
//            }
//            else if(switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_LEFT]) && ! switch_is_mid(chassis_move_mode->chassis_RC->rc.last_s[CHASSIS_MODE_CHANNEL_LEFT]))
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
//            }
//            else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_LEFT]) && ! switch_is_up(chassis_move_mode->chassis_RC->rc.last_s[CHASSIS_MODE_CHANNEL_LEFT]))
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
//            }
//            else if(mec_arm_control.mec_arm_mode == MEC_ARM_ZERO_POSITION_MODE) // 防止边沿检测不灵敏导致模式相异
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
//            }
//            else if(mec_arm_control.mec_arm_mode != MEC_ARM_ZERO_POSITION_MODE) // 防止边沿检测不灵敏导致模式相异
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
//            }
        }
        // 若右侧开关为中档
        else if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_RIGHT]))
        {
            if(gimbal_control.gimbal_motor_mode == GIMBAL_ZERO_POSITION_MODE)
            {
                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
            }
            else if(gimbal_control.gimbal_motor_mode == GIMBAL_CUSTOM_MODE || gimbal_control.gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE)
            {
                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
            }
            else
            {
                Chassis_Behaviour_Mode = CHASSIS_ZERO_SPEED;
            }
//            if(switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_LEFT]) && ! switch_is_down(chassis_move_mode->chassis_RC->rc.last_s[CHASSIS_MODE_CHANNEL_LEFT]))
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
//            }
//            else if(switch_is_mid(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_LEFT]) && ! switch_is_mid(chassis_move_mode->chassis_RC->rc.last_s[CHASSIS_MODE_CHANNEL_LEFT]))
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
//            }
//            else if(switch_is_up(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_LEFT]) && ! switch_is_up(chassis_move_mode->chassis_RC->rc.last_s[CHASSIS_MODE_CHANNEL_LEFT]))
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
//            }
//            else if(mec_arm_control.mec_arm_mode == MEC_ARM_ZERO_POSITION_MODE) // 防止边沿检测不灵敏导致模式相异
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
//            }
//            else if(mec_arm_control.mec_arm_mode != MEC_ARM_ZERO_POSITION_MODE) // 防止边沿检测不灵敏导致模式相异
//            {
//                Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
//            }
        }
        // 若开关为下档:车车直接Die掉
        else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_RIGHT]))
        {
            Chassis_Behaviour_Mode = CHASSIS_ZERO_SPEED; // 底盘不动
        }

        //添加自己的逻辑判断进入新模式

        last_s = chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_CHANNEL_RIGHT];
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        if(gimbal_control.gimbal_motor_mode == GIMBAL_ZERO_POSITION_MODE)
        {
            Chassis_Behaviour_Mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW;
        }
        else if(gimbal_control.gimbal_motor_mode == GIMBAL_CUSTOM_MODE || gimbal_control.gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE)
        {
            Chassis_Behaviour_Mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW;
        }
        else
        {
            Chassis_Behaviour_Mode = CHASSIS_ZERO_SPEED;
        }
    }

    /* 根据行为模式选择一个底盘控制模式 */

    if (Chassis_Behaviour_Mode == CHASSIS_ZERO_SPEED)
    {
        chassis_move_mode->chassis_mode = CHASSIS_ZERO_SPEED; // 开环输出（一般直接给电流0）
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_NO_MOVE)
    {
        chassis_move_mode->chassis_mode = CHASSIS_NO_MOVE; // 底盘有旋转速度控制（一般直接给0速度）
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW; // 底盘有底盘角度控制闭环
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_ENGINEER_NO_FOLLOW_YAW)
    {
        chassis_move_mode->chassis_mode = CHASSIS_ENGINEER_NO_FOLLOW_YAW; // 底盘有平移速度控制
    }
}

/**
  * @brief          设置控制量.根据不同底盘控制模式，三个参数会控制不同运动.在这个函数里面，会调用不同的控制函数.
  * @param[out]     vx_set, 通常控制纵向移动.
  * @param[out]     vy_set, 通常控制横向移动.
  * @param[out]     wz_set, 通常控制旋转运动.
  * @param[in]      chassis_move_rc_to_vector,  包括底盘所有信息.
  * @retval         none
  */

void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    if (Chassis_Behaviour_Mode == CHASSIS_ZERO_SPEED)
    {
        CHASSIS_zero_speed_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (Chassis_Behaviour_Mode == CHASSIS_ENGINEER_NO_FOLLOW_YAW)
    {
        chassis_engineer_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void CHASSIS_zero_speed_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}

/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘与云台控制到的相对角度
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_infantry_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    //遥控器的通道值以及键盘按键 得出 一般情况下的速度设定值
    Chassis_RC_To_Control_Vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //摇摆角度是利用sin函数生成，swing_time是sin函数的输入值
    static float swing_time = 0.0f;

    static float swing_angle = 0.0f;

    //max_angle 是sin函数的幅值
    static float max_angle = SWING_NO_MOVE_ANGLE;

    //swing_time 在一个控制周期内，加上 add_time
    static float const add_time = PI * 0.5f * configTICK_RATE_HZ / CHASSIS_CONTROL_TIME_MS;

    static uint8_t swing_flag = 0;
}

/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void chassis_engineer_follow_chassis_yaw_control(float *vx_set, float *vy_set, float *angle_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    static float add_delta_angle = 0.0f;

    Chassis_RC_To_Control_Vector(vx_set, vy_set, chassis_move_rc_to_vector);

    /* 摇杆的角度差由摇杆和鼠标移动叠加得到 */
    if(toe_is_error(DBUS_TOE) == 0)
    {
        add_delta_angle = CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]+ chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out * YAW_MOUSE_SEN;
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        add_delta_angle = CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->vt_rc_control->ch_3+ chassis_move_rc_to_vector->chassis_cmd_slow_set_wz.out * YAW_MOUSE_SEN;
    }

    if(toe_is_error(DBUS_TOE) == 0)
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, chassis_move_rc_to_vector->chassis_RC->mouse.x); // 先进行低通滤波
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_wz, chassis_move_rc_to_vector->vt_rc_control->mouse_x); // 先进行低通滤波
    }

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - add_delta_angle);

    /* 停止信号，不需要缓慢停止，直接减速到零 */
    if (add_delta_angle < CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_Z_RC_SEN && add_delta_angle > -CHASSIS_RC_DEADLINE * CHASSIS_ANGLE_Z_RC_SEN)// 进入死区
    {
        *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set);
    }

}

/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  * @author         RM
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */
static void chassis_engineer_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    Chassis_RC_To_Control_Vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = 0;
}

/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度，正值 左移速度， 负值 右移速度
  * @param[in]      wz_set 旋转速度， 正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         none
  */

static void chassis_open_set_control(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}

/**
  * @brief          小陀螺模式
  * @author         LWH
  * @param[in]      vx_set前进的速度,正值 前进速度， 负值 后退速度
  * @param[in]      vy_set左右的速度,正值 左移速度， 负值 右移速度
  * @param[in]      wz_set底盘设置的旋转速度,正值 逆时针旋转，负值 顺时针旋转
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
  */

static void  chassis_gyroscope(float *vx_set, float *vy_set, float *wz_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    Chassis_RC_To_Control_Vector(vx_set, vy_set, chassis_move_rc_to_vector);

    /* 变速小陀螺由以下几部分角速度叠加而成
     * 1.fixed_gyrospeed：固定角速度
     * 2.changeable_gyrospeed：变化的角速度
     * 3.xy_deleted_gyrospeed：平动削减分量
     * */

    const float fixed_gyrospeed = CHASSIS_SPIN_MAIN_SPEED;
    float changeable_gyrospeed = get_changeable_gyrospeed(1234, 0.31, 0.58, 1.25, 0.02);
    float xy_deleted_gyrospeed = - fabs(chassis_move_rc_to_vector->vx_set) * CHASSIS_SPIN_LOW_SEN - fabs(chassis_move_rc_to_vector->vy_set) * CHASSIS_SPIN_LOW_SEN;

    *wz_set = fixed_gyrospeed + changeable_gyrospeed + xy_deleted_gyrospeed;
}

/**
  * @brief          小陀螺模式变速部分生成
  * @author         Light
  * @param[in]      random_seed   随机数种子
  * @param[in]      range_min     生成随机数的最小值
  * @param[in]      range_max     生成随机数的最小值
  * @param[in]      range_const   正弦函数的基础值
  * @param[in]      sin_frequency 正弦函数的频率
  * @param[in]      range_const   正弦函数的基础值
  * @note           变速部分的幅值大小由一个（常数+随机数）* sin函数得到
  * @retval         返回空
  */
static float get_changeable_gyrospeed(const unsigned int random_seed, float range_min, float range_max, float range_const, float sin_frequency)
{
    float gyroscope_random = Range_Number(range_min, range_max, &random_seed); // 幅值随机数
    static float gyroscope_frequency = 0.0f; // 正弦频率
    float Speed_Changeable = (range_const + gyroscope_random) * arm_sin_f32(gyroscope_frequency);

    gyroscope_frequency += sin_frequency; // 相位自增

    return Speed_Changeable;
}