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

#include "App_Gimbal_Behaviour.h"
#include "arm_math.h"
#include "Board_Buzzer.h"
#include "App_Detect_Task.h"
#include "Alg_UserLib.h"
#include "App_Chassis_Task.h"
#include "math.h"
#include "stdio.h"
#include "App_Calibrate_Task.h"
#include "cmsis_os.h"

// 当云台在校准, 设置蜂鸣器频率和强度
#define gimbal_warn_buzzer_on() Buzzer_On(31, 20000)
#define gimbal_warn_buzzer_off() Buzzer_OFF()
#define int_abs(x) ((x) > 0 ? (x) : (-x))
#define PI 3.1415926535f
#define GRAVITY 9.78



/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  * @param          输入的遥控器值
  * @param          输出的死区处理后遥控器值
  * @param          死区值
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  * @param          对应轴的角速度，单位rad/s
  * @param          计时时间，到达GIMBAL_CALI_STEP_TIME的时间后归零
  * @param          记录的角度 rad
  * @param          反馈的角度 rad
  * @param          记录的编码值 raw
  * @param          反馈的编码值 raw
  * @param          校准的步骤 完成一次 加一
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set);
static void gimbal_zero_force_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set);
static void gimbal_zero_position_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set);
static void gimbal_angle_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set);
static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set);
static void gimbal_motionless_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set);
static void gimbal_custom_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set);

//云台行为状态机
gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          被gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    // 云台行为状态机设置
    gimbal_behaviour_set(gimbal_mode_set);

    // 根据云台行为状态机设置电机状态机 拷贝行为状态
    if(gimbal_behaviour == GIMBAL_ZERO_FORCE  && joint_cali.calibrate_finished_flag == CALIBRATE_FINISHED)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_ZERO_FORCE_MODE;
    }
    else if(gimbal_behaviour == GIMBAL_ZERO_POSITION)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_ZERO_POSITION_MODE;
    }
    else if(gimbal_behaviour == GIMBAL_ANGLE_CONTROL)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_ANGLE_CONTROL_MODE;
    }
    else if(gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_MOTIONLESS_MODE;
    }
    else if(gimbal_behaviour == GIMBAL_CUSTOM)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_CUSTOM_MODE;
    }
    else if(gimbal_behaviour == GIMBAL_AUTO_GET_GND_MINE)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_AUTO_GET_GND_MINE_MODE;
    }
    else if(gimbal_behaviour == GIMBAL_AUTO_GET_L_MINE)
    {
        gimbal_mode_set->gimbal_motor_mode = GIMBAL_AUTO_GET_L_MINE_MODE;
    }

//    if (gimbal_behaviour == GIMBAL_ZERO_FORCE) //无力状态
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_ZERO_FORCE;
//    }
//    else if (gimbal_behaviour == GIMBAL_INIT) //云台初始化状态
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_INIT;
//    }
//    else if (gimbal_behaviour == GIMBAL_CALI)//云台校准状态
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_CALI;
//    }
//    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE) //陀螺仪控制状态
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_ABSOLUTE_ANGLE;
//    }
//    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTIONLESS;
//    }
//    else if (gimbal_behaviour == GIMBAL_ERROR)
//    {
//        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_ERROR;
//    }
}

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @param[out]     add_yaw:设置的yaw角度增加值，单位 rad
  * @param[out]     add_j1:设置的pitch角度增加值，单位 rad
  * @param[out]     add_j2:设置的pitch角度增加值，单位 rad
  * @param[in]      gimbal_mode_set:云台数据指针
  * @retval         none
  */
void gimbal_behaviour_control_set(float *add_yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_j1 == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    // 选择行为模式
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }
    else if(gimbal_behaviour == GIMBAL_ZERO_POSITION)
    {
        gimbal_zero_position_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }
    else if(gimbal_behaviour == GIMBAL_ANGLE_CONTROL)
    {
        gimbal_angle_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }
    else if(gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }
    else if(gimbal_behaviour == GIMBAL_CUSTOM)
    {
        gimbal_custom_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }
    else if(gimbal_behaviour == GIMBAL_AUTO_GET_GND_MINE)
    {
        gimbal_angle_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }
    else if(gimbal_behaviour == GIMBAL_AUTO_GET_L_MINE)
    {
        gimbal_angle_control(add_yaw, add_j1, add_j2, gimbal_control_set);
    }



//    else if (gimbal_behaviour == GIMBAL_INIT)
//    {
//        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
//    }
//    else if (gimbal_behaviour == GIMBAL_CALI)
//    {
//        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
//    }
//    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
//    {
//        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
//    }
//    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
//    {
//        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
//    }
//    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
//    {
//        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
//    }
//    else if(gimbal_behaviour == GIMBAL_ERROR)
//    {
//
//    }
    //请在这里添加其他行为
    // if(gimbal_behaviour == 行为 )
    // {

    // }

}

/**
  * @brief          云台在某些行为下，需要底盘不动
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int8_t last_left_s = RC_SW_UP;
int8_t last_right_s = RC_SW_MID;
uint16_t last_v = 0;
uint16_t last_keyboard_0x304 = 0;

/* 一键地矿 */
extern float _gnd_mine_emulation_;     // 俯仰滚轮累计值，用于控制一键地矿
extern float _store_emulation_;        // 累计到达一定值，表明存在左侧/右侧吸盘
extern float _take_emulation_;         // 累计到达一定值，表明进行放矿/拿矿
extern int16_t _gnd_mine_flag;
extern int16_t _store_flag;
extern int16_t _take_flag;

/* 一键大资源岛矿 */
extern float _gimbal_l_mine_emulation_;       // 累计到达一定值，表明选择资源岛的三个通道

/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behaviour_set(gimbal_control_t *gimbal_mode_set)
{
    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;

    if (gimbal_mode_set == NULL)
    {
        return;
    }

    if(toe_is_error(DBUS_TOE) == 0)
    {
        /* 开关控制云台模式 */
        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT]))// 右下
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE; // 无力状态

            /* 左下 可以重新进行J1\J2\末端的校准 */
            if(switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT])) // 左下
            {
                static TickType_t time_begin = 0;

                if( int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[0]) == 660 &&
                    int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[1]) == 660 &&
                    int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[2]) == 660 &&
                    int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[3]) == 660)
                {
                    if(time_begin == 0) // 记录起始时间戳
                    {
                        time_begin = xTaskGetTickCount();
                    }


                    if ((xTaskGetTickCount() - time_begin) > 1000 ) // 持续时间超过一秒
                    {
                        joint_cali.calibrate_finished_flag = 0; // 归零校验标志位，重新进行校准
                    }

                }
                else // 中途退出校准
                {
                    time_begin = 0;
                }
            }

        }
        else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT]))// 右中
        {
            if ( switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT]) && ! switch_is_down(last_left_s) ) // 左下
            {
                gimbal_behaviour = GIMBAL_ZERO_POSITION; // 归零位
                //同时自定义控制器也会归零位
            }
            else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT]) && ! switch_is_mid(last_left_s) )// 左中
            {
                #if _CUSTOM_USED_
                gimbal_behaviour = GIMBAL_CUSTOM;
                #else
                gimbal_behaviour = GIMBAL_ANGLE_CONTROL; // 开始控制大关节角度
                #endif

            }
            else if( switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT]) && ! switch_is_up(last_left_s) )// 左上
            {
                #if _CUSTOM_USED_
                gimbal_behaviour = GIMBAL_AUTO_GET_GND_MINE;
                #else
                gimbal_behaviour = GIMBAL_MOTIONLESS;
                #endif
            }

            /* 关闭吸盘 */
            if(switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT]) && !switch_is_mid(last_right_s))
            {
                pump_master_off;
            }

        }
        else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT])) //右上
        {
            if ( switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT]) && ! switch_is_down(last_left_s) ) // 左下
            {
                gimbal_behaviour = GIMBAL_ZERO_POSITION; // 归零位
                //同时自定义控制器也会归零位
            }
            else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT]) && ! switch_is_mid(last_left_s) )// 左中
            {
                #if _CUSTOM_USED_
                gimbal_behaviour = GIMBAL_CUSTOM;
                #else
                gimbal_behaviour = GIMBAL_ANGLE_CONTROL; // 开始控制大关节角度
                #endif

            }
            else if( switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT]) && ! switch_is_up(last_left_s) )// 左上
            {
                #if _CUSTOM_USED_
                gimbal_behaviour = GIMBAL_AUTO_GET_GND_MINE;
                #else
                gimbal_behaviour = GIMBAL_MOTIONLESS;
                #endif

            }

            /* 开启吸盘 */
            if(switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT]) && !switch_is_up(last_right_s))
            {
                pump_master_on;
            }

        }
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        // N档 中
        if(gimbal_mode_set->vt_rc_control->mode_sw == 1)
        {
            // 暂停按键进入归零位模式
            if(gimbal_mode_set->vt_rc_control->pause == 1)
            {
                if (gimbal_mode_set->vt_rc_control->last_pause == 0)
                {
                    gimbal_behaviour = GIMBAL_ZERO_POSITION;
                }
            }

            // 左侧自定义按键进入自定义控制器模式
            if(gimbal_mode_set->vt_rc_control->fn_1 == 1)
            {
                if (gimbal_mode_set->vt_rc_control->last_fn_1 == 0)
                {
                    gimbal_behaviour = GIMBAL_CUSTOM;
                }
            }

            // 右侧自定义按键进入一键地矿模式
            if(gimbal_mode_set->vt_rc_control->fn_2 == 1)
            {
                if (gimbal_mode_set->vt_rc_control->last_fn_2 == 0)
                {
                    gimbal_behaviour = GIMBAL_AUTO_GET_GND_MINE;
                }
            }

        }
        // C档 左
        else if(gimbal_mode_set->vt_rc_control->mode_sw == 0)
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;

            static TickType_t tick_start = 0;

            if( int_abs(gimbal_mode_set->vt_rc_control->ch_0) == 660 &&
                int_abs(gimbal_mode_set->vt_rc_control->ch_1) == 660 &&
                int_abs(gimbal_mode_set->vt_rc_control->ch_2) == 660 &&
                int_abs(gimbal_mode_set->vt_rc_control->ch_3) == 660)
            {
                if(tick_start == 0) // 记录起始时间戳
                {
                    tick_start = xTaskGetTickCount();
                }


                if ((xTaskGetTickCount() - tick_start) > 1000 ) // 持续时间超过一秒
                {
                    joint_cali.calibrate_finished_flag = 0; // 归零校验标志位，重新进行校准
                }

            }
            else // 中途退出校准
            {
                tick_start = 0;
            }
        }
        // S档 右
        else if(gimbal_mode_set->vt_rc_control->mode_sw == 2)
        {
            // 左侧自定义按键控制左侧气泵
            if(gimbal_mode_set->vt_rc_control->fn_1 == 1)
            {
                if (gimbal_mode_set->vt_rc_control->last_fn_1 == 0)
                {
                    sucker_left_toggle
                }
            }

            // 右侧自定义按键控制右侧气泵
            if(gimbal_mode_set->vt_rc_control->fn_2 == 1)
            {
                if (gimbal_mode_set->vt_rc_control->last_fn_2 == 0)
                {
                   sucker_right_toggle
                }
            }
        }

        // 扳机控制主吸盘
        if(gimbal_mode_set->vt_rc_control->trigger == 1)
        {
            if(gimbal_mode_set->vt_rc_control->last_trigger == 0)
            {
                pump_master_toggle
            }
        }

        RC_Hub.vt_rc_control.last_fn_1      = RC_Hub.vt_rc_control.fn_1;
        RC_Hub.vt_rc_control.last_fn_2      = RC_Hub.vt_rc_control.fn_2;
        RC_Hub.vt_rc_control.last_pause     = RC_Hub.vt_rc_control.pause;
        RC_Hub.vt_rc_control.last_trigger   = RC_Hub.vt_rc_control.trigger;
    }

    /* 键鼠操作行为模式，这里的操作会覆盖之前的摇杆操作 */

    // PART0 DT7
    if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_B) )
    {
        if((last_v & KEY_PRESSED_OFFSET_B) == 0)
        {
//            pump_master_toggle;  交由图传链路控制
        }
    }

    else if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)) // ctrl + 对应按键 防止误触
    {
        // 控制副气泵 一般都是打开的
        if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_V) )
        {
            if((last_v & KEY_PRESSED_OFFSET_V) == 0)
            {
//                pump_slave_toggle;   交由图传链路控制
            }
        }
        //切换模式
        else if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) )
        {
//            if((last_v & KEY_PRESSED_OFFSET_SHIFT) == 0)
//            {
//                static uint16_t behaviour_turn = 0;
//
//                if( behaviour_turn %2 == 0)
//                {
//                    gimbal_behaviour = GIMBAL_AUTO_GET_GND_MINE;
//                }
//                else
//                {
//                    gimbal_behaviour = GIMBAL_CUSTOM;
//                }
//
//                behaviour_turn++;
//            }
        }
        else if ( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F))
        {
            gimbal_behaviour = GIMBAL_ZERO_POSITION;
        }
        /* 自动取/存矿键位操作 */
        if(gimbal_behaviour == GIMBAL_AUTO_GET_GND_MINE)
        {
            // ctrl + W 中存
            if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W) )
            {
                if((last_v & KEY_PRESSED_OFFSET_W) == 0)
                {
                    _store_emulation_ = 0;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + Q 左存/左取地矿
            else if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q) )
            {
                if((last_v & KEY_PRESSED_OFFSET_Q) == 0)
                {
                    _store_emulation_ = GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + S 下取回地矿
            else if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S) )
            {
                if((last_v & KEY_PRESSED_OFFSET_S) == 0)
                {
                    _take_emulation_ = GIMBAL_GND_TAKE_LIMITATION;
                }
            }
                // ctrl + E 右存/右取地矿
            else if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) )
            {
                if((last_v & KEY_PRESSED_OFFSET_E) == 0)
                {
                    _store_emulation_ = -GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + G 控制取地矿
            else if( (gimbal_mode_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) )
            {
                if((last_v & KEY_PRESSED_OFFSET_G) == 0)
                {
                    static int16_t _gnd_mine_turn_ = 0;

                    if(_gnd_mine_turn_ %2 ==0)
                    {
                        _gnd_mine_emulation_ = GIMBAL_GND_MINE_LIMITATION;
                    }
                    else
                    {
                        _gnd_mine_emulation_ = 0;
                    }

                    _gnd_mine_turn_++;

                }
            }
        }

    }

    // PART1 图传链路
    if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_B) )
    {
        if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_B) == 0)
        {
            pump_master_toggle;
        }
    }

    else if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_CTRL)) // ctrl + 对应按键 防止误触
    {
        // 控制副气泵 一般都是打开的
        if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_V) )
        {
            if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_V) == 0)
            {
                pump_slave_toggle;
            }
        }
        //切换模式
        else if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_SHIFT) )
        {
            if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_SHIFT) == 0)
            {
                static uint16_t behaviour_turn = 0;

                if( behaviour_turn %2 == 0)
                {
                    gimbal_behaviour = GIMBAL_AUTO_GET_GND_MINE;
                }
                else
                {
                    gimbal_behaviour = GIMBAL_CUSTOM;
                }

                behaviour_turn++;
            }
        }
        else if ( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_F))
        {
            gimbal_behaviour = GIMBAL_ZERO_POSITION;
        }
        /* 自动取/存矿键位操作 */
        if(gimbal_behaviour == GIMBAL_AUTO_GET_GND_MINE)
        {
            // ctrl + W 中存
            if( (gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_W) )
            {
                if((last_v & KEY_PRESSED_OFFSET_W) == 0)
                {
                    _store_emulation_ = 0;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + Q 左存/左取地矿
            else if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_Q) )
            {
                if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_Q) == 0)
                {
                    _store_emulation_ = GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + S 下取回地矿
            else if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_S) )
            {
                if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_S) == 0)
                {
                    _take_emulation_ = GIMBAL_GND_TAKE_LIMITATION;
                }
            }
                // ctrl + E 右存/右取地矿
            else if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_E) )
            {
                if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_E) == 0)
                {
                    _store_emulation_ = -GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + G 控制取地矿
            else if( (gimbal_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_G) )
            {
                if((gimbal_mode_set->vt_rc_control->gimbal_key_last & KEY_PRESSED_OFFSET_G) == 0)
                {
                    static int16_t _gnd_mine_turn_ = 0;

                    if(_gnd_mine_turn_ %2 ==0)
                    {
                        _gnd_mine_emulation_ = GIMBAL_GND_MINE_LIMITATION;
                    }
                    else
                    {
                        _gnd_mine_emulation_ = 0;
                    }

                    _gnd_mine_turn_++;

                }
            }
        }

    }



//    if( toe_is_error(DBUS_TOE) && toe_is_error(VT_TOE))
//    {
//        gimbal_behaviour = GIMBAL_ZERO_FORCE;
//    }

    last_gimbal_behaviour = gimbal_behaviour;

    last_left_s  = gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT];
    last_right_s = gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT];

    /* 记录上次键盘 */
    last_v = gimbal_mode_set->gimbal_rc_ctrl->key.v;
    RC_Hub.vt_rc_control.gimbal_key_last = RC_Hub.vt_rc_control.key;

}

/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是当云台行为模式是GIMBAL_ZERO_FORCE_MODE
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      add_j1:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_force_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || add_j1 == NULL || add_j2 == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    gimbal_control_set->gimbal_yaw_motor.mit_on_off = MIT_ON;
    *yaw = 0.0f;
    *add_j1 = 0.0f;
    *add_j2 = 0.0f;
}
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_POSITION, 这个函数会被调用,云台控制模式是当云台行为模式是GIMBAL_ZERO_POSITION_MODE
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_zero_position_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || add_j1 == NULL || add_j2 == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    gimbal_control_set->gimbal_yaw_motor.mit_on_off = MIT_ON;

    /* 使用摇杆或者鼠标控制 */
    *yaw    = 0.0f;
    if (toe_is_error(DBUS_TOE) == 0)
    {
        *add_j1 = gimbal_control_set->gimbal_rc_ctrl->mouse.y * L_JOINT1_MOUSE_SEN; // 在使用图传时，微调角度取得更好的视野
        *add_j2 = gimbal_control_set->gimbal_rc_ctrl->mouse.y * L_JOINT2_MOUSE_SEN; // 在使用图传时，微调角度取得更好的视野

    }
    else if (toe_is_error(VT_TOE) == 0)
    {
        *add_j1 = -gimbal_control_set->vt_rc_control->mouse_y * L_JOINT1_MOUSE_SEN;
        *add_j2 = -gimbal_control_set->vt_rc_control->mouse_y * L_JOINT2_MOUSE_SEN;
    }
}

/**
  * @brief          当云台行为模式是GIMBAL_ANGLE_CONTROL, 这个函数会被调用,云台控制模式是当云台行为模式是GIMBAL_ANGLE_CONTROL_MODE
  *                 云台角度跟随操作手指令
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      add_j1:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_angle_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || add_j1 == NULL || add_j2 == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    gimbal_control_set->gimbal_yaw_motor.mit_on_off = MIT_ON;
    static int16_t yaw_channel = 0, j1_channel = 0, j2_channel = 0;

    if(toe_is_error(DBUS_TOE) == 0)
    {
        rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND)
        rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[J1_CHANNEL], j1_channel, RC_DEADBAND)
        rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[J2_CHANNEL], j2_channel, RC_DEADBAND)
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        rc_deadband_limit(gimbal_control_set->vt_rc_control->ch_3, yaw_channel, RC_DEADBAND)
        rc_deadband_limit(gimbal_control_set->vt_rc_control->ch_2, j1_channel, RC_DEADBAND)
        rc_deadband_limit(-gimbal_control_set->vt_rc_control->wheel, j2_channel, RC_DEADBAND)
    }

    /* 使用摇杆或者鼠标控制 */
    *yaw    = yaw_channel * YAW_RC_SEN;
    *add_j1 = j1_channel * L_JOINT1_RC_SEN;
    *add_j2 = j2_channel * L_JOINT2_RC_SEN;
//    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
}

/**
  * @brief          云台立刻保持静止
  * @author         RM
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_motionless_control(float *yaw, float *add_j1, float *add_j2, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || add_j1 == NULL || add_j2 == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    gimbal_control_set->gimbal_yaw_motor.mit_on_off = MIT_ON;

    /* 增量为0 */
    *yaw = 0.0f;
    *add_j1 = 0.0f;
    *add_j2 = 0.0f;
}

/**
  * @brief          跟随自定义控制器，目标角度值来自图传链路
  * @author         Light
  * @param[in]      yaw: yaw/j0轴角度控制，为目标角度 单位 rad
  * @param[in]      j1: j1轴角度控制，为目标 单位 rad
  * @param[in]      j2: j2轴角度控制，为目标 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_custom_control(float *yaw, float *j1, float *j2, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || j1 == NULL || j2 == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    gimbal_control_set->gimbal_yaw_motor.mit_on_off = MIT_ON;

    /* 增量为0 */
    *yaw = 0.0f;
    *j1 = 0.0f;
    *j2 = 0.0f;
}

/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  * @author         RM
  * @param[out]     yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_init_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

//    //初始化状态控制量计算
//    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
//    {
//        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
//        *yaw = 0.0f;
//    }
//    else
//    {
//        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
//        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
//    }
}

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  * @author         RM
  * @param[out]     yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[out]     pitch:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */
static void gimbal_cali_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = 6000;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
//        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
//                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
//                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -10000;
        *yaw = 0;

//        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
//                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
//                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

//        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
//                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
//                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

//        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
//                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
//                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  * @retval         none
  */

static void gimbal_absolute_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{

    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[J1_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * L_JOINT1_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * L_JOINT1_MOUSE_SEN;

    //*yaw  = yaw_channel * YAW_GYRO_SEN;
    //*pitch = pitch_channel * PITCH_GYRO_SEN;

    // abs_limit(pitch,3.04);
    // abs_limit(yaw,3.04);
}

/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @param[in]      yaw: yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch: pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void gimbal_relative_angle_control(float *yaw, float *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
//    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
//    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
}



