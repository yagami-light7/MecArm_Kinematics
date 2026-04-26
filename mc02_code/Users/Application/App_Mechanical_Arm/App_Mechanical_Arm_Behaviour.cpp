#include "App_Mechanical_Arm_Behaviour.h"
#include "App_Detect_Task.h"

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


/* 函数声明 */
static void mec_arm_behavour_set(mec_arm_control_t *mec_arm_mode_set);
static void mec_arm_angle_control(float *add_s_j0, float *add_s_j1, float *add_s_j2, mec_arm_control_t *mec_control_set);

/* 创建行为枚举 */
mec_arm_behaviour_e mec_arm_behaviour = MEC_ARM_ZERO_FORCE;
		
void mec_arm_behaviour_mode_set(mec_arm_control_t *mec_arm_mode_set)
{
    if (mec_arm_mode_set == NULL)
    {
        return;
    }

    /* 完成状态的设置 */
    mec_arm_behavour_set(mec_arm_mode_set);

    /* 根据上一步设置的状态执行模式的设置 */
    // 根据末端关节行为状态机设置电机状态机 拷贝行为状态
    if(mec_arm_behaviour == MEC_ARM_ZERO_FORCE && joint_cali.calibrate_finished_flag == CALIBRATE_FINISHED)
    {
        mec_arm_mode_set->mec_arm_mode = MEC_ARM_ZERO_FORCE_MODE;
    }
    else if(mec_arm_behaviour == MEC_ARM_ZERO_POSITION)
    {
        mec_arm_mode_set->mec_arm_mode = MEC_ARM_ZERO_POSITION_MODE;
    }
    else if(mec_arm_behaviour == MEC_ARM_ANGLE_CONTROL)
    {
        mec_arm_mode_set->mec_arm_mode = MEC_ARM_ANGLE_CONTROL_MODE;
    }
    else if(mec_arm_behaviour == MEC_ARM_MOTIONLESS)
    {
        mec_arm_mode_set->mec_arm_mode = MEC_ARM_MOTIONLESS_MODE;
    }
    else if(mec_arm_behaviour == MEC_ARM_CUSTOM)
    {
        mec_arm_mode_set->mec_arm_mode = MEC_ARM_CUSTOM_MODE;
    }
    else if(mec_arm_behaviour == MEC_AUTO_GET_GND_MINE)
    {
        mec_arm_mode_set->mec_arm_mode = MEC_AUTO_GET_GND_MINE_MODE;
    }

}


void mec_arm_behaviour_control_set(float *add_s_j0, float *add_s_j1, float *add_s_j2, mec_arm_control_t *mec_arm_control_set)
{

    if (add_s_j0 == NULL || add_s_j1 == NULL || add_s_j2 == NULL || mec_arm_control_set == NULL)
    {
        return;
    }

    if (mec_arm_behaviour == MEC_ARM_ZERO_FORCE)
    {
        *add_s_j0 = 0.0f;
        *add_s_j1 = 0.0f;
        *add_s_j2 = 0.0f;
    }
    else if (mec_arm_behaviour == MEC_ARM_ZERO_POSITION)
    {
        *add_s_j0 = 0.0f;
        *add_s_j1 = mec_arm_control_set->mec_arm_rc_ctrl->mouse.y * L_JOINT2_MOUSE_SEN;
        *add_s_j2 = 0.0f;

        if(*add_s_j1 == 0)
        {
            *add_s_j1 = mec_arm_control_set->mec_arm_rc_ctrl_0x304->mouse_y * L_JOINT2_MOUSE_SEN;
        }
    }
    else if (mec_arm_behaviour == MEC_ARM_ANGLE_CONTROL)
    {
        mec_arm_angle_control(add_s_j0, add_s_j1, add_s_j2, mec_arm_control_set);
    }
    else if (mec_arm_behaviour == MEC_ARM_MOTIONLESS)
    {
        *add_s_j0 = 0.0f;
        *add_s_j1 = 0.0f;
        *add_s_j2 = 0.0f;
    }
    else if (mec_arm_behaviour == MEC_ARM_CUSTOM)
    {
        mec_arm_angle_control(add_s_j0, add_s_j1, add_s_j2, mec_arm_control_set);
    }
    else if (mec_arm_behaviour == MEC_AUTO_GET_GND_MINE)
    {
        mec_arm_angle_control(add_s_j0, add_s_j1, add_s_j2, mec_arm_control_set);
    }
}

extern int8_t last_left_s;
extern int8_t last_right_s;
extern uint16_t last_v;
extern uint16_t last_keyboard_0x304;

/* 一键地矿 */
extern float _gnd_mine_emulation_;     // 俯仰滚轮累计值，用于控制一键地矿
extern float _store_emulation_;        // 累计到达一定值，表明存在左侧/右侧吸盘
extern float _take_emulation_;         // 累计到达一定值，表明进行放矿/拿矿
extern int16_t _gnd_mine_flag;
extern int16_t _store_flag;
extern int16_t _take_flag;

/* 一键大资源岛矿 */
extern float _gimbal_l_mine_emulation_;       // 累计到达一定值，表明选择资源岛的三个通道


static void mec_arm_behavour_set(mec_arm_control_t *mec_arm_mode_set)
{
    static mec_arm_behaviour_e last_mec_arm_behaviour = MEC_ARM_ZERO_FORCE;

    if (mec_arm_mode_set == NULL)
    {
        return;
    }

    /* 开关控制云台模式 */
    if(toe_is_error(DBUS_TOE) == 0)
    {
        if (switch_is_down(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_RIGHT]))// 右下
        {
            mec_arm_behaviour = MEC_ARM_ZERO_FORCE; // 无力状态
        }
        else if (switch_is_mid(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_RIGHT]))// 右中
        {
            if ( switch_is_down(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_LEFT]) && !switch_is_down(last_left_s) ) // 左下
            {
                mec_arm_behaviour = MEC_ARM_ZERO_POSITION; // 归零位
                //同时自定义控制器也会归零位
            }
            else if (switch_is_mid(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_LEFT]) && !switch_is_mid(last_left_s))// 左中
            {
                #if _CUSTOM_USED_
                mec_arm_behaviour = MEC_ARM_CUSTOM;
                #else
                mec_arm_behaviour = MEC_ARM_MOTIONLESS; // 静止不动
                #endif
            }
            else if( switch_is_up(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_LEFT]) && !switch_is_up(last_left_s) )// 左上
            {
                #if _CUSTOM_USED_
                mec_arm_behaviour = MEC_AUTO_GET_GND_MINE;
                #else
                mec_arm_behaviour = MEC_ARM_ANGLE_CONTROL; // 开始控制末端关节角度
                #endif
            }
            else if(mec_arm_mode_set->gimbal_control->gimbal_motor_mode == GIMBAL_CUSTOM_MODE) // 防止边沿检测不灵敏导致模式相异
            {
                mec_arm_behaviour = MEC_ARM_CUSTOM;
            }
            else if(mec_arm_mode_set->gimbal_control->gimbal_motor_mode == GIMBAL_CUSTOM_MODE) // 防止边沿检测不灵敏导致模式相异
            {
                mec_arm_behaviour = MEC_ARM_CUSTOM;
            }
            else if(mec_arm_mode_set->gimbal_control->gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE) // 防止边沿检测不灵敏导致模式相异
            {
                mec_arm_behaviour = MEC_AUTO_GET_GND_MINE;
            }

        }
        else if (switch_is_up(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_RIGHT])) //右上
        {
            if ( switch_is_down(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_LEFT]) && !switch_is_down(last_left_s) ) // 左下
            {
                mec_arm_behaviour = MEC_ARM_ZERO_POSITION; // 归零位
                //同时自定义控制器也会归零位
            }
            else if (switch_is_mid(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_LEFT]) && !switch_is_mid(last_left_s))// 左中
            {
                #if _CUSTOM_USED_
                mec_arm_behaviour = MEC_ARM_CUSTOM;
                #else
                mec_arm_behaviour = MEC_ARM_MOTIONLESS; // 静止不动
                #endif
            }
            else if( switch_is_up(mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[S_JOINT_MODE_CHANNEL_LEFT]) && !switch_is_up(last_left_s) )// 左上
            {
                #if _CUSTOM_USED_
                mec_arm_behaviour = MEC_AUTO_GET_GND_MINE;
                #else
                mec_arm_behaviour = MEC_ARM_ANGLE_CONTROL; // 开始控制末端关节角度
                #endif
            }
            else if(mec_arm_mode_set->gimbal_control->gimbal_motor_mode == GIMBAL_CUSTOM_MODE) // 防止边沿检测不灵敏导致模式相异
            {
                mec_arm_behaviour = MEC_ARM_CUSTOM;
            }
            else if(mec_arm_mode_set->gimbal_control->gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE) // 防止边沿检测不灵敏导致模式相异
            {
                mec_arm_behaviour = MEC_AUTO_GET_GND_MINE;
            }

        }

    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        if(gimbal_control.gimbal_motor_mode == GIMBAL_ZERO_POSITION_MODE)
        {
            mec_arm_behaviour = MEC_ARM_ZERO_POSITION;
        }
        else if(gimbal_control.gimbal_motor_mode == GIMBAL_CUSTOM_MODE)
        {
            mec_arm_behaviour = MEC_ARM_CUSTOM;
        }
        else if(gimbal_control.gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE)
        {
            mec_arm_behaviour = MEC_AUTO_GET_GND_MINE;
        }
        else if(gimbal_control.gimbal_motor_mode == GIMBAL_ZERO_FORCE_MODE)
        {
            mec_arm_behaviour = MEC_ARM_ZERO_FORCE;
        }
    }


    /* 键鼠操作行为模式，这里的操作会覆盖之前的摇杆操作 */

    // PART0 DT7
    if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_CTRL)) // ctrl + 对应按键 防止误触
    {
        //
        if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_SHIFT) )
        {
            // 删去操作，选择跟随gimbal即可
        }
        else if ( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_F))
        {
            mec_arm_behaviour = MEC_ARM_ZERO_POSITION;
        }

        /* 自动取/存矿键位操作 */
        if(mec_arm_behaviour == MEC_AUTO_GET_GND_MINE)
        {
            // ctrl + W 中存
            if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_W) )
            {
                if((last_v & KEY_PRESSED_OFFSET_W) == 0)
                {
                    _store_emulation_ = 0;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + Q 左存/左取地矿
            else if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q) )
            {
                if((last_v & KEY_PRESSED_OFFSET_Q) == 0)
                {
                    _store_emulation_ = GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + S 下取回地矿
            else if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_S) )
            {
                if((last_v & KEY_PRESSED_OFFSET_S) == 0)
                {
                    _take_emulation_ = GIMBAL_GND_TAKE_LIMITATION;
                }
            }
                // ctrl + E 右存/右取地矿
            else if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E) )
            {
                if((last_v & KEY_PRESSED_OFFSET_E) == 0)
                {
                    _store_emulation_ = -GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + G 控制取地矿
            else if( (mec_arm_mode_set->mec_arm_rc_ctrl->key.v & KEY_PRESSED_OFFSET_G) )
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

    // PART1 图传链路数据
    if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_CTRL)) // ctrl + 对应按键 防止误触
    {
        //
        if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_SHIFT) )
        {
            // 删去操作，选择跟随gimbal即可
        }
        else if ( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_F))
        {
            mec_arm_behaviour = MEC_ARM_ZERO_POSITION;
        }

        /* 自动取/存矿键位操作 */
        if(mec_arm_behaviour == MEC_AUTO_GET_GND_MINE)
        {
            // ctrl + W 中存
            if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_W) )
            {
                if((mec_arm_mode_set->vt_rc_control->mec_arm_key_last & KEY_PRESSED_OFFSET_W) == 0)
                {
                    _store_emulation_ = 0;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + Q 左存/左取地矿
            else if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_Q) )
            {
                if((mec_arm_mode_set->vt_rc_control->mec_arm_key_last & KEY_PRESSED_OFFSET_Q) == 0)
                {
                    _store_emulation_ = GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + S 下取回地矿
            else if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_S) )
            {
                if((mec_arm_mode_set->vt_rc_control->mec_arm_key_last & KEY_PRESSED_OFFSET_S) == 0)
                {
                    _take_emulation_ = GIMBAL_GND_TAKE_LIMITATION;
                }
            }
                // ctrl + E 右存/右取地矿
            else if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_E) )
            {
                if((mec_arm_mode_set->vt_rc_control->mec_arm_key_last & KEY_PRESSED_OFFSET_E) == 0)
                {
                    _store_emulation_ = -GIMBAL_GND_STORE_LIMITATION;
                    _take_emulation_  = 0;
                }
            }
                // ctrl + G 控制取地矿
            else if( (mec_arm_mode_set->vt_rc_control->key & KEY_PRESSED_OFFSET_G) )
            {
                if((mec_arm_mode_set->vt_rc_control->mec_arm_key_last & KEY_PRESSED_OFFSET_G) == 0)
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

        RC_Hub.vt_rc_control.mec_arm_key_last = RC_Hub.vt_rc_control.key;
    }

//    if( toe_is_error(DBUS_TOE) && toe_is_error(VT_TOE))
//    {
//        mec_arm_behaviour = MEC_ARM_ZERO_FORCE;
//    }

    last_mec_arm_behaviour = mec_arm_behaviour;

//    last_left_s  = mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_LEFT];
//    last_right_s = mec_arm_mode_set->mec_arm_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL_RIGHT];
//
//    /* 记录上次键盘 */
//    last_v = mec_arm_mode_set->mec_arm_rc_ctrl->key.v;
}


/**
  * @brief          当末端行为模式是MEC_ARM_ANGLE_CONTROL, 这个函数会被调用,末端控制模式是当末端行为模式是MEC_ARM_ANGLE_CONTROL_MODE
  *                 末端角度跟随操作手指令
  * @param[in]      yaw:发送yaw电机的原始值，会直接通过can 发送到电机
  * @param[in]      add_j1:发送pitch电机的原始值，会直接通过can 发送到电机
  * @param[in]      gimbal_control_set: 云台数据指针
  * @retval         none
  */
static void mec_arm_angle_control(float *add_s_j0, float *add_s_j1, float *add_s_j2, mec_arm_control_t *mec_control_set)
{
    if (add_s_j0 == NULL || add_s_j1 == NULL || add_s_j2 == NULL || mec_control_set == NULL)
    {
        return;
    }

    static int16_t s_j0_channel = 0, s_j1_channel = 0, s_j2_channel = 0;

    if(toe_is_error(DBUS_TOE) == 0)
    {
        rc_deadband_limit(mec_control_set->mec_arm_rc_ctrl->rc.ch[S_JOINT0_CHANNEL], s_j0_channel, RC_DEADBAND);
        rc_deadband_limit(mec_control_set->mec_arm_rc_ctrl->rc.ch[S_JOINT1_CHANNEL], s_j1_channel, RC_DEADBAND);
        rc_deadband_limit(mec_control_set->mec_arm_rc_ctrl->rc.ch[S_JOINT2_CHANNEL], s_j2_channel, RC_DEADBAND);
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        rc_deadband_limit(mec_control_set->vt_rc_control->wheel, s_j0_channel, RC_DEADBAND)
        rc_deadband_limit(mec_control_set->vt_rc_control->ch_2, s_j1_channel, RC_DEADBAND)
        rc_deadband_limit(mec_control_set->vt_rc_control->ch_3, s_j2_channel, RC_DEADBAND)
    }

    *add_s_j0 = s_j0_channel * S_JOINT0_RC_SEN;
    *add_s_j1 = s_j1_channel * S_JOINT1_RC_SEN;
    *add_s_j2 = s_j2_channel * S_JOINT2_RC_SEN;
//    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;
}