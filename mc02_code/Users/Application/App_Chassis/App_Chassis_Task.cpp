/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history+
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *  V2.0.0     Nov-8-2024      Light           1. 增加消息队列
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "App_Chassis_Task.h"
#include "App_Chassis_Behaviour.h"

#include "cmsis_os.h"

#include "arm_math.h"
#include "Alg_PID.h"
#include "Dev_Remote_Control.h"
#include "Dev_FDCAN.h"
#include "App_Detect_Task.h"
#include "App_INS_Task.h"
#include "UC_Referee.h"
#include "APL_RC_Hub.h"

/* 消除DT7摇杆死区 */
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

/* 记录堆栈最高值 */
#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t chassis_high_water;
#endif


/**
  * @brief          初始化"chassis_move"变量，包括pid初始化， 遥控器指针初始化，3508底盘电机指针初始化，云台电机初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(Chassis_Move_t *chassis_move_init);


/**
  * @brief          设置底盘控制模式，主要在'Chassis_Behaviour_Mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(Chassis_Move_t *chassis_move_mode);


/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(Chassis_Move_t *chassis_move_transit);


/**
  * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(Chassis_Move_t *chassis_move_update);


/**
  * @brief          设置底盘控制设置值, 三运动控制值是通过chassis_behaviour_control_set函数设置的
  *                 根据DT7遥控的设定值，计算得到步兵的三个自由度上的设定速度
  * @param[out]     chassis_move_control:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(Chassis_Move_t *chassis_move_control);


/**
  * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(Chassis_Move_t *chassis_move_control_loop);

/* 底盘运动结构体 */
Chassis_Move_t chassis_move;

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void Chassis_Task(void *pvParameters)
{
    /* 空闲以待初始化 */
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    /* 底盘初始化 */
    chassis_init(&chassis_move);

    while (1)
    {
        /* 底盘模式设置 */
        chassis_set_mode(&chassis_move);

        /* 底盘模式切换 数据保存 */
        chassis_mode_change_control_transit(&chassis_move);

        /* 底盘数据更新 */
        chassis_feedback_update(&chassis_move);

        /* 底盘控制量设置 */
        chassis_set_contorl(&chassis_move);

        /* 底盘控制循环 */
        chassis_control_loop(&chassis_move);

        /* 确保至少一个电机在线， 这样CAN控制包可以被接收到 */
//        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
//        {
            /* 发送电机电流 */
            FDCAN_cmd_Chassis(chassis_move.motor_chassis[0].Give_Current, chassis_move.motor_chassis[1].Give_Current,
                            chassis_move.motor_chassis[2].Give_Current, chassis_move.motor_chassis[3].Give_Current);

//        }
        /* 任务阻塞 时间片切出 */
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



/**
  * @brief          初始化"chassis_move"变量，包括pid初始化，一阶低通滤波初始化，遥控器指针初始化，3508底盘电机指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
  * @param[out]     chassis_move_init:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_init(Chassis_Move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    /**
      * PART1.  本部分代码将PID参数拷贝进PID参数数组 之后进行初始化 包括以下部分的PID初始化
      *         1.底盘的速度环与角度环
      *         2.待续
      */

    /* PID参数：底盘速度环与角度环 */
    #if CAR_Omnidirectional
        const static float motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP_Omnidirectional, M3505_MOTOR_SPEED_PID_KI_Omnidirectional, M3505_MOTOR_SPEED_PID_KD_Omnidirectional};
        const static float chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP_Omnidirectional, CHASSIS_FOLLOW_GIMBAL_PID_KI_Omnidirectional, CHASSIS_FOLLOW_GIMBAL_PID_KD_Omnidirectional};

    #else //CAR_McNamm
        const static float motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP_McNamm, M3505_MOTOR_SPEED_PID_KI_McNamm, M3505_MOTOR_SPEED_PID_KD_McNamm};
        const static float chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP_McNamm, CHASSIS_FOLLOW_GIMBAL_PID_KI_McNamm, CHASSIS_FOLLOW_GIMBAL_PID_KD_McNamm};

    #endif

    /* PID初始化：获取底盘电机数据指针，底盘的速度环与角度环 */
    #if CAR_Omnidirectional

        for (uint8_t i = 0; i < 4; i++)
        {
            PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT_Omnidirectional, M3505_MOTOR_SPEED_PID_MAX_IOUT_Omnidirectional);
        }

        PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT_Omnidirectional, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT_Omnidirectional);

    #else //CAR_McNamm

        for (uint8_t i = 0; i < 4; i++)
        {
            PID_init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT_McNamm, M3505_MOTOR_SPEED_PID_MAX_IOUT_McNamm);
        }

        PID_init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT_McNamm, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT_McNamm);

    #endif

    /**
      * PART2.  本部分代码将低通滤波参数拷贝进滤波参数数组 之后进行初始化 包括以下部分的低通滤波初始化
      *         1.DT7发送设定值的低通滤波
      *         2.鼠标控制旋转速度的低通滤波
      *         3.待续
      */
    /* 滤波参数：DT7发送设定值的低通滤波 */
    const static float chassis_DT7_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_DT7_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
    const static float chassis_DT7_z_order_filter[1] = {CHASSIS_ACCEL_Z_NUM};

    /* 滤波初始化：DT7发送设定值的低通滤波 */
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_DT7_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_DT7_y_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_wz, CHASSIS_CONTROL_TIME, chassis_DT7_z_order_filter);

    /**
      * PART3.  本部分代码通过获取对应数据的句柄 来降低程序之间的耦合性 包括以下部分的句柄
      *         1.DT7遥控数据指针
      *         2.M3508电机数据指针
      *         3.云台电机指针
      *         4.陀螺仪姿态角数据指针
      *         5.图传链路键鼠数据指针
      *         6.待续
      */

    /* DT7遥控数据指针 */
    chassis_move_init->chassis_RC = &RC_Hub.dr16_control;

    /* 图传链路键鼠数据指针 */
    chassis_move_init->chassis_RC_0x304 = &RC_Hub.remote_control;

    /* VT远程控制指针 */
    chassis_move_init->vt_rc_control = &RC_Hub.vt_rc_control;

    /* M3508电机数据指针 */
    #if CAR_Omnidirectional

        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_init->motor_chassis[i].Chassis_Motor_Measure = get_Chassis_Motor_Measure_point(i);
        }

    #else //CAR_McNamm

        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_init->motor_chassis[i].Chassis_Motor_Measure = Get_Chassis_Motor_Measure_Point(i);
        }

    #endif

    /* 云台电机指针针 */
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
//    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

    /* 陀螺仪姿态角数据指针 */
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();

    /**
      * PART4.  本部分代码初始化一些基本底盘参数
      *         1. 小陀螺旋转速度缩放
      *         2. 底盘默认模式
      *         3. 待续
      */
    /* 小陀螺旋转速度缩放 */
    chassis_move_init->chassis_spin_change_sen = CHASSIS_SPIN_LOW_SPEED;

    /* 底盘默认模式 */
    chassis_move_init->chassis_mode = CHASSIS_ZERO_SPEED;
    /* 底盘X/Y方向最大速度 */
    chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed =  NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

    /* 更新结构体数据 */
    chassis_feedback_update(chassis_move_init);
}



/**
  * @brief          设置底盘控制模式，主要在'Chassis_Behaviour_Mode_set'函数中改变
  * @param[out]     chassis_move_mode:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_mode(Chassis_Move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    Chassis_Behaviour_Mode_Set(chassis_move_mode);//in file "App_Chassis_Behaviour.c"
}



/**
  * @brief          底盘模式改变，有些参数需要改变，例如底盘控制yaw角度设定值应该变成当前底盘yaw角度
  * @param[out]     chassis_move_transit:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_mode_change_control_transit(Chassis_Move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode) // 模式未切换
    {
        return;
    }

//    /* 切入底盘跟随云台模式 将相对角度置零 */
//    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
//    {
//        chassis_move_transit->chassis_relative_angle_set = 0.0f;
//    }

    /* 切入底盘跟随底盘模式 目标角度即为当前角度 */
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

//     /* 切入底盘不跟随模式 目标角度即为当前角度 */
//    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
//    {
//        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
//    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}



/**
  * @brief          根据遥控器通道值，计算机器人三自由度上的速度设定值（z轴的角速度之后再次解算得到）
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
void Chassis_RC_To_Control_Vector(float *vx_set, float *vy_set, Chassis_Move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    /**
      * PART1.  本部分代码将DT7摇杆通道值进行死区滤波，并进行映射计算得到用户期望的速度
      *         1.摇杆死区滤波
      *         2.摇杆映射计算
      *         3.若开启小陀螺并进行键鼠控制，速度（功率）将拉满
      *         4.待续
      */
    static int16_t vx_channel_ref, vy_channel_ref = 0;          // 摇杆原始数据
    static  float  vx_channel_target, vy_channel_target = 0;    // 处理完毕之后的机器人期望速度

    /* 死区滤波：摇杆在中间 其值却不为0 */
    if(toe_is_error(DBUS_TOE) == 0)
    {
        rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel_ref, CHASSIS_RC_DEADLINE)
        rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel_ref, CHASSIS_RC_DEADLINE)
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        rc_deadband_limit(chassis_move_rc_to_vector->vt_rc_control->ch_1, vx_channel_ref, CHASSIS_RC_DEADLINE)
        rc_deadband_limit(chassis_move_rc_to_vector->vt_rc_control->ch_0, vy_channel_ref, CHASSIS_RC_DEADLINE)
    }

    /* 映射计算 */
    vx_channel_target = vx_channel_ref * CHASSIS_VX_RC_SEN;
    vy_channel_target = vy_channel_ref * CHASSIS_VY_RC_SEN;

    /* 使用键鼠控制 */
    if(toe_is_error(DBUS_TOE) == 0)
    {
        if( (chassis_move_rc_to_vector->chassis_RC->key.v & KEY_PRESSED_OFFSET_CTRL) == 0 ) // 没有按下ctrl键
        {
            if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
            {
                vx_channel_target = chassis_move_rc_to_vector->vx_max_speed;
            }
            else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
            {
                vx_channel_target = chassis_move_rc_to_vector->vx_min_speed;
            }

            if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
            {
                vy_channel_target = chassis_move_rc_to_vector->vy_min_speed;
            }
            else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
            {
                vy_channel_target = chassis_move_rc_to_vector->vy_max_speed;
            }
        }
    }
    else if(toe_is_error(VT_TOE) == 0)
    {
        if( (chassis_move_rc_to_vector->vt_rc_control->key & KEY_PRESSED_OFFSET_CTRL) == 0)
        {
            if (chassis_move_rc_to_vector->vt_rc_control->key & CHASSIS_FRONT_KEY)
            {
                vx_channel_target = chassis_move_rc_to_vector->vx_max_speed;
            }
            else if (chassis_move_rc_to_vector->vt_rc_control->key & CHASSIS_BACK_KEY)
            {
                vx_channel_target = chassis_move_rc_to_vector->vx_min_speed;
            }

            if (chassis_move_rc_to_vector->vt_rc_control->key & CHASSIS_LEFT_KEY)
            {
                vy_channel_target = chassis_move_rc_to_vector->vy_min_speed;
            }
            else if (chassis_move_rc_to_vector->vt_rc_control->key & CHASSIS_RIGHT_KEY)
            {
                vy_channel_target = chassis_move_rc_to_vector->vy_max_speed;
            }
        }
    }

    /**
      * PART2.  本部分代码对映射完毕后的期望速度进行处理 并拷贝给速度设定值 包括以下处理
      *         1. 对期望速度进行低通滤波
      *         2. 停止信号触发 直接置零速度 实现急停
      *         3. 待续
      */

    /* 对期望速度进行低通滤波 */
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_channel_target);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_channel_target);

    /* 停止信号，不需要缓慢停止，直接减速到零 */
    if (vx_channel_target < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_channel_target > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN) // 进入摇杆死区
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }

    if (vy_channel_target < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_channel_target > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN) // 进入摇杆死区
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }

    /* 得到机器人速度设定值 */
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}



/**
  * @brief          得到底盘控制设置值, 三自由度上的控制设置值是通过chassis_behaviour_control_set函数设置的
  *                 根据三自由度上的控制设置值，计算得到步兵的三个自由度上的速度设置值
  * @param[out]     chassis_move_control:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_set_contorl(Chassis_Move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    /**
      * PART1.  本部分代码获取三自由度上的控制设定值（DT7），根据行为状态进行不同的解算，从而得到最后的机器人三自由度速度设定值，包括以下行为的解算
      *         1.底盘跟随云台模式  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW
      *         2.底盘跟随yaw模式  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW
      *         3.底盘无角度控制   CHASSIS_VECTOR_NO_FOLLOW_YAW
      *         4.底盘无力模式     CHASSIS_VECTOR_RAW
      *         5.底盘小陀螺模式   CHASSIS_GYROSCOPE
      */

    static float vx_DT7_set = 0.0f, vy_DT7_set = 0.0f, angle_DT7_set = 0.0f ;

    /* 获取三个控制设置值 */
    chassis_behaviour_control_set(&vx_DT7_set, &vy_DT7_set,  &angle_DT7_set, chassis_move_control);

//    /* 底盘跟随云台模式 */
//    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
//    {
//        float sin_yaw = 0.0f, cos_yaw = 0.0f;
//
//        // 旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
//        sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//        cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
//        chassis_move_control->vx_set =  cos_yaw * vx_DT7_set + sin_yaw * vy_DT7_set;
//        chassis_move_control->vy_set = -sin_yaw * vx_DT7_set + cos_yaw * vy_DT7_set;
//
//        // 设置控制相对云台角度
//        chassis_move_control->chassis_relative_angle_set = rad_format(angle_DT7_set);
//
//        // 计算旋转PID角速度
//        chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
//
//        // 速度限幅
//        chassis_move_control->vx_set = float_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
//        chassis_move_control->vy_set = float_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
//    }

    /* 底盘跟随底盘yaw模式 */
    if (chassis_move_control->chassis_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        float delat_angle = 0.0f;

        // 设置底盘控制的角度
        chassis_move_control->chassis_yaw_set = rad_format(angle_DT7_set);
        delat_angle = -rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);

        // 计算旋转的角速度
        chassis_move_control->wz_set = PID_Calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);

        // 速度限幅
        chassis_move_control->vx_set = float_constrain(vx_DT7_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = float_constrain(vy_DT7_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

    /* 底盘无角度控制，只能平移旋转 */
    else if (chassis_move_control->chassis_mode == chassis_move_control->chassis_mode)
    {
        //“angle_DT7_set” 是旋转速度控制
        chassis_move_control->wz_set = angle_DT7_set;
        chassis_move_control->vx_set = float_constrain(vx_DT7_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
        chassis_move_control->vy_set = float_constrain(vy_DT7_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
    }

    /* 底盘无力模式 */
    else if (chassis_move_control->chassis_mode == CHASSIS_ZERO_SPEED)
    {
        // 在原始模式，设置值是发送到CAN总线
        chassis_move_control->vx_set = vx_DT7_set;
        chassis_move_control->vy_set = vy_DT7_set;
        chassis_move_control->wz_set = angle_DT7_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
    }
}

/**
  * @brief          底盘运动学正解算 chassis_feedback_update()中被调用
  * @param[in]      wheel_speed: 底盘电机速度 输入底盘电机结构体Chassis_Motor_t来读转速
  * @param[out]     vx: 步兵纵向速度 m/s
  * @param[out]     vx: 步兵纵向速度 m/s
  * @param[out]     wz: 步兵旋转速度 rad/s
  * @retval         none
  */
static void M3508_speed_to_chassis_vector(float *vx, float *vy, float *wz, const Chassis_Motor_t wheel_speed[4])
{
    /* 定义换算所需基本常数 */
    const float rotate_ratio_f = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_OFFSET) / 1000;    // 线量与角量的转化（半径）
    const float rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_OFFSET) / 1000;
    const float wheel_rpm_ratio = 60.0f / (PERIMETER * PI / 1000 * CHASSIS_DECELE_RATIO);     // 将Rpm转化为m/s

    /* 底盘运动学正解算 */

    /* vx:m/s vy:m/s wz:rad/s */
    *vx = (-wheel_speed[0].Speed + wheel_speed[1].Speed + wheel_speed[2].Speed - wheel_speed[3].Speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX * invSqrt(2) / wheel_rpm_ratio;
    *vy = (-wheel_speed[0].Speed - wheel_speed[1].Speed + wheel_speed[2].Speed + wheel_speed[3].Speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY * invSqrt(2) / wheel_rpm_ratio;
    *wz = (+wheel_speed[0].Speed + wheel_speed[1].Speed + wheel_speed[2].Speed + wheel_speed[3].Speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / rotate_ratio_f  / wheel_rpm_ratio;
}


/**
  * @brief          底盘运动学逆解算 在chassis_control_loop()中被调用
  * @param[in]      vx_set: 步兵纵向速度 m/s
  * @param[in]      vy_set: 步兵横向速度 m/s
  * @param[in]      wz_set: 步兵旋转速度 rad/s
  * @param[out]     wheel_speed: 底盘电机速度Rpm
  * @retval         none
  */
static void chassis_vector_to_M3508_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
    /* 定义换算所需基本常数 */

    static float rotate_ratio_f = 0.0f; // 线量与角量的转化（半径）
    static float rotate_ratio_b = 0.0f; // 旋转比率 = （（轮子半径平均值）+ 由于云台产生的偏差））/ rad

    static float wheel_rpm_ratio = 0.0f; // 将Rpm转化为m/s

    /* 底盘运动学逆解算 最后单位全部由m/s换算为rpm
     * 增益因子的存在是由于DT7解算完毕后的数据进行1:1拷贝，运动效果不令人满意，当然也可以在其他地方添加增益因子，例如：
     *      1. 映射计算 宏CHASSIS_VX_RC_SEN CHASSIS_VY_RC_SEN
     *      2. 控制设定值与速度设定值的转化 chassis_set_contorl()函数中PART1
     *      3. ......
     * 总而言之这一部分可以灵活的调节
     * */
    rotate_ratio_f = (McKnum_Wheel_X + McKnum_Wheel_Y) /1000 / 57.3; // rad/s转化为 m/s
    wheel_rpm_ratio = 60.0f/(PERIMETER * PI / 1000 * CHASSIS_DECELE_RATIO);

    /* 最后单位全部由m/s换算为Rpm*/
    float Vector_DOF_x = vx_set * wheel_rpm_ratio * 0.1;
    float Vector_DOF_y = vy_set * wheel_rpm_ratio * 0.1;
    float Vector_DOF_z = wz_set * rotate_ratio_f * wheel_rpm_ratio * 32;

    wheel_speed[0] = (-Vector_DOF_x - Vector_DOF_y + Vector_DOF_z); //对应Motor1
    wheel_speed[1] = ( Vector_DOF_x - Vector_DOF_y + Vector_DOF_z); //对应Motor2
    wheel_speed[2] = ( Vector_DOF_x + Vector_DOF_y + Vector_DOF_z); //对应Motor3
    wheel_speed[3] = (-Vector_DOF_x + Vector_DOF_y + Vector_DOF_z); //对应Motor0
}


/**
  * @brief          底盘测量数据更新，主要进行底盘运动学正解算，得到机器人三自由度上的速度
  * @param[out]     chassis_move_update:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_feedback_update(Chassis_Move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;

    }

    /* 根据步兵机器人ID 设置不同的底盘角度环PID参数 */
//    static uint16_t car_id;
//    robot_id(&car_id);
//
//    if(car_id == 3 || car_id == 103)
//    {
//        PID_Change(&chassis_move_update->chassis_angle_pid, 40.0, 0.0, 0.0);
//    }
//    else if(car_id == 4 || car_id == 104)
//    {
//        PID_Change(&chassis_move_update->chassis_angle_pid, 15.0, 0.0, 0.0);
//    }

    /**
      * PART1.  本部分代码更新一些底盘测量参数并进行单位换算，包括以下数据
      *         1. 底盘电机转速 角加速度加速度
      *         2. 待续
      */
    /* 更新底盘电机测量数据 */
    for (uint8_t i = 0; i < 4; i++)
    {
        /* CHASSIS_CONTROL_FREQUENCE这个500ms如何确保准确？*/
        chassis_move_update->motor_chassis[i].Speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].Chassis_Motor_Measure->speed_rpm; // 将底盘电机的速度单位转化为Rpm
        chassis_move_update->motor_chassis[i].Accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE; // 加速度是速度的PID微分
    }

    /**
      * PART2.  本部分代码利用电机测量数据和定义的换算常数进行运动学正解算 得到以下数据
      *         1. 机器人x、y自由度上的平动速度 z自由度上的旋转速度
      *         2. 待续
      * Note:   具体解算公式不再赘述，后来者可以查询“中科大电控教程”第五章
      */
    /* 底盘运动学正解算 */
    M3508_speed_to_chassis_vector(&(chassis_move_update->vx),&(chassis_move_update->vy),&(chassis_move_update->wz), chassis_move_update->motor_chassis);

    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
//    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
//    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
}


/**
  * @brief          控制循环，根据控制设定值，PID控制器计算电机电流值，进行输出控制
  *                 将"chassis_set_contorl"中得到的三自由度上的设定速度进行运动学逆解算，得到电机设定转速后进行PID运算
  * @param[out]     chassis_move_control_loop:"chassis_move"变量指针.
  * @retval         none
  */
static void chassis_control_loop(Chassis_Move_t *chassis_move_control_loop)
{
    /*本部分代码包括以下部分：
     * 1.底盘运动学逆解算得到底盘电机速度设定值
     * 2.对电机速度设定值进行线性缩放并计算速度环PID
     * 3.功率控制（内含缓冲能量环PID计算）
     * 4.输出give_current给电机
     * 5.待续
     * */
    float max_vector = 0.0f, vector_rate = 0.0f; // 最大转速以及转速限制下的换算比例

    float temp = 0.0f; // 记录电机转速峰值

    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 拷贝底盘运动解算得到的电机速度数据

    uint8_t i = 0;

    /* 底盘运动学逆解算 */
    chassis_vector_to_M3508_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);

    /* @非RAW模式 将wheel_speed数据进行PID计算，得到电流值后下发 */

    for (i = 0; i < 4; i++)
    {
        /* 底盘电机速度值设定 */
        chassis_move_control_loop->motor_chassis[i].Speed_Set = wheel_speed[i];

        /* 记录电机设定转速峰值 */
        temp = fabs(chassis_move_control_loop->motor_chassis[i].Speed_Set);
        if (max_vector < temp)
        {
            max_vector = temp;
        }
    }

    /* 底盘电机速度缩放 */
    if (max_vector > MAX_WHEEL_SPEED)
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].Speed_Set *= vector_rate;
        }
    }

    /* 底盘电机PID计算 */
    for (i = 0; i < 4; i++)
    {
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].Speed, chassis_move_control_loop->motor_chassis[i].Speed_Set);
    }

    /* 底盘电机功率控制 */
    //chassis_power_control(chassis_move_control_loop);

    /* 底盘电机赋电流值 */
    for (i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].Give_Current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
}
