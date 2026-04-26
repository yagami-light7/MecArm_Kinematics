/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "App_Gimbal_Task.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "Dev_FDCAN.h"
#include "Alg_UserLib.h"
#include "App_Detect_Task.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "App_Gimbal_Behaviour.h"
#include "App_INS_Task.h"
#include "Alg_PID.h"
#include "UC_Referee.h"
#include "App_Mechanical_Arm_Task.h"
#include "APL_RC_Hub.h"

/* 电机编码值规整 0—8191 */
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

/* PID初始化 */
#define gimbal_total_pid_clear(gimbal_clear)                                                    \
    {                                                                                           \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);    \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_gyro_speed_pid);      \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);    \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_enconde_speed_pid);   \
        gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_CV_angle_pid);          \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_CV_gyro_speed_pid);            \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_error_pid);                    \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_error_gyro_speed_pid);         \
    }

/* 取绝对值 */
#define int_abs(x) ((x) > 0 ? (x) : (-x))

/* 控制云台小米电机 */
#define FDCAN_cmd_yaw(POS, VEL, KP, KD, TORQ)     MIT_Ctrl_L_Joint(&Large_Joint_CAN, 1, POS, VEL, KP, KD, TORQ)
#define FDCAN_cmd_l_j1(POS, VEL, KP, KD, TORQ)    MIT_Ctrl_L_Joint(&Large_Joint_CAN, 2, POS, VEL, KP, KD, TORQ)
#define FDCAN_cmd_l_j2(POS, VEL, KP, KD, TORQ)    MIT_Ctrl_L_Joint(&Large_Joint_CAN, 3, POS, VEL, KP, KD, TORQ)

/* 电机PID参数值 */
#define KP_YAW 1.0f
#define KD_YAW 0.0f


#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t gimbal_high_water;
#endif

float yaw_angle,yaw_angle_set;
float add_yaw = 0,add_pitch = 0;

gimbal_control_t gimbal_control;

//pid数组赋值
static const float Yaw_gyro_speed_pid[3] = {YAW_GYRO_SPEED_PID_KP, YAW_GYRO_SPEED_PID_KI, YAW_GYRO_SPEED_PID_KD};

static const float Yaw_error_pid[3] = {YAW_ERROR_PID_KP, YAW_ERROR_PID_KI, YAW_ERROR_PID_KD};

static const float Yaw_error_gyro_speed_pid[3] = {YAW_ERROR_GYRO_SPEED_PID_KP, YAW_ABSOLUTE_ANGLE_PID_KD, YAW_ABSOLUTE_ANGLE_PID_KD};

//五次多项式数组
static const float yaw_polynomial[6] = {-0.00572f, -0.02049, 0.0665360f,-9.9368f,1.5493f, -9.9398f};

// PID参数相关函数
static void gimbal_PID_init(gimbal_PID_t *pid, float maxout, float intergral_limit, float kp, float ki, float kd);
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);
static float gimbal_PID_calc(gimbal_PID_t *pid, float get, float set, float error_delta);

// 主循环中流程函数
static void gimbal_init(gimbal_control_t *init);
static void gimbal_set_mode(gimbal_control_t *set_mode);
static void gimbal_feedback_update(gimbal_control_t *feedback_update);
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);
static float motor_ecd_to_angle_change(float ecd, float offset_ecd);
static void gimbal_set_control(gimbal_control_t *set_control);
static void gimbal_control_loop(gimbal_control_t *control_loop);

// 不同模式下的控制
static void gimbal_motor_absolute_yaw_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);
static void gimbal_motor_error_control(gimbal_motor_t *gimbal_motor);

// 不同模式下的限位控制
static void gimbal_absolute_yaw_angle_limit(gimbal_control_t *gimbal_control, float add);
static void gimbal_relative_yaw_angle_limit(gimbal_motor_t *gimbal_motor, float add);

/**
 * @brief          云台校准计算
 * @param[in]      gimbal_cali: 校准数据
 * @param[out]     yaw_offset:yaw电机云台中值
 * @param[out]     pitch_offset:pitch 电机云台中值
 * @param[out]     max_yaw:yaw 电机最大机械角度
 * @param[out]     min_yaw: yaw 电机最小机械角度
 * @param[out]     max_pitch: pitch 电机最大机械角度
 * @param[out]     min_pitch: pitch 电机最小机械角度
 * @retval         none
 */
static void calc_gimbal_cali(const gimbal_step_cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, float *max_yaw, float *min_yaw, float *max_pitch, float *min_pitch);

#if GIMBAL_TEST_MODE
// j-scope 帮助pid调参
static void J_scope_gimbal_test(void);
#endif

/**
 * @brief          机械臂大臂任务，间隔 GIMBAL_CONTROL_TIME 1ms
 *                 机械臂末端任务在App_Mechanical_Arm_Task.c中
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void Gimbal_Task(void *pvParameters)
{
    /* 空闲以待初始化 */
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    // 云台初始化
    gimbal_init(&gimbal_control);

    /* 判断上电校准是否完成 */
    while (joint_cali.calibrate_finished_flag != CALIBRATE_FINISHED)
    {
        // 控制模式切换 控制数据过渡
        gimbal_mode_change_control_transit(&gimbal_control);

        /* 下发PID计算 */
        FDCAN_cmd_l_j1(gimbal_control.gimbal_l_joint1.target_angle, gimbal_control.gimbal_l_joint1.target_velocity,
                       gimbal_control.gimbal_l_joint1.kp, gimbal_control.gimbal_l_joint1.kd,
                       gimbal_control.gimbal_l_joint1.target_torq);
        vTaskDelay(GIMBAL_CONTROL_TIME);

        FDCAN_cmd_l_j2(gimbal_control.gimbal_l_joint2.target_angle, gimbal_control.gimbal_l_joint2.target_velocity,
                       gimbal_control.gimbal_l_joint2.kp, gimbal_control.gimbal_l_joint2.kd,
                       gimbal_control.gimbal_l_joint2.target_torq);
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }

    while (1)
    {
        // 设置云台控制模式
        gimbal_set_mode(&gimbal_control);

        // 控制模式切换 控制数据过渡
        gimbal_mode_change_control_transit(&gimbal_control);

        // 云台数据反馈
         gimbal_feedback_update(&gimbal_control);

        // 设置云台控制量
        gimbal_set_control(&gimbal_control);

        // 若电管断电
//        if (robot_state.mains_power_gimbal_output == 0 || (toe_is_error(VT_TOE) && toe_is_error(DBUS_TOE)))
        if ((toe_is_error(VT_TOE) && toe_is_error(DBUS_TOE)))
        {
            // 关闭所有气泵
            pump_master_off
            sucker_left_off
            sucker_right_off

            /* YAW/Joint0 */
            gimbal_control.gimbal_yaw_motor.kp = 0.0f;
            gimbal_control.gimbal_yaw_motor.kd = 1.0f;

            /* Joint1 */
            gimbal_control.gimbal_l_joint1.kp = 0.0f;
            gimbal_control.gimbal_l_joint1.kd = 2.0f;

            /* Joint2 */
            gimbal_control.gimbal_l_joint2.kp = 0.0f;
            gimbal_control.gimbal_l_joint2.kd = 4.0f;

            gimbal_control.gimbal_yaw_motor.target_angle = 0.0f;
            gimbal_control.gimbal_l_joint1.target_angle = 0.0f;
            gimbal_control.gimbal_l_joint2.target_angle = 0.0f;

            gimbal_control.gimbal_yaw_motor.target_velocity = 0.0f;
            gimbal_control.gimbal_l_joint1.target_velocity = 0.0f;
            gimbal_control.gimbal_l_joint2.target_velocity = 0.0f;

            gimbal_control.gimbal_yaw_motor.target_torq = 0.0f;
            gimbal_control.gimbal_l_joint1.target_torq = 0.0f;
            gimbal_control.gimbal_l_joint2.target_torq = 0.0f;
        }

        /* l_joint0 */
        FDCAN_cmd_yaw(gimbal_control.gimbal_yaw_motor.target_angle, gimbal_control.gimbal_yaw_motor.target_velocity,
                      gimbal_control.gimbal_yaw_motor.kp, gimbal_control.gimbal_yaw_motor.kd,
                      gimbal_control.gimbal_yaw_motor.target_torq);
        osDelay(1);

        FDCAN_cmd_l_j1(gimbal_control.gimbal_l_joint1.target_angle, gimbal_control.gimbal_l_joint1.target_velocity,
                       gimbal_control.gimbal_l_joint1.kp, gimbal_control.gimbal_l_joint1.kd,
                       gimbal_control.gimbal_l_joint1.target_torq);
        osDelay(1);

        FDCAN_cmd_l_j2(gimbal_control.gimbal_l_joint2.target_angle, gimbal_control.gimbal_l_joint2.target_velocity,
                       gimbal_control.gimbal_l_joint2.kp, gimbal_control.gimbal_l_joint2.kd,
                       gimbal_control.gimbal_l_joint2.target_torq);
        osDelay(1);



#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif
        vTaskDelay(GIMBAL_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    // 电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = Get_Large_Joint_Motor_Measure_Point(0); //将yaw轴电机的指针存入云台结构体中
    init->gimbal_l_joint1.gimbal_motor_measure  = Get_Large_Joint_Motor_Measure_Point(1);
    init->gimbal_l_joint2.gimbal_motor_measure  = Get_Large_Joint_Motor_Measure_Point(2);

    // 陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();

    // 遥控器数据指针获取
    init->gimbal_rc_ctrl = &RC_Hub.dr16_control;

    // 图传链路键鼠数据指针
    init->gimbal_rc_ctrl_0x304 = &RC_Hub.remote_control;

    // VT远程控制指针
    init->vt_rc_control = &RC_Hub.vt_rc_control;

    // 自定义控制器数据结构体获取
    init->custom_ctrl = &RC_Hub.custom_control;

    /* 初始化低通滤波参数，以对小米电机回传的测量参数进行滤波处理 */
    static float mit_motor_filter_num[1] = {4.51335545855564f};
    first_order_filter_init(&init->mit_motor_filter, GIMBAL_CONTROL_TIME, mit_motor_filter_num);

    /* 初始化斜坡函数，用于一键取矿/存矿 */
    ramp_init(&init->gimbal_yaw_motor._gimbal_s_mine_ramp_, GIMBAL_CONTROL_TIME_S, GIMBAL_YAW_MOTOR_MAX_ANGLE     , GIMBAL_YAW_MOTOR_MIN_ANGLE     );
    ramp_init(&init->gimbal_l_joint1._gimbal_s_mine_ramp_ , GIMBAL_CONTROL_TIME_S, GIMBAL_L_JOINT1_MOTOR_MAX_ANGLE, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE);
    ramp_init(&init->gimbal_l_joint2._gimbal_s_mine_ramp_ , GIMBAL_CONTROL_TIME_S, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE, GIMBAL_L_JOINT2_MOTOR_MIN_ANGLE);

    Ramp_Init(&init->gimbal_yaw_motor._gimbal_l_mine_ramp_, GIMBAL_CONTROL_TIME_S);
    Ramp_Init(&init->gimbal_l_joint1._gimbal_l_mine_ramp_, GIMBAL_CONTROL_TIME_S);
    Ramp_Init(&init->gimbal_l_joint2._gimbal_l_mine_ramp_, GIMBAL_CONTROL_TIME_S);

    // 获取机器人id
    robot_id(&init->car_id);

    // 初始化电机模式
    init->gimbal_motor_mode = init->last_gimbal_motor_mode = GIMBAL_ZERO_FORCE_MODE;

    gimbal_feedback_update(init);

    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_enconde_speed_set = init->gimbal_yaw_motor.motor_enconde_speed;

}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }

    /* 云台数据更新 */
    // 角度信息
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->pos,0);
    feedback_update->gimbal_l_joint1.relative_angle  =  motor_ecd_to_angle_change(feedback_update->gimbal_l_joint1.gimbal_motor_measure->pos, 0);
    feedback_update->gimbal_l_joint2.relative_angle  =  motor_ecd_to_angle_change(feedback_update->gimbal_l_joint2.gimbal_motor_measure->pos, 0);
}


/*
 * 小米铁蛋电机编码测试数据
 * 一圈：编码值pos变化3.2左右 右转增加
 * pos：[-12.5,12.5]
 * 这里对原始编码数据进行循环限幅处理得到相对角度（单位rad）
 * */
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 *                 小米铁蛋电机编码测试数据
 *                 半圈（180°）：编码值pos变化3.2左右 右转增加
 *                 pos：[-12.5,12.5]
 *                 这里对原始编码数据进行循环限幅处理并选择正确的旋转方向得到相对角度（单位rad）
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static float motor_ecd_to_angle_change(float ecd, float offset_ecd)
{
    float relative_ecd = ecd - offset_ecd;

    if (relative_ecd > MIT_HALF_ECD_RANGE)
    {
        relative_ecd -= MIT_ECD_RANGE;
    }
    else if (relative_ecd < -MIT_HALF_ECD_RANGE)
    {
        relative_ecd += MIT_ECD_RANGE;
    }

    return relative_ecd * MIT_MOTOR_ECD_TO_RAD;
}


/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }

    /* 云台模式更改 */
    if (gimbal_mode_change->gimbal_motor_mode != gimbal_mode_change->last_gimbal_motor_mode)
    {
        // 大关节电机状态机切换保存数据
        if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_ZERO_FORCE_MODE &&
            gimbal_mode_change->gimbal_motor_mode == GIMBAL_ZERO_FORCE_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 0.0f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 1.0f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 0.0f;
            gimbal_mode_change->gimbal_l_joint1.kd = 2.0f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 0.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 4.0f;

            /* 无力模式切入角度控制模式，会迅速回到上一个时刻的角度控制值，在此清零，默认归零位 */
            gimbal_mode_change->gimbal_yaw_motor.target_angle = 0.0f;
            gimbal_mode_change->gimbal_l_joint1.target_angle = 0.0f;
            gimbal_mode_change->gimbal_l_joint2.target_angle = 0.0f;

        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_ZERO_POSITION_MODE &&
                   gimbal_mode_change->gimbal_motor_mode == GIMBAL_ZERO_POSITION_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 7.5f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 2.5f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 5.0f;
            gimbal_mode_change->gimbal_l_joint1.kd = 2.0f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 5.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 2.0f;
        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_ANGLE_CONTROL_MODE &&
                   gimbal_mode_change->gimbal_motor_mode == GIMBAL_ANGLE_CONTROL_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 15.0f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 3.0f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 15.0f;
            gimbal_mode_change->gimbal_l_joint1.kd = 3.0f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 15.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 2.0f;
        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_MOTIONLESS_MODE &&
                   gimbal_mode_change->gimbal_motor_mode == GIMBAL_MOTIONLESS_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 15.0f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 3.0f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 15.0f;
            gimbal_mode_change->gimbal_l_joint1.kd = 3.0f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 15.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 2.0f;
        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_CUSTOM_MODE &&
                   gimbal_mode_change->gimbal_motor_mode == GIMBAL_CUSTOM_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 12.5f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 1.5f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 12.5f;
            gimbal_mode_change->gimbal_l_joint1.kd = 1.5f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 25.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 1.0f;
        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_AUTO_GET_GND_MINE_MODE &&
                   gimbal_mode_change->gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 12.5f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 3.0f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 12.5f;
            gimbal_mode_change->gimbal_l_joint1.kd = 5.0f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 25.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 8.0f;
        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_AUTO_GET_L_MINE_MODE &&
                 gimbal_mode_change->gimbal_motor_mode == GIMBAL_AUTO_GET_L_MINE_MODE)
        {
            /* YAW/Joint0 */
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 12.5f;
            gimbal_mode_change->gimbal_yaw_motor.kd = 1.0f;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 12.5f;
            gimbal_mode_change->gimbal_l_joint1.kd = 1.0f;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 25.0f;
            gimbal_mode_change->gimbal_l_joint2.kd = 1.0f;
        }
        else if (gimbal_mode_change->last_gimbal_motor_mode != GIMBAL_CALI_MODE &&
                 gimbal_mode_change->gimbal_motor_mode == GIMBAL_CALI_MODE)
        {
            gimbal_mode_change->gimbal_yaw_motor.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_yaw_motor.kp = 0;
            gimbal_mode_change->gimbal_yaw_motor.kd = 0;

            /* Joint1 */
            gimbal_mode_change->gimbal_l_joint1.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint1.kp = 0;
            gimbal_mode_change->gimbal_l_joint1.kd = 0;

            /* Joint2 */
            gimbal_mode_change->gimbal_l_joint2.mit_on_off = MIT_ON;
            gimbal_mode_change->gimbal_l_joint2.kp = 0;
            gimbal_mode_change->gimbal_l_joint2.kd = 0;
        }

    }
    gimbal_mode_change->last_gimbal_motor_mode = gimbal_mode_change->gimbal_motor_mode;
}

// 用于调试，记录每次的增量


/* 一键地矿 */
float _gnd_mine_emulation_ = 0;     // 俯仰滚轮累计值，用于控制一键地矿
float _store_emulation_ = 0;        // 累计到达一定值，表明存在左侧/右侧吸盘
float _take_emulation_ = 0;         // 累计到达一定值，表明进行放矿/拿矿
int16_t _gnd_mine_flag = 0;
int16_t _store_flag = 0;
int16_t _take_flag = 0;

/* 一键大资源岛矿 */
float _gimbal_l_mine_emulation_ = 0;       // 累计到达一定值，表明选择资源岛的三个通道

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    /* 控制值
     * 1.遥控器增量值
     * 2.自定义控制器目标值
     * 3.视觉目标值
     * */
    float add_yaw_angle = 0.0f;
    float add_l_j1_angle = 0.0f;
    float add_l_j2_angle = 0.0f;

    /* 获取控制值 */
    gimbal_behaviour_control_set(&add_yaw_angle, &add_l_j1_angle, &add_l_j2_angle ,set_control);

    /*  对增量进行处理，转化为最后下发给MIT电机的目标参数
     *  拷贝云台目标设定值,kp kd等其它参数已经在gimbal_mode_change_control_transit中设置了
     *  */
    // 电机模式控制
    if(set_control->gimbal_motor_mode == GIMBAL_ZERO_FORCE_MODE)
    {
        /* gimbal_mode_change_control_transit中设置了输出为0*/
        set_control->gimbal_yaw_motor.target_angle    = 0.0f;
        set_control->gimbal_yaw_motor.target_torq     = 0.0f;
        set_control->gimbal_yaw_motor.target_velocity = 0.0f;

        set_control->gimbal_l_joint1.target_angle     = 0.0f;
        set_control->gimbal_l_joint1.target_torq      = 0.0f;
        set_control->gimbal_l_joint1.target_velocity  = 0.0f;

        set_control->gimbal_l_joint2.target_angle     = 0.0f;
        set_control->gimbal_l_joint2.target_torq      = 0.0f;
        set_control->gimbal_l_joint2.target_velocity  = 0.0f;
    }
    else if(set_control->gimbal_motor_mode == GIMBAL_ZERO_POSITION_MODE)
    {
        set_control->gimbal_yaw_motor.target_angle = 0.0f;
        set_control->gimbal_yaw_motor.target_velocity = 0.0f;
        set_control->gimbal_yaw_motor.target_torq = 0.0f;



#if _CUSTOM_USED_
        set_control->gimbal_l_joint1.target_angle += - add_l_j1_angle * 4.0f;
        set_control->gimbal_l_joint1.target_velocity = 0.0f;
        set_control->gimbal_l_joint1.target_torq = motor_l_joint[1].tor;

        set_control->gimbal_l_joint2.target_angle += add_l_j2_angle * 1.5f;
        set_control->gimbal_l_joint2.target_velocity = 0.0f;
        set_control->gimbal_l_joint2.target_torq = motor_l_joint[2].tor;

        /* 进行限幅 */
        set_control->gimbal_l_joint1.target_angle = float_constrain(set_control->gimbal_l_joint1.target_angle, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 1.2f, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 2.0f);
        set_control->gimbal_l_joint2.target_angle = float_constrain(set_control->gimbal_l_joint2.target_angle, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 0.6f, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 0.0f);
#else
        set_control->gimbal_l_joint1.target_angle = -0.1f;
        set_control->gimbal_l_joint1.target_velocity = 0.0f;
        set_control->gimbal_l_joint1.target_torq = motor_l_joint[1].tor;

        set_control->gimbal_l_joint2.target_angle = -0.3f;
        set_control->gimbal_l_joint2.target_velocity = 0.0f;
        set_control->gimbal_l_joint2.target_torq = motor_l_joint[2].tor;

#endif
    }
    else if(set_control->gimbal_motor_mode == GIMBAL_ANGLE_CONTROL_MODE)
    {
        set_control->gimbal_yaw_motor.target_angle += -add_yaw_angle;
        set_control->gimbal_yaw_motor.target_velocity = 0.0f;
        set_control->gimbal_yaw_motor.target_torq = 0.0f;

        set_control->gimbal_l_joint1.target_angle += -add_l_j1_angle;
        set_control->gimbal_l_joint1.target_velocity = 0.0f;
        set_control->gimbal_l_joint1.target_torq = 0.0f;

        set_control->gimbal_l_joint2.target_angle += -add_l_j2_angle;
        set_control->gimbal_l_joint2.target_velocity = 0.0f;
        set_control->gimbal_l_joint2.target_torq = 0.0f;
    }
    else if(set_control->gimbal_motor_mode == GIMBAL_MOTIONLESS_MODE)
    {
        /* 大关节目标角度不做更改 */
//        set_control->gimbal_yaw_motor.target_angle = set_control->gimbal_yaw_motor.target_angle;
//
//        set_control->gimbal_l_joint1.target_angle  = set_control->gimbal_yaw_motor.target_angle;
//
//        set_control->gimbal_l_joint2.target_angle  = set_control->gimbal_yaw_motor.target_angle;
    }
    else if(set_control->gimbal_motor_mode == GIMBAL_CUSTOM_MODE)
    {
        /* 利用斜坡函数进行示教 */
        Ramp_Calc(&set_control->gimbal_yaw_motor._gimbal_l_mine_ramp_, gimbal_control.custom_ctrl->custom_angle_set[0], 0.020f);
        set_control->gimbal_yaw_motor.target_angle = set_control->gimbal_yaw_motor._gimbal_l_mine_ramp_.current;
        set_control->gimbal_yaw_motor.target_velocity = 0.0f;
        set_control->gimbal_yaw_motor.target_torq = 0.0f;

        Ramp_Calc(&set_control->gimbal_l_joint1._gimbal_l_mine_ramp_, gimbal_control.custom_ctrl->custom_angle_set[1], 0.020f);
        set_control->gimbal_l_joint1.target_angle  = set_control->gimbal_l_joint1._gimbal_l_mine_ramp_.current;
        set_control->gimbal_l_joint1.target_velocity = 0.0f;
        set_control->gimbal_l_joint1.target_torq = 0.0f;

        Ramp_Calc(&set_control->gimbal_l_joint2._gimbal_l_mine_ramp_, gimbal_control.custom_ctrl->custom_angle_set[2], 0.020f);
        set_control->gimbal_l_joint2.target_angle  = set_control->gimbal_l_joint2._gimbal_l_mine_ramp_.current;
        set_control->gimbal_l_joint2.target_velocity = 0.0f;
        set_control->gimbal_l_joint2.target_torq = 0.0f;

        /* 对关节电机进行限幅重映射 */
        gimbal_control.gimbal_yaw_motor.target_angle = float_constrain(gimbal_control.gimbal_yaw_motor.target_angle, GIMBAL_YAW_MOTOR_MIN_ANGLE, GIMBAL_YAW_MOTOR_MAX_ANGLE);
        gimbal_control.gimbal_l_joint1.target_angle  = float_remap(gimbal_control.gimbal_l_joint1.target_angle,  -0.3, 2.2, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 1.2f,  GIMBAL_L_JOINT1_MOTOR_MAX_ANGLE);
        gimbal_control.gimbal_l_joint2.target_angle  = float_remap(gimbal_control.gimbal_l_joint2.target_angle,  -2.9, 0.3, GIMBAL_L_JOINT2_MOTOR_MIN_ANGLE, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE);
    }
    else if (set_control->gimbal_motor_mode == GIMBAL_AUTO_GET_GND_MINE_MODE)
    {
        /* DT7累计控制值 */
        _gnd_mine_emulation_ += add_l_j2_angle;
        _store_emulation_    += add_yaw_angle  * 0.00003;
        _take_emulation_     += add_l_j1_angle * 0.00003;

        /* 控制值限幅 */
        _gnd_mine_emulation_ = float_constrain(_gnd_mine_emulation_, 0, GIMBAL_GND_MINE_LIMITATION);
        _store_emulation_    = float_constrain(_store_emulation_,    -GIMBAL_GND_STORE_LIMITATION,GIMBAL_GND_STORE_LIMITATION);
        _take_emulation_     = float_constrain(_take_emulation_,     0,GIMBAL_GND_TAKE_LIMITATION);


        /* 进行取矿 */
        if(_gnd_mine_emulation_ <  GIMBAL_GND_MINE_LIMITATION * 0.9f)
        {
            set_control->gimbal_yaw_motor.target_angle = Arm_Linear_Move(set_control->gimbal_yaw_motor.gimbal_motor_measure->pos, 0.0f, 1.2f);
            set_control->gimbal_l_joint1.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint1.gimbal_motor_measure->pos, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 4.35f, 1.50f);
            set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 1.55f, 0.50f);
        }
        /* 取矿完毕 进行存矿/取回先前存的矿 */
        else
        {
            /* 手上拿一个 / 归中 */
            if(fabsf(_store_emulation_) < GIMBAL_GND_STORE_LIMITATION * 0.9f)
            {
                /* J0回中 */
                set_control->gimbal_yaw_motor.target_angle = Arm_Linear_Move(set_control->gimbal_yaw_motor.gimbal_motor_measure->pos, 0.0f, 1.2f);

                /* J1回中 */
                set_control->gimbal_l_joint1.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint1.gimbal_motor_measure->pos, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 1.20f, 1.45f);

                /* J2处理 */
                if(gimbal_control.gimbal_l_joint1.gimbal_motor_measure->pos > GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 2.5f)
                {
                    /* 地矿垂直上升阶段 */
                    set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 2.30f, 0.080f);
                }
                else
                {
                    /* 地矿已脱离资源岛阶段 */
                    if(fabsf(gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->pos - 0) > 0.2f )
                    {
                        // 取回时 不要撞掉矿石
                        set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 1.5f, 0.20f);
                    }
                    else
                    {
                        set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 0.8f, 0.60f);
                    }
                }
            }
            /* 存放在左侧吸盘 */
            else if(_store_emulation_ > GIMBAL_GND_STORE_LIMITATION * 0.9f)
            {
                set_control->gimbal_yaw_motor.target_angle = Arm_Linear_Move(set_control->gimbal_yaw_motor.gimbal_motor_measure->pos, -2.50f, 1.80f);
                set_control->gimbal_l_joint1.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint1.gimbal_motor_measure->pos, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 2.0f, 1.00f);

                if( fabsf( set_control->gimbal_yaw_motor.gimbal_motor_measure->pos + 2.50f ) < 0.5f)
                {
                    set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 0.95f, 0.08f);
                }
                else
                {
                    set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 1.4f, 0.60f);
                }

                sucker_left_control

            }
            /* 存放在右侧吸盘 */
            else if(_store_emulation_ < -GIMBAL_GND_STORE_LIMITATION * 0.9f)
            {
                set_control->gimbal_yaw_motor.target_angle = Arm_Linear_Move(set_control->gimbal_yaw_motor.gimbal_motor_measure->pos, +2.50f, 1.80f);
                set_control->gimbal_l_joint1.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint1.gimbal_motor_measure->pos, GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 2.00f, 1.00f);

                if( fabsf( set_control->gimbal_yaw_motor.gimbal_motor_measure->pos - 2.50f ) < 0.5f)
                {
                    set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 0.95f, 0.08f);
                }
                else
                {
                    set_control->gimbal_l_joint2.target_angle  = Arm_Linear_Move(set_control->gimbal_l_joint2.gimbal_motor_measure->pos, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE - 1.4f, 0.60f);
                }
                sucker_right_control

            }

        }

    }
    else if (set_control->gimbal_motor_mode == GIMBAL_AUTO_GET_L_MINE_MODE) // 未实装
    {
        /* 累计控制值 */
        _gimbal_l_mine_emulation_ += add_yaw_angle  * 0.00003;
        _gimbal_l_mine_emulation_  = float_constrain(_gimbal_l_mine_emulation_, -GIMBAL_GND_STORE_LIMITATION * 4,GIMBAL_GND_STORE_LIMITATION * 4);

        /* 选择中间通道 */
        if(fabsf(_gimbal_l_mine_emulation_) < GIMBAL_GND_STORE_LIMITATION * 0.9)
        {
            // YAW
            Ramp_Calc(&gimbal_control.gimbal_yaw_motor._gimbal_l_mine_ramp_, 0.0f, 0.003f);
            // J1
            Ramp_Calc(&gimbal_control.gimbal_l_joint1._gimbal_l_mine_ramp_, 3.2f, 0.0032f);
            // J2
            Ramp_Calc(&gimbal_control.gimbal_l_joint2._gimbal_l_mine_ramp_, -2.30f, 0.0016f + gimbal_control.gimbal_l_joint1._gimbal_l_mine_ramp_.current * gimbal_control.gimbal_l_joint1._gimbal_l_mine_ramp_.current * 0.00048);

        }
        /* 选择左侧通道 */
        else if(_gimbal_l_mine_emulation_ > GIMBAL_GND_STORE_LIMITATION * 0.9)
        {
            // YAW
            Ramp_Calc(&gimbal_control.gimbal_yaw_motor._gimbal_l_mine_ramp_, -0.5f, 0.003f);
            // J1
            Ramp_Calc(&gimbal_control.gimbal_l_joint1._gimbal_l_mine_ramp_, 0.0f, 0.003f);
            // J2
            Ramp_Calc(&gimbal_control.gimbal_l_joint2._gimbal_l_mine_ramp_, 0.0f, 0.003f);
        }
        /* 选择右侧通道 */
        else if(_gimbal_l_mine_emulation_ < -GIMBAL_GND_STORE_LIMITATION * 0.9)
        {
            // YAW
            Ramp_Calc(&gimbal_control.gimbal_yaw_motor._gimbal_l_mine_ramp_, 0.5f, 0.003f);
            // J1
            Ramp_Calc(&gimbal_control.gimbal_l_joint1._gimbal_l_mine_ramp_, 0.0f, 0.003f);
            // J2
            Ramp_Calc(&gimbal_control.gimbal_l_joint2._gimbal_l_mine_ramp_, 0.0f, 0.003f);
        }

        /* 设置控制值 */
        set_control->gimbal_yaw_motor.target_angle = gimbal_control.gimbal_yaw_motor._gimbal_l_mine_ramp_.current;
        set_control->gimbal_yaw_motor.target_velocity = 0.0f;
        set_control->gimbal_yaw_motor.target_torq = 0.0f;

        set_control->gimbal_l_joint1.target_angle  = gimbal_control.gimbal_l_joint1._gimbal_l_mine_ramp_.current;
        set_control->gimbal_l_joint1.target_velocity = 0.0f;
        set_control->gimbal_l_joint1.target_torq = -10.0f;

        set_control->gimbal_l_joint2.target_angle  = gimbal_control.gimbal_l_joint2._gimbal_l_mine_ramp_.current;
        set_control->gimbal_l_joint2.target_velocity = 0.0f;
        set_control->gimbal_l_joint2.target_torq = 0.0f;

    }
    else if (set_control->gimbal_motor_mode == GIMBAL_CALI_MODE) // 校准模式
    {
        // 已经在校准钩子函数中进行设置 仅针对大关节J1
    }

//    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
//    {
//        // raw模式下，直接发送控制值
//        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
//    }
    /*
     * 工程机器人没有安装滑环，故在所有的模式下，底盘和云台/机械臂关节的相对角度应该进行限幅
     *
     * 根据对应的模式，下发目标位置，并进行限幅[-90,90]
     * 1.6是MI电机编码器的90°
     * */
    gimbal_control.gimbal_yaw_motor.target_angle = float_constrain(gimbal_control.gimbal_yaw_motor.target_angle, GIMBAL_YAW_MOTOR_MIN_ANGLE, GIMBAL_YAW_MOTOR_MAX_ANGLE);
    gimbal_control.gimbal_l_joint1.target_angle  = float_constrain(gimbal_control.gimbal_l_joint1.target_angle,  GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE, GIMBAL_L_JOINT1_MOTOR_MAX_ANGLE);
    gimbal_control.gimbal_l_joint2.target_angle  = float_constrain(gimbal_control.gimbal_l_joint2.target_angle,  GIMBAL_L_JOINT2_MOTOR_MIN_ANGLE, GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE);

}

/**
 * @brief          检测J1是否堵转，最初用于校准电机角度数据
 * @param[in]      none
 * @retval         返回1 代表已经堵转， 返回0 代表未堵转
 */
bool detect_mit_j1_block(void)
{
    // 标志位
    static int i = 0;

    /* 根据反馈扭矩判断堵转 */
    if( (fabsf(gimbal_control.gimbal_l_joint1.gimbal_motor_measure->tor) > 2.0f) && (fabsf(gimbal_control.gimbal_l_joint1.gimbal_motor_measure->vel) < 0.3f) )
    {
        /* 峰值计数 */
        i++;
        if(i > 20)
        {
            i = 0;

            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          检测J2是否堵转，最初用于校准电机角度数据
 * @param[in]      none
 * @retval         返回1 代表已经堵转， 返回0 代表未堵转
 */
bool detect_mit_j2_block(void)
{
    // 标志位
    static int i = 0;

    /* 根据反馈扭矩判断堵转 */
    if( (fabsf(gimbal_control.gimbal_l_joint2.gimbal_motor_measure->tor) > 2.0f) && (fabsf(gimbal_control.gimbal_l_joint2.gimbal_motor_measure->vel) < 0.3f))
    {
        /* 峰值计数 */
        i++;
        if(i > 1)
        {
            i = 0;

            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}


/**
 * @brief          返回大关节控制结构体gimbal_control的指针
 * @param[out]     none
 * @retval         none
 */
gimbal_control_t *Get_Gimbal_Control_Point(void)
{
    return &gimbal_control;
}

/**
 * @brief          返回大关节控制结构体gimbal_control的指针
 * @param[out]     none
 * @retval         none
 */
gimbal_motor_mode_e * Get_Gimbal_Mode_Point(void)
{
    return &gimbal_control.gimbal_motor_mode;
}