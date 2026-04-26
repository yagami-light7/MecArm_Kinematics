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

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "Dev_FDCAN.h"
#include "Alg_PID.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "Alg_UserLib.h"
#include "App_Custom_Task.h"
#include "APL_RC_Hub.h"
#include "Alg_UserLib.h"

//****************
#define DEGREESTORAD 0.01745f
//******************



//用于计算云台陀螺仪角速度的增量式PID
//enconder speed close-loop PID params, max out and max iout
//编码器速度环 PID参数以及 PID最大输出，积分输出
#if CAR_Omnidirectional
#define PITCH_GYRO_SPEED_PID_KP        6000.0f
#define PITCH_GYRO_SPEED_PID_KI        200.0f
#define PITCH_GYRO_SPEED_PID_KD        3000.0f
#define PITCH_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_GYRO_SPEED_PID_MAX_IOUT  3000.0f
#define YAW_GYRO_SPEED_PID_KP        12000.0f
#define YAW_GYRO_SPEED_PID_KI        500.0f
#define YAW_GYRO_SPEED_PID_KD        3000.0f
#define YAW_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_GYRO_SPEED_PID_MAX_IOUT  0.0f

#else
#define YAW_GYRO_SPEED_PID_KP        12000.0f
#define YAW_GYRO_SPEED_PID_KI        500.0f
#define YAW_GYRO_SPEED_PID_KD        3000.0f
#define YAW_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_GYRO_SPEED_PID_MAX_IOUT  0.0f
#endif


//用于计算云台《辅瞄》陀螺仪角速度的增量式PID
//gyro speed close-loop PID params, max out and max iout
//速度环 PID参数以及 PID最大输出，积分输出
#define YAW_ERROR_GYRO_SPEED_PID_KP        10000.0f
#define YAW_ERROR_GYRO_SPEED_PID_KI        0.0f
#define YAW_ERROR_GYRO_SPEED_PID_KD        0.0f
#define YAW_ERROR_GYRO_SPEED_PID_MAX_OUT   30000.0f
#define YAW_ERROR_GYRO_SPEED_PID_MAX_IOUT  0.0f


//用于计算云台绝对值角度的位置式PID参数
//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_ABSOLUTE_ANGLE_PID_KP        18.0f
#define YAW_ABSOLUTE_ANGLE_PID_KI        0.0f
#define YAW_ABSOLUTE_ANGLE_PID_KD        0.0f
#define YAW_ABSOLUTE_ANGLE_PID_MAX_OUT   10.0f
#define YAW_ABSOLUTE_ANGLE_PID_MAX_IOUT  0.0f

//用于计算云台相对值角度的位置式PID参数
//encode angle close-loop PID params, max out and max iout
//角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_RELATIVE_ANGLE_PID_KP        8.0f
#define YAW_RELATIVE_ANGLE_PID_KI        0.0f
#define YAW_RELATIVE_ANGLE_PID_KD        0.0f
#define YAW_RELATIVE_ANGLE_PID_MAX_OUT   10.0f
#define YAW_RELATIVE_ANGLE_PID_MAX_IOUT  0.0f

#define YAW_ERROR_PID_KP        26.0f
#define YAW_ERROR_PID_KI        0.0f
#define YAW_ERROR_PID_KD        0.0f
#define YAW_ERROR_PID_MAX_OUT   10.0f
#define YAW_ERROR_PID_MAX_IOUT  0.0f



//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201

//yaw控制通道
#define YAW_CHANNEL   2       //
//pitch控制通道
#define J1_CHANNEL 3       //
#define J2_CHANNEL 4       //
//状态开关通道
#define GIMBAL_MODE_CHANNEL_RIGHT     0 		    //遥控器右面挡位
#define GIMBAL_MODE_CHANNEL_LEFT      1			    //遥控器左面挡位

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//按键宏定义区
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R

//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10

//摇杆灵敏度
// #define YAW_RC_SEN    -(0.4)/660
// #define PITCH_RC_SEN  -(0.1)/660
#define YAW_RC_SEN           -(0.01)/660
#define L_JOINT1_RC_SEN      -(0.005)/660
#define L_JOINT2_RC_SEN      -(0.005)/660

#define GIMBAL_GND_MINE_LIMITATION      -L_JOINT2_RC_SEN
#define GIMBAL_GND_STORE_LIMITATION     -YAW_RC_SEN * 2.0f
#define GIMBAL_GND_TAKE_LIMITATION      -L_JOINT1_RC_SEN

//鼠标灵敏度
#define YAW_MOUSE_SEN   0.00002f
#define L_JOINT1_MOUSE_SEN     0.00001f
#define L_JOINT2_MOUSE_SEN    -0.00001f

//
#define YAW_ENCONDER_SEN    0.01f
#define PITCH_ENCONDER_SEN  0.01f

//云台任务控制周期
#define GIMBAL_CONTROL_TIME 1
#define GIMBAL_CONTROL_TIME_S 0.001
#define GIMBAL_CONTROL_FREQUENCY 1000

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

//pitch反装，yaw不反装
#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define MIT_HALF_ECD_RANGE  3.2f
#define MIT_ECD_RANGE       6.4f


//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.15f

//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.008f
#define GIMBAL_INIT_YAW_SPEED       0.008f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//电机编码值转化成角度值
#ifndef MIT_MOTOR_ECD_TO_RAD

    #define MIT_MOTOR_ECD_TO_RAD 0.9817476875f //      2*  PI  /6.4

#endif

#define DEGREESTORAD 0.01745f

#define QIANKUI -3000.0f

#define MIT_OFF 0 // 不对MIT电机下发报文 无力模式
#define MIT_ON  1

/* 大关节电机角度最值 */
#define GIMBAL_YAW_MOTOR_MAX_ANGLE        +3.00f
#define GIMBAL_YAW_MOTOR_MIN_ANGLE        -3.00f
#define GIMBAL_L_JOINT1_MOTOR_MAX_ANGLE   gimbal_control.gimbal_l_joint1.MIT_Origin_Angle + 5.45f
#define GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE   gimbal_control.gimbal_l_joint1.MIT_Origin_Angle
#define GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE   gimbal_control.gimbal_l_joint2.MIT_Origin_Angle -0.01f
#define GIMBAL_L_JOINT2_MOTOR_MIN_ANGLE   gimbal_control.gimbal_l_joint2.MIT_Origin_Angle -3.05f

/* 封装线性插值函数，用于大臂的平滑运动 */
#define Arm_Linear_Move(current, target, slope) Linear_Ramp(current, target, slope, GIMBAL_CONTROL_TIME)


typedef enum
{
    GIMBAL_ZERO_FORCE_MODE = 0,// 无力
    GIMBAL_ZERO_POSITION_MODE, // 大关节归零位
    GIMBAL_ANGLE_CONTROL_MODE, // 大关节角度控制
    GIMBAL_MOTIONLESS_MODE,    // 大关节无运动（位置锁住）

    GIMBAL_CUSTOM_MODE,        // 自定义控制器模式，大臂跟随小臂

    /* 自动模式 */
    GIMBAL_AUTO_GET_GND_MINE_MODE,  // 一键取小资源岛地矿模式
    GIMBAL_AUTO_GET_L_MINE_MODE,    // 一键取大资源岛矿石模式

    /* 校准模式 */
    GIMBAL_CALI_MODE,

} gimbal_motor_mode_e;

//云台独立使用Pid结构体
typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} gimbal_PID_t;

//云台电机结构体
typedef struct
{
    //电机反馈参数结构体
    const MIT_Motor_Measure_t *gimbal_motor_measure;

    // 校准 机械限位位置记录
    float MIT_Origin_Angle; // J1别漂！！！

    //校准使用 中值编码值 最大的编码值，最小的编码值
    uint16_t offset_ecd;     //ecd
    float max_relative_angle; //rad
    float min_relative_angle; //rad

    float max_absolute_angle; //rad
    float min_absolute_angle; //rad

    //当前相对角度和目标相对角度
    float relative_angle;     //rad
    float relative_angle_set; //rad

    //当前绝对角度和目标绝对角度
    float absolute_angle;     //rad
    float absolute_angle_set; //rad

    //当前陀螺仪测得的绝对角速度和绝对角速度设定
    float motor_gyro;         //rad/s   主要用这个
    float motor_gyro_set;     //rad/s

    //当前通过电机反馈测得的ECD角速度ECD角速度设定
    float motor_enconde_speed;    //ecd/s
    float motor_enconde_speed_set;//ecd/s

    //直驱模式下发送的电流
    float raw_cmd_current;
    //PID算出的实时电流设定值
    float current_set;
    //最后赋值的电流
    int16_t given_current;

    /* MIT关节电机目标参数 */
    uint8_t mit_on_off; // 0：无力模式 1：下发FDCAN报文
    float target_angle;
    float target_velocity;
    float target_torq;
    float kp;
    float kd;


    /* 斜坡函数 */
    ramp_function_source_t _gimbal_s_mine_ramp_; // 用于取小资源岛矿石的斜坡函数
    ramp_fun_t _gimbal_l_mine_ramp_; // 用于取大资源岛矿石的斜坡函数

    /* 五次多项式插值 */
    Quintic_fun_t _gimbal_quintic_fun_;

}gimbal_motor_t;//云台电机结构体

//校准使用的结构体
typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

//云台控制总结构体
typedef struct
{
    /* 遥控结构体指针 */
    const dr16_control_t *gimbal_rc_ctrl;

    /* 图传链路键鼠数据指针 */
    const remote_control_t *gimbal_rc_ctrl_0x304;

    /* 图传远程控制结构体指针 */

    const vt_rc_control_t *vt_rc_control;

    /* 自定义控制器结构体指针 */
    const custom_control_t *custom_ctrl;

    /* 云台模式选择 */
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;

    //云台电机角度指针
    const float *gimbal_INT_angle_point;
    //云台电机角速度指针
    const float *gimbal_INT_gyro_point;

    /* 大关节l_joint电机 */
    gimbal_motor_t gimbal_yaw_motor; // yaw轴云台电机结构体
    gimbal_motor_t gimbal_l_joint1;  // joint1 shoulder关节
    gimbal_motor_t gimbal_l_joint2;  // joint2 elbow关节

    /* 低通滤波 */
    first_order_filter_type_t mit_motor_filter; // 小米电机 低通滤波器

    gimbal_step_cali_t gimbal_cali;
    uint16_t car_id;

} gimbal_control_t;//云台控制总结构体

extern gimbal_control_t gimbal_control;

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
extern void Gimbal_Task(void *pvParameters);

/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);


/*
  * @brief          云台校准计算，将校准记录的中值,最大 最小值返回
  * @param[out]     yaw 中值 指针
  * @param[out]     pitch 中值 指针
  * @param[out]     yaw 最大相对角度 指针
  * @param[out]     yaw 最小相对角度 指针
  * @param[out]     pitch 最大相对角度 指针
  * @param[out]     pitch 最小相对角度 指针
  * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern bool
cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, float *max_yaw, float *min_yaw, float *max_pitch,
                     float *min_pitch);

/**
  * @brief          云台校准设置，将校准的云台中值以及最小最大机械相对角度
  * @param[in]      yaw_offse:yaw 中值
  * @param[in]      pitch_offset:pitch 中值
  * @param[in]      max_yaw:max_yaw:yaw 最大相对角度
  * @param[in]      min_yaw:yaw 最小相对角度
  * @param[in]      max_yaw:pitch 最大相对角度
  * @param[in]      min_yaw:pitch 最小相对角度
  * @retval         返回空
  * @waring         这个函数使用到gimbal_control 静态变量导致函数不适用以上通用指针复用
  */
extern void
set_cali_gimbal_hook(uint16_t yaw_offset, const uint16_t pitch_offset, const float max_yaw, const float min_yaw,
                     const float max_pitch, const float min_pitch);

/**
 * @brief          检测J1是否堵转，最初用于校准电机角度数据
 * @param[in]      none
 * @retval         返回1 代表已经堵转， 返回0 代表未堵转
 */
extern bool detect_mit_j1_block(void);

/**
 * @brief          检测J2是否堵转，最初用于校准电机角度数据
 * @param[in]      none
 * @retval         返回1 代表已经堵转， 返回0 代表未堵转
 */
extern bool detect_mit_j2_block(void);

/**
 * @brief          返回大关节控制结构体gimbal_control的指针
 * @param[out]     none
 * @retval         none
 */
extern gimbal_control_t *Get_Gimbal_Control_Point(void);


/**
 * @brief          返回大关节控制结构体gimbal_control的指针
 * @param[out]     none
 * @retval         none
 */
extern gimbal_motor_mode_e * Get_Gimbal_Mode_Point(void);

#ifdef __cplusplus
}
#endif

#endif

//extern void get_relative_angle(float *relative_angle);
