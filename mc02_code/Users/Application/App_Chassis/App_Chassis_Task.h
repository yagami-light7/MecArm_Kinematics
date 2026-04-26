/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             底盘控制任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef INFANTRY_ROBOT_APP_CHASSIS_TASK_H
#define INFANTRY_ROBOT_APP_CHASSIS_TASK_H

#include "main.h"

#include "Dev_FDCAN.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"

#include "Alg_PID.h"
#include "Alg_UserLib.h"

#include "App_Gimbal_Task.h"
#include "APL_RC_Hub.h"

//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1

//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0

//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 2

//选择底盘状态 开关通道号
#define CHASSIS_MODE_CHANNEL_RIGHT 0 //右侧按键开关
#define CHASSIS_MODE_CHANNEL_LEFT  1 //左侧按键开关

//遥控器前进摇杆（max 660）转化成车体前后速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f

//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000010f

//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

/*  */
#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f
#define CHASSIS_ACCEL_Z_NUM 0.0888888888f

//摇杆死区
#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

//底盘与云台之间的距离
#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002f

//底盘任务控制频率
#define CHASSIS_CONTROL_FREQUENCE 500.0f

//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//底盘小陀螺按键
#define GYROSCOPE_ON_KEY    KEY_PRESSED_OFFSET_V
#define GYROSCOPE_OFF_KEY   KEY_PRESSED_OFFSET_B

//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY   KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY    KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY    KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY   KEY_PRESSED_OFFSET_D

// M3508电机原始转速数据转化成Rpm的比例  3591/187
//#define M3508_MOTOR_RPM_TO_VECTOR           0.052074631021999f
#define M3508_MOTOR_RPM_TO_VECTOR           0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN     M3508_MOTOR_RPM_TO_VECTOR

//单个底盘电机最大速度
#define MAX_WHEEL_SPEED 15.0f

/* 无超电 X方向最大速度 */
#define NORMAL_MAX_CHASSIS_SPEED_X 3.5f

/* 无超电 Y方向最大速度 */
#define NORMAL_MAX_CHASSIS_SPEED_Y 3.5f

//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.0f

//底盘小陀螺减缓速度
#define CHASSIS_SPIN_LOW_SPEED 1.5f

//底盘小陀螺基本速度
#define CHASSIS_SPIN_MAIN_SPEED 10

//底盘小陀螺减去x,y方向的速度比例系数
#define CHASSIS_SPIN_LOW_SEN 0.6

/* 具体参数 根据机械设计修改 单位：mm */
#define WHEELBASE       187     // 外径与底盘中心距离
#define WHEELTRACK      163     // 内径与底盘中心距离
#define GIMBAL_OFFSET   6       // 云台偏移量，暂不知如何测量
#define PERIMETER       152.4   // 全向轮轮直径
#define CHASSIS_DECELE_RATIO 19 // M3508电机减速比

#define McKnum_Wheel_X 163.24 	//麦轮投影点横坐标
#define McKnum_Wheel_Y 186.965 	//麦轮投影点纵坐标

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f

//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//全向轮底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP_Omnidirectional 15000.0f
#define M3505_MOTOR_SPEED_PID_KI_Omnidirectional 20.0f
#define M3505_MOTOR_SPEED_PID_KD_Omnidirectional 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT_Omnidirectional MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT_Omnidirectional 2000.0f

//全向轮底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP_Omnidirectional 15.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI_Omnidirectional 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD_Omnidirectional 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT_Omnidirectional 18.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT_Omnidirectional 0.2f

//麦克纳姆轮底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP_McNamm 11500.0f
#define M3505_MOTOR_SPEED_PID_KI_McNamm 0.0f
#define M3505_MOTOR_SPEED_PID_KD_McNamm 45.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT_McNamm MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT_McNamm 0.0f

//麦克纳姆轮底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP_McNamm 10.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI_McNamm 0
#define CHASSIS_FOLLOW_GIMBAL_PID_KD_McNamm 0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT_McNamm 200.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT_McNamm 0

//底盘摇杆x方向PID
#define CHASSIS_DT7_X_PID_KP 0.5f
#define CHASSIS_DT7_X_PID_KI 0.0f
#define CHASSIS_DT7_X_PID_KD 0.0f
#define CHASSIS_DT7_X_PID_MAX_OUT 10.0f
#define CHASSIS_DT7_X_PID_MAX_IOUT 2.0f

//底盘遥感y方向PID
#define CHASSIS_DT7_Y_PID_KP 0.5f
#define CHASSIS_DT7_Y_PID_KI 0.0f
#define CHASSIS_DT7_Y_PID_KD 0.0f
#define CHASSIS_DT7_Y_PID_MAX_OUT 10.0f
#define CHASSIS_DT7_Y_PID_MAX_IOUT 2.0f

/* 底盘状态 */
typedef enum
{
    CHASSIS_ZERO_SPEED,                   //底盘保持不动,速度为0
    CHASSIS_NO_MOVE,                      //底盘保持不动，yaw不动
    CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW,  //工程底盘角度控制底盘
    CHASSIS_ENGINEER_NO_FOLLOW_YAW,       //工程底盘只能平移

//    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   // 底盘会跟随云台相对角度
//    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,  // 底盘有底盘角度控制闭环
//    CHASSIS_VECTOR_NO_FOLLOW_YAW,       // 底盘有旋转速度控制
//    CHASSIS_VECTOR_RAW,                 // 底盘电流开环 无力模式
//    CHASSIS_GYROSCOPE,                  // 底盘小陀螺模式
}Chassis_Mode_e;

/* 底盘电机数据 */
typedef struct
{
    const DJI_Motor_Measure_t *Chassis_Motor_Measure;
    float Accel;
    float Speed;     // Rpm
    float Speed_Set; // Rpm
    int16_t Give_Current;
}Chassis_Motor_t;

/* 底盘控制结构体 */
typedef struct
{
    const dr16_control_t *chassis_RC;                 //底盘使用的遥控器指针
    const remote_control_t *chassis_RC_0x304;   //图传链路键鼠数据指针
    const vt_rc_control_t *vt_rc_control;       // 图传远程控制

    const gimbal_motor_t *chassis_yaw_motor;    //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角.
    const gimbal_motor_t *chassis_pitch_motor;  //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
    const float *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
    Chassis_Mode_e chassis_mode;                //底盘控制状态机
    Chassis_Mode_e last_chassis_mode;           //底盘上次控制状态机
    Chassis_Motor_t motor_chassis[4];           //底盘电机数据
    PidTypeDef_t motor_speed_pid[4];            //底盘电机速度pid
    PidTypeDef_t chassis_angle_pid;             //底盘跟随角度pid
//    PidTypeDef_t chassis_DT7_x_pid;             //底盘摇杆x方向pid
//    PidTypeDef_t chassis_DT7_y_pid;             //底盘摇杆y方向pid
    PidTypeDef_t buffer_pid;                    //缓冲pid

    /* 低通滤波器：DT7发送的设定值 */
    first_order_filter_type_t chassis_cmd_slow_set_vx;  //使用一阶低通滤波减缓机器人速度设定值
    first_order_filter_type_t chassis_cmd_slow_set_vy;  //使用一阶低通滤波减缓机器人速度设定值
    first_order_filter_type_t chassis_cmd_slow_set_wz;  //使用一阶低通滤波减缓机器人速度设定值

    ramp_function_source_t vx_ramp;
    ramp_function_source_t vy_ramp;

    float vx;                          //底盘速度 前进方向 前为正，单位 m/s
    float vy;                          //底盘速度 左右方向 左为正  单位 m/s
    float wz;                          //底盘旋转角速度，逆时针为正 单位 rad/s
    float vx_set;                      //底盘设定速度 前进方向 前为正，单位 m/s
    float vy_set;                      //底盘设定速度 左右方向 左为正，单位 m/s
    float wz_set;                      //底盘设定旋转角速度，逆时针为正 单位 rad/s
    float chassis_relative_angle;      //底盘与云台的相对角度，单位 rad
    float chassis_relative_angle_set;  //设置相对云台控制角度
    float chassis_yaw_set;

    float vx_max_speed;  //前进方向最大速度 单位m/s
    float vx_min_speed;  //后退方向最大速度 单位m/s
    float vy_max_speed;  //左方向最大速度 单位m/s
    float vy_min_speed;  //右方向最大速度 单位m/s
    float chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
    float chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
    float chassis_roll;  //陀螺仪和云台电机叠加的roll角度

    first_order_filter_type_t chassis_x_filter; //底盘x方向滤波器
    first_order_filter_type_t chassis_y_filter; //底盘y方向滤波器

    float chassis_spin_change_sen; //底盘旋转速度

    float vaEstimateKF_F[4];
    float vaEstimateKF_P[4];
    float vaEstimateKF_Q[4];
    float vaEstimateKF_R[4];
    float vaEstimateKF_H[4];

    // 机体坐标加速度
    float MotionAccel_b[3];
    // 绝对系加速度
    float MotionAccel_n[3];

//    float wr, wl;
//    float vrb, vlb;
//    float aver_v;

}Chassis_Move_t;


#ifdef __cplusplus
extern "C" {
#endif

/**
  * @brief          底盘任务，间隔 CHASSIS_CONTROL_TIME_MS 2ms
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void Chassis_Task(void *pvParameters);

/**
  * @brief          根据遥控器通道值，计算纵向和横移速度
  *
  * @param[out]     vx_set: 纵向速度指针
  * @param[out]     vy_set: 横向速度指针
  * @param[out]     chassis_move_rc_to_vector: "chassis_move" 变量指针
  * @retval         none
  */
extern void Chassis_RC_To_Control_Vector(float *vx_set, float *vy_set, Chassis_Move_t *chassis_move_rc_to_vector);

#ifdef __cplusplus
}
#endif

#endif
