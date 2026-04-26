#ifndef MACHANICAL_ARM_TASK_H
#define MACHANICAL_ARM_TASK_H

#include "main.h"
#include "Dev_FDCAN.h"
#include "Alg_PID.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"
#include "arm_math.h"
#include "App_Custom_Task.h"
#include "App_Calibrate_Task.h"
#include "App_Gimbal_Task.h"

#include "Alg_Utilities.hpp"


#define MEC_ARM_TASK_INIT_TIME  201
#define MEC_ARM_CONTROL_TIME    1
#define MEC_ARM_CONTROL_TIME_S  0.001

// s_joint0控制通道
#define S_JOINT0_CHANNEL    4       //
#define S_JOINT0_RC_SEN     -(0.02)/660
#define S_JOINT0_MOUSE_SEN   0.00002f
// s_joint1控制通道
#define S_JOINT1_CHANNEL    3       //
#define S_JOINT1_RC_SEN      -(0.006)/660
#define S_JOINT1_MOUSE_SEN   0.00001f
// s_joint2控制通道
#define S_JOINT2_CHANNEL    2      //
#define S_JOINT2_RC_SEN      -(0.006)/660
#define S_JOINT2_MOUSE_SEN   0.00001f

#define MEC_GND_MINE_LIMITATION     -L_JOINT2_RC_SEN
#define MEC_GND_STORE_LIMITATION    -YAW_RC_SEN

//状态开关通道
#define S_JOINT_MODE_CHANNEL_RIGHT     0 		    //遥控器右面挡位
#define S_JOINT_MODE_CHANNEL_LEFT      1			//遥控器左面挡位

#define S_JOINT0_ANGLE_PID_KP 150.0f
#define S_JOINT0_ANGLE_PID_KI 0.0f
#define S_JOINT0_ANGLE_PID_KD 0.0f
#define S_JOINT0_ANGLE_PID_MAX_IOUT 0.0f
#define S_JOINT0_ANGLE_PID_MAX_OUT 150.0f

#define S_JOINT0_SPEED_PID_KP 150.0f
#define S_JOINT0_SPEED_PID_KI 0
#define S_JOINT0_SPEED_PID_KD 10.0f
#define S_JOINT0_SPEED_PID_MAX_IOUT 0
#define S_JOINT0_SPEED_PID_MAX_OUT 28000.0f

#define S_JOINT1_ANGLE_PID_KP 80.0f
#define S_JOINT1_ANGLE_PID_KI 0.0f
#define S_JOINT1_ANGLE_PID_KD 0.0f
#define S_JOINT1_ANGLE_PID_MAX_IOUT 0.0f
#define S_JOINT1_ANGLE_PID_MAX_OUT 12.0f

#define S_JOINT1_SPEED_PID_KP 780.0f
#define S_JOINT1_SPEED_PID_KI 0
#define S_JOINT1_SPEED_PID_KD 15.0f
#define S_JOINT1_SPEED_PID_MAX_IOUT 0
#define S_JOINT1_SPEED_PID_MAX_OUT 9500.0f

#define S_JOINT2_ANGLE_PID_KP           S_JOINT1_ANGLE_PID_KP
#define S_JOINT2_ANGLE_PID_KI           S_JOINT1_ANGLE_PID_KI
#define S_JOINT2_ANGLE_PID_KD           S_JOINT1_ANGLE_PID_KD
#define S_JOINT2_ANGLE_PID_MAX_IOUT     S_JOINT1_ANGLE_PID_MAX_IOUT
#define S_JOINT2_ANGLE_PID_MAX_OUT      S_JOINT1_ANGLE_PID_MAX_OUT

#define S_JOINT2_SPEED_PID_KP           S_JOINT1_SPEED_PID_KP
#define S_JOINT2_SPEED_PID_KI           S_JOINT1_SPEED_PID_KI
#define S_JOINT2_SPEED_PID_KD           S_JOINT1_SPEED_PID_KD
#define S_JOINT2_SPEED_PID_MAX_IOUT     S_JOINT1_SPEED_PID_MAX_IOUT
#define S_JOINT2_SPEED_PID_MAX_OUT      S_JOINT1_SPEED_PID_MAX_OUT

/* 零位设定 若机械更改这一部分代码必须修改 后续会改成上电后自检限位设定零位 */
#define S_JOINT0_ZERO_POSITION 664
#define S_JOINT1_ZERO_POSITION 0
#define S_JOINT2_ZERO_POSITION 0

//电机码盘值最大以及中值
#define DJI_HALF_ECD_RANGE  4096
#define DJI_ECD_RANGE       8191

//电机编码值转化成角度值
#ifndef DJI_MOTOR_ECD_TO_RAD
    #define DJI_MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

/* 末端差速器校准完成标志 */
#define CALIBRATE_FINISHED 1

/* 小关节电机角度最值（差速器处理较为特殊） */
#define MEC_ARM_MOTOR_0_MAX_ANGLE        +3.10f
#define MEC_ARM_MOTOR_0_MIN_ANGLE        -3.10f
#define GIMBAL_L_JOINT1_MOTOR_MAX_ANGLE   gimbal_control.gimbal_l_joint1.MIT_Origin_Angle + 5.45f
#define GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE   gimbal_control.gimbal_l_joint1.MIT_Origin_Angle
#define GIMBAL_L_JOINT2_MOTOR_MAX_ANGLE   gimbal_control.gimbal_l_joint2.MIT_Origin_Angle -0.01f
#define GIMBAL_L_JOINT2_MOTOR_MIN_ANGLE   gimbal_control.gimbal_l_joint2.MIT_Origin_Angle -3.05f

typedef enum
{
    MEC_ARM_ZERO_FORCE_MODE = 0, // 无力模式
    MEC_ARM_ZERO_POSITION_MODE,  // 零位模式
    MEC_ARM_ANGLE_CONTROL_MODE,  // 角度控制模式
    MEC_ARM_MOTIONLESS_MODE,     // 静置模式
    MEC_ARM_CUSTOM_MODE,         // 自定义控制器模式
    MEC_ARM_CALI_MODE,           // 校准模式

    /* 自动模式 */
    MEC_AUTO_GET_GND_MINE_MODE,  // 一键取地矿模式

} mec_arm_motor_mode_e;

typedef struct
{
    /* 电机回传的原始数据 */
    const MIT_Motor_Measure_t *mec_arm_motor_measure;
    /* 滤波之后的电机数据 以下数据可能并不是都需要滤波 */
//    float Pos_ref;
//    float Vel_ref;
    float Tor_ref;
    /* 电机数据设定值 */
    float Pos_set;
    float Vel_set;
    float Tor_set;
    float Kp_set;
    float Kd_set;
    MIT_Motor_Mode mit_mode;
}MIT_Motor_t;

typedef struct
{
    /* DJI电机测量值 */
    const DJI_Motor_Measure_t *mec_arm_motor_measure;

    /* PID结构体 */
    PidTypeDef_t mec_arm_motor_angle_pid;
    PidTypeDef_t mec_arm_motor_speed_pid;

    /* 斜坡函数 */
    ramp_function_source_t _mec_s_mine_ramp_; // 用于取小资源岛矿石的斜坡函数

    /* 当前参数ref */
    float   angle;
    float   accel;
    float   speed;
    uint16_t ecd;
    uint16_t last_ecd;
    int32_t  d_angle;
    double   total_angle;

    /* 角度最值 */
    float   max_angle;
    float   min_angle;

    /* 起始位置角度：适用于末端差速器 零点=峰值 的情景 */
    float  origin_angle;

    /* 目标参数设定值set */
    float   angle_set;
    float   speed_set;
    int16_t give_current;

}DJI_Motor_t;

typedef enum
{
    DJI_MOTOR_6020 = 0,
    DJI_MOTOR_2006,
} Motor_Type_e;

typedef struct
{
    /* 遥控结构体指针 */
    const dr16_control_t *mec_arm_rc_ctrl;
    const remote_control_t *mec_arm_rc_ctrl_0x304;

    /* 图传远程控制结构体指针 */
    const vt_rc_control_t *vt_rc_control;

    /* 自定义控制器结构体指针 */
    const custom_control_t *custom_ctrl;

    /* 电机控制结构体 */
    DJI_Motor_t s_joint_motor[3]; // 大疆电机控制

    /* 机械臂行为模式结构体 */
    mec_arm_motor_mode_e mec_arm_mode;
    mec_arm_motor_mode_e last_mec_arm_mode;

    /* 大关节控制结构体指针 */
    gimbal_control_t *gimbal_control;
//    first_order_filter_type_t mit_motor_filter; // 小米电机 低通滤波器

}mec_arm_control_t;

extern mec_arm_control_t mec_arm_control;


/* Cpp port */
#ifdef __cplusplus
struct DJI_Motor_t_cpp : public DJI_Motor_t
{
    Math::linear::Ramp mec_l_mine_ramp;
};
#endif


/* Cpp port */
#ifdef __cplusplus
extern "C"{
#endif

void Mechanical_Arm_Task(void *pvParameters);

/**
 * @brief          电机校准计算，将校准记录的中值,最大 最小值返回
 * @param[in]      id    关节电机ID

 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 */
bool cmd_cali_joint_hook(void);


#ifdef __cplusplus
}
#endif

//extern const mechanical_arm_motor_t *get_yaw_motor_point(void);
//extern const mechanical_arm_motor_t *get_j4_motor_point(void);
//extern const mechanical_arm_motor_t *get_j5_motor_point(void);
//extern const mechanical_arm_motor_t *get_j6_motor_point(void);

#endif
