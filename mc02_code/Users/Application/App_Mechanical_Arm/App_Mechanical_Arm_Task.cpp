#include <stdlib.h>
#include "App_Mechanical_Arm_Task.h"
#include "App_Mechanical_Arm_Behaviour.h"
#include "App_Detect_Task.h"
#include "App_Gimbal_Task.h"
#include "Alg_Utilities.hpp"
#include "UC_Referee.h"

#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }
		
#if INCLUDE_uxTaskGetStackHighWaterMark
    uint32_t mec_arm_high_water;
#endif

/* 总控制结构体创建 */
mec_arm_control_t mec_arm_control;


/* 函数声明 */
static void mec_arm_init(mec_arm_control_t *init);
static void mec_arm_set_mode(mec_arm_control_t *set_mode);
static void mec_arm_feedback_update(mec_arm_control_t *update);
static void mec_arm_mode_change_control_transit(mec_arm_control_t *change);
static void mec_arm_set_control(mec_arm_control_t *set_control);
static void mec_arm_control_loop(mec_arm_control_t *loop);

static void dji_motor_ecd_to_angle_change(DJI_Motor_t *angle_change, uint16_t offset_ecd, Motor_Type_e type);
static void mec_arm_zero_postion_control(DJI_Motor_t *zero_pos);
static void mec_arm_angle_control(DJI_Motor_t *angle_control, Motor_Type_e type);
static void mec_arm_motionless_control(DJI_Motor_t *motionless_control, Motor_Type_e type);
static void mec_arm_custom_control(DJI_Motor_t *motionless_control, Motor_Type_e type);
static void detect_2006_current(DJI_Motor_t *s_joint1, DJI_Motor_t *s_joint2);
static void d_gear_angle_slove(float *j1_angle, float *j2_angle, float add_pitch, float add_yaw);
static void d_gear_angle_gnd_slove(float *j1_angle, float *j2_angle, float pitch);

/* PID参数数组 */
static const float s_joint0_angle_pid[3] = {S_JOINT0_ANGLE_PID_KP, S_JOINT0_ANGLE_PID_KI, S_JOINT0_ANGLE_PID_KD};
static const float s_joint1_angle_pid[3] = {S_JOINT1_ANGLE_PID_KP, S_JOINT1_ANGLE_PID_KI, S_JOINT1_ANGLE_PID_KD};
static const float s_joint2_angle_pid[3] = {S_JOINT2_ANGLE_PID_KP, S_JOINT2_ANGLE_PID_KI, S_JOINT2_ANGLE_PID_KD};

static const float s_joint0_speed_pid[3] = {S_JOINT0_SPEED_PID_KP, S_JOINT0_SPEED_PID_KI, S_JOINT0_SPEED_PID_KD};
static const float s_joint1_speed_pid[3] = {S_JOINT1_SPEED_PID_KP, S_JOINT1_SPEED_PID_KI, S_JOINT1_SPEED_PID_KD};
static const float s_joint2_speed_pid[3] = {S_JOINT2_SPEED_PID_KP, S_JOINT2_SPEED_PID_KI, S_JOINT2_SPEED_PID_KD};

/* 末端差速器角度值 */
static float dif_pitch_angle = 0;
static float dif_yaw_angle   = 0;
//static float s_JOINT0_ANGLE_PID_KP = 3.2f;
//static float s_JOINT0_ANGLE_PID_KI = 0.0f;
//static float s_JOINT0_ANGLE_PID_KD = 0.0f;
//static float s_JOINT0_ANGLE_PID_MAX_IOUT = 0.0f;
//static float s_JOINT0_ANGLE_PID_MAX_OUT = 8.5f;
//
//static float s_JOINT0_SPEED_PID_KP = 35.0f;
//static float s_JOINT0_SPEED_PID_KI = 15.0f;
//static float s_JOINT0_SPEED_PID_KD = 1.0f;
//static float s_JOINT0_SPEED_PID_MAX_IOUT = 9000.0f;
//static float s_JOINT0_SPEED_PID_MAX_OUT = 28000.0f;

//static float s_JOINT1_ANGLE_PID_KP = 80.0f;
//static float s_JOINT1_ANGLE_PID_KI = 0.0f;
//static float s_JOINT1_ANGLE_PID_KD = 0.0f;
//static float s_JOINT1_ANGLE_PID_MAX_IOUT = 0.0f;
//static float s_JOINT1_ANGLE_PID_MAX_OUT = 400.0f;

static float s_JOINT1_SPEED_PID_KP = 60.0f;
static float s_JOINT1_SPEED_PID_KI = 15.0f;
static float s_JOINT1_SPEED_PID_KD = 0.0f;
static float s_JOINT1_SPEED_PID_MAX_IOUT = 6000.0f;
static float s_JOINT1_SPEED_PID_MAX_OUT = 10000.0f;

////static float s_JOINT2_ANGLE_PID_KP = 15.0f;
////static float s_JOINT2_ANGLE_PID_KI = 0.0f;
////static float s_JOINT2_ANGLE_PID_KD = 0.0f;
////static float s_JOINT2_ANGLE_PID_MAX_IOUT = 0.0f;
////static float s_JOINT2_ANGLE_PID_MAX_OUT = 300.0f;
//
//static float s_JOINT2_SPEED_PID_KP = 6.0f;
//static float s_JOINT2_SPEED_PID_KI = 18.0;
//static float s_JOINT2_SPEED_PID_KD = 1.0f;
//static float s_JOINT2_SPEED_PID_MAX_IOUT = 6000.0f;
//static float s_JOINT2_SPEED_PID_MAX_OUT = 10000.0f;

/**
 * @brief          机械臂末端任务，间隔 MEC_ARM_CONTROL_TIME 1ms
 *                 机械臂大臂在App_Gimbal_Task.c 中
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void Mechanical_Arm_Task(void *pvParameters)
{
    /* 空闲以待初始化 */
    vTaskDelay(MEC_ARM_TASK_INIT_TIME);

    /* 末端关节初始化 */
    mec_arm_init(&mec_arm_control);


    /* 判断电机是否都上线 */
    while (toe_is_error(S_JOINT_MOTOR3_TOE) || toe_is_error(S_JOINT_MOTOR4_TOE) || toe_is_error(S_JOINT_MOTOR5_TOE))
    {
        vTaskDelay(MEC_ARM_CONTROL_TIME);
        mec_arm_feedback_update(&mec_arm_control);
    }

    /* 判断上电校准是否完成 */
    while (joint_cali.calibrate_finished_flag != CALIBRATE_FINISHED)
    {
        vTaskDelay(MEC_ARM_CONTROL_TIME);

        /* 不断回传数据刷新电机数据结构体 */
        mec_arm_feedback_update(&mec_arm_control);

        /* 直接执行PID计算， 参数设置已经在校准函数中设置完毕 */
        mec_arm_control_loop(&mec_arm_control);

        /* 下发PID计算 */
        FDCAN_cmd_Small_Joint(0,
                              mec_arm_control.s_joint_motor[1].give_current,
                              mec_arm_control.s_joint_motor[2].give_current);
    }

    while (1)
    {
        /* 设置末端关节控制模式 */
        mec_arm_set_mode(&mec_arm_control);

        /* 控制模式切换 数据过渡 */
        mec_arm_mode_change_control_transit(&mec_arm_control);

        /* 末端关节数据反馈 */
        mec_arm_feedback_update(&mec_arm_control);

        /* 设置末端关节控制量 */
        mec_arm_set_control(&mec_arm_control);

        /* 末端关节控制PID计算 */
        mec_arm_control_loop(&mec_arm_control);

        /* FDCAN下发电流 */
        // 确保至少一个电机在线， 这样CAN控制包可以被接收到
        if (!(toe_is_error(S_JOINT_MOTOR3_TOE) && toe_is_error(S_JOINT_MOTOR4_TOE) && toe_is_error(S_JOINT_MOTOR5_TOE)))
        {
            // 若电管断电
//            if (robot_state.mains_power_gimbal_output == 0 || (toe_is_error(VT_TOE) && toe_is_error(DBUS_TOE)))
//            {
//                FDCAN_cmd_Small_Joint(0,0,0);
//            }
            if ((toe_is_error(VT_TOE) && toe_is_error(DBUS_TOE)))
            {
                FDCAN_cmd_Small_Joint(0,0,0);
            }
            else
            {
                FDCAN_cmd_Small_Joint(mec_arm_control.s_joint_motor[0].give_current,
                                     mec_arm_control.s_joint_motor[1].give_current,
                                     mec_arm_control.s_joint_motor[2].give_current);
            }
        }

        vTaskDelay(MEC_ARM_CONTROL_TIME);
			
#if INCLUDE_uxTaskGetStackHighWaterMark
        mec_arm_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


static void mec_arm_init(mec_arm_control_t *init)
{
    /* 获取遥控器数据指针 */
    init->mec_arm_rc_ctrl = &RC_Hub.dr16_control;

    /* 获取图传链路键鼠数据指针 */
    init->mec_arm_rc_ctrl_0x304 = &RC_Hub.remote_control;

    /* VT远程控制指针 */
    init->vt_rc_control = &RC_Hub.vt_rc_control;

    /* 获取自定义控制器数据指针 */
    init->custom_ctrl = &RC_Hub.custom_control;

    /* 获取末端关节电机数据指针 */
    for (int i = 0; i < 3; i++)
    {
        init->s_joint_motor[i].mec_arm_motor_measure = Get_Small_Joint_Motor_Measure_Point(i);
    }

    /* 获取大关节控制结构体指针 */
    init->gimbal_control = Get_Gimbal_Control_Point();

    /* 初始化电机模式 */
    init->mec_arm_mode = init->last_mec_arm_mode = MEC_ARM_ZERO_FORCE_MODE;

    /* 设置电机角度最值 */
    init->s_joint_motor[0].max_angle =  MEC_ARM_MOTOR_0_MAX_ANGLE;
    init->s_joint_motor[0].min_angle =  MEC_ARM_MOTOR_0_MIN_ANGLE;

    /* 初始化斜坡函数，用于一键取矿/存矿 */    //差速器电机角度处理较为特殊，此处暂设为0
    ramp_init(&init->s_joint_motor[0]._mec_s_mine_ramp_, MEC_ARM_CONTROL_TIME_S, MEC_ARM_MOTOR_0_MAX_ANGLE,MEC_ARM_MOTOR_0_MIN_ANGLE);
    ramp_init(&init->s_joint_motor[1]._mec_s_mine_ramp_, MEC_ARM_CONTROL_TIME_S, 0, 0);
    ramp_init(&init->s_joint_motor[2]._mec_s_mine_ramp_, MEC_ARM_CONTROL_TIME_S, 0, 0);


    /* 初始化电机PID */
    PID_init(&init->s_joint_motor[0].mec_arm_motor_angle_pid, PID_POSITION, s_joint0_angle_pid, S_JOINT0_ANGLE_PID_MAX_OUT, S_JOINT0_ANGLE_PID_MAX_IOUT);
    PID_init(&init->s_joint_motor[1].mec_arm_motor_angle_pid, PID_POSITION, s_joint1_angle_pid, S_JOINT1_ANGLE_PID_MAX_OUT, S_JOINT1_ANGLE_PID_MAX_IOUT);
    PID_init(&init->s_joint_motor[2].mec_arm_motor_angle_pid, PID_POSITION, s_joint2_angle_pid, S_JOINT2_ANGLE_PID_MAX_OUT, S_JOINT2_ANGLE_PID_MAX_IOUT);

    PID_init(&init->s_joint_motor[0].mec_arm_motor_speed_pid, PID_POSITION, s_joint0_speed_pid, S_JOINT0_SPEED_PID_MAX_OUT, S_JOINT0_SPEED_PID_MAX_IOUT);
    PID_init(&init->s_joint_motor[1].mec_arm_motor_speed_pid, PID_POSITION, s_joint1_speed_pid, S_JOINT1_SPEED_PID_MAX_OUT, S_JOINT1_SPEED_PID_MAX_IOUT);
    PID_init(&init->s_joint_motor[2].mec_arm_motor_speed_pid, PID_POSITION, s_joint2_speed_pid, S_JOINT2_SPEED_PID_MAX_OUT, S_JOINT2_SPEED_PID_MAX_IOUT);

    //清除所有PID
    for (int i = 0; i < 3; i++)
    {
        PID_clear(&init->s_joint_motor[i].mec_arm_motor_angle_pid);
        PID_clear(&init->s_joint_motor[i].mec_arm_motor_speed_pid);
    }

    mec_arm_feedback_update(init);
//
//    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
//    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
//    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
//    init->gimbal_yaw_motor.motor_enconde_speed_set = init->gimbal_yaw_motor.motor_enconde_speed;
//    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
//    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
//    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
//    init->gimbal_pitch_motor.motor_enconde_speed_set = init->gimbal_pitch_motor.motor_enconde_speed;

}

static void mec_arm_set_mode(mec_arm_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }

    mec_arm_behaviour_mode_set(set_mode);
}

/**
 * @brief          末端关节测量数据更新，包括电机速度，欧拉角度
 *                 这些数据是主要是将FDCAN回传的数据进行单位的转换之后拷贝给上一层结构体
 * @param[out]     change:"mec_arm_control_t"变量指针.
 * @retval         none
 */
static void mec_arm_feedback_update(mec_arm_control_t *update)
{
    if (update == NULL)
    {
        return;
    }

    /* 角度数据更新 单位：rad */
    // 6020电机角度
    dji_motor_ecd_to_angle_change(&update->s_joint_motor[0], S_JOINT0_ZERO_POSITION, DJI_MOTOR_6020);

    // 2060电机角度
    dji_motor_ecd_to_angle_change(&update->s_joint_motor[1], S_JOINT1_ZERO_POSITION, DJI_MOTOR_2006);
    dji_motor_ecd_to_angle_change(&update->s_joint_motor[2], S_JOINT2_ZERO_POSITION, DJI_MOTOR_2006);

    /* 速度数据更新 单位：rpm */
    update->s_joint_motor[0].speed = update->s_joint_motor[0].mec_arm_motor_measure->speed_rpm;
    update->s_joint_motor[1].speed = update->s_joint_motor[1].mec_arm_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;
    update->s_joint_motor[2].speed = update->s_joint_motor[2].mec_arm_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED;

}

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 *                 这里的DJI电机是2006和6020
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @param[in]      type:进行类型转化电机种类
 * @retval         相对角度，单位rad
 */
static void dji_motor_ecd_to_angle_change(DJI_Motor_t *angle_change, uint16_t offset_ecd, Motor_Type_e type)
{
    int32_t relative_ecd = angle_change->mec_arm_motor_measure->ecd - offset_ecd;

    /* 6020电机做循环限幅处理 */
    if (type == DJI_MOTOR_6020)
    {
        if (relative_ecd > DJI_HALF_ECD_RANGE)
        {
            relative_ecd -= DJI_ECD_RANGE;
        }
        else if (relative_ecd < -DJI_HALF_ECD_RANGE)
        {
            relative_ecd += DJI_ECD_RANGE;
        }

        angle_change->angle = relative_ecd * DJI_MOTOR_ECD_TO_RAD;
    }

    /* 2006电机需要计算总圈数之后换算 */
    else if( type == DJI_MOTOR_2006 )
    {
        angle_change->ecd = angle_change->mec_arm_motor_measure->ecd;

        if (angle_change->ecd - angle_change->last_ecd > DJI_HALF_ECD_RANGE)
        {
            angle_change->d_angle = -angle_change->last_ecd - (DJI_ECD_RANGE - angle_change->ecd);
        }
        else if (angle_change->ecd - angle_change->last_ecd < -DJI_HALF_ECD_RANGE)
        {
            angle_change->d_angle = angle_change->ecd + (DJI_ECD_RANGE - angle_change->last_ecd);
        }
        else
        {
            angle_change->d_angle = angle_change->ecd - angle_change->last_ecd;
        }
        // 将角度增量加入计数器
        angle_change->total_angle += angle_change->d_angle * 1.0;

        // 记录角度
        angle_change->angle = angle_change->total_angle;

        angle_change->last_ecd = angle_change->ecd;

        /* 2*PI/8192/36 */
        angle_change->angle = angle_change->total_angle * (2.1305288720633905968306772076277e-5);
    }

}

static void mec_arm_mode_change_control_transit(mec_arm_control_t *change)
{
    if (change == NULL)
    {
        return;
    }

    /* 模式发生更改 */
    if(change->last_mec_arm_mode != change->mec_arm_mode)
    {
        // 末端关节电机状态机切换保存数据
        if (change->last_mec_arm_mode != MEC_ARM_ZERO_FORCE_MODE && change->mec_arm_mode == MEC_ARM_ZERO_FORCE_MODE)
        {
            for (int i = 0; i < 3; i++)
            {
                change->s_joint_motor[i].give_current = change->s_joint_motor[i].mec_arm_motor_measure->given_current;
            }

        }
        else if (change->last_mec_arm_mode != MEC_ARM_ZERO_POSITION_MODE && change->mec_arm_mode == MEC_ARM_ZERO_POSITION_MODE)
        {
            for (int i = 0; i < 3; i++)
            {
                change->s_joint_motor[i].angle_set = 0;
            }

        }
        else if (change->last_mec_arm_mode != MEC_ARM_ANGLE_CONTROL_MODE && change->mec_arm_mode == MEC_ARM_ANGLE_CONTROL_MODE)
        {
            for (int i = 0; i < 3; i++)
            {
                change->s_joint_motor[i].angle_set = change->s_joint_motor[i].angle;
            }
        }
        else if (change->last_mec_arm_mode != MEC_ARM_MOTIONLESS_MODE && change->mec_arm_mode == MEC_ARM_MOTIONLESS_MODE)
        {

        }
        else if (change->last_mec_arm_mode != MEC_ARM_CUSTOM_MODE && change->mec_arm_mode == MEC_ARM_CUSTOM_MODE)
        {

        }
    }

    change->last_mec_arm_mode = change->mec_arm_mode;
}


/* 一键地矿 */
extern float _gnd_mine_emulation_;     // 俯仰滚轮累计值，用于控制一键地矿
extern float _store_emulation_;        // 累计到达一定值，表明存在左侧/右侧吸盘
extern float _take_emulation_;         // 累计到达一定值，表明进行放矿/拿矿

/* 一键大资源岛矿 */
extern float _gimbal_l_mine_emulation_;       // 累计到达一定值，表明选择资源岛的三个通道

static void mec_arm_set_control(mec_arm_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    // 遥控器控制量赋值或者视觉控制量赋值
    float add_s_j0_angle = 0.0f;
    float add_s_j1_angle = 0.0f;
    float add_s_j2_angle = 0.0f;

    /* 获取增量 */
    mec_arm_behaviour_control_set(&add_s_j0_angle, &add_s_j1_angle, &add_s_j2_angle ,set_control);

    /*  对增量进行处理，转化为最后下发给MIT电机的目标参数
     *  拷贝云台目标设定值,kp kd等其它参数已经在gimbal_mode_change_control_transit中设置了
     *  */
    // 电机模式控制
    if(set_control->mec_arm_mode == MEC_ARM_ZERO_FORCE_MODE)
    {
        /* gimbal_mode_change_control_transit中设置了输出为0*/
        for (int i = 0; i < 3; i++)
        {
            set_control->s_joint_motor[i].give_current = 0;
        }
    }
    else if(set_control->mec_arm_mode == MEC_ARM_ZERO_POSITION_MODE)
    {
        static float pitch_emulation = 0.0f;

        pitch_emulation += add_s_j1_angle * 1.2f;
        pitch_emulation =  float_constrain(pitch_emulation, 0.3 * PI, PI);

        set_control->s_joint_motor[0].angle_set = 0;
        set_control->s_joint_motor[1].angle_set = set_control->s_joint_motor[1].origin_angle + dif_yaw_angle -  PI + pitch_emulation;
        set_control->s_joint_motor[2].angle_set = set_control->s_joint_motor[2].origin_angle + dif_yaw_angle + PI - pitch_emulation;
    }
    else if(set_control->mec_arm_mode == MEC_ARM_ANGLE_CONTROL_MODE)
    {
        if(add_s_j1_angle != 0 || add_s_j2_angle != 0)
        {
            // 放慢j0速度
            set_control->s_joint_motor[0].angle_set += add_s_j0_angle * 0.2;
        }
        else
        {
            set_control->s_joint_motor[0].angle_set += add_s_j0_angle;
        }

        d_gear_angle_slove(&set_control->s_joint_motor[1].angle_set, &set_control->s_joint_motor[2].angle_set, add_s_j1_angle, add_s_j2_angle);
//        set_control->s_joint_motor[1].speed_set = +add_s_j1_angle + add_s_j2_angle;
//        set_control->s_joint_motor[2].speed_set = -add_s_j1_angle + add_s_j2_angle;
        /* 对末端电机进行角度的限幅
         * 6020 机械角度固定，如果机械位置修改，应该修改代码，否则疯车
         * 2006 角度的最值上电后会自检记录*/
        set_control->s_joint_motor[0].angle_set = float_constrain(set_control->s_joint_motor[0].angle_set, set_control->s_joint_motor[0].min_angle, set_control->s_joint_motor[0].max_angle);

        /* 对2006组成的手腕进行电流检测，产生尖峰则不再继续旋转 */
//        detect_2006_current(&set_control->s_joint_motor[1], &set_control->s_joint_motor[2]);

    }
    else if(set_control->mec_arm_mode == MEC_ARM_MOTIONLESS_MODE)
    {
        /* 小关节目标角度不做更改 */
//        set_control->gimbal_yaw_motor.target_angle = set_control->gimbal_yaw_motor.target_angle;
//
//        set_control->gimbal_l_joint1.target_angle  = set_control->gimbal_yaw_motor.target_angle;
//
//        set_control->gimbal_l_joint2.target_angle  = set_control->gimbal_yaw_motor.target_angle;
    }
    else if(set_control->mec_arm_mode == MEC_ARM_CUSTOM_MODE)
    {
        set_control->s_joint_motor[0].angle_set = set_control->custom_ctrl->custom_angle_set[3];
        d_gear_angle_slove(&set_control->s_joint_motor[1].angle_set, &set_control->s_joint_motor[2].angle_set, add_s_j1_angle, add_s_j2_angle);

        /* 对末端电机进行角度的限幅
         * 6020 机械角度固定，如果机械位置修改，应该修改代码，否则疯车
         * 2006 角度的最值上电后会自检记录*/
        set_control->s_joint_motor[0].angle_set = float_constrain(set_control->s_joint_motor[0].angle_set, set_control->s_joint_motor[0].min_angle, set_control->s_joint_motor[0].max_angle);

//        /* 对2006组成的手腕进行电流检测，产生尖峰则不再继续旋转 */
//        detect_2006_current(&set_control->s_joint_motor[1], &set_control->s_joint_motor[2]);
    }
    else if (set_control->mec_arm_mode == MEC_AUTO_GET_GND_MINE_MODE)
    {
        /* 控制值限幅 */
        _gnd_mine_emulation_ = float_constrain(_gnd_mine_emulation_, 0, GIMBAL_GND_MINE_LIMITATION);
        _store_emulation_    = float_constrain(_store_emulation_,    -GIMBAL_GND_STORE_LIMITATION,GIMBAL_GND_STORE_LIMITATION);
        _take_emulation_     = float_constrain(_take_emulation_,     0,GIMBAL_GND_TAKE_LIMITATION);

        /* 调整斜坡函数参数 平滑到达目标位置 */
        mec_arm_control.s_joint_motor[0]._mec_s_mine_ramp_.max_value = +0.15f;
        mec_arm_control.s_joint_motor[1]._mec_s_mine_ramp_.min_value = -2.40f;
        mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.max_value = +2.60f;
        mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.min_value = +2.55f;

//        ramp_calc(&mec_arm_control.s_joint_motor[0]._mec_s_mine_ramp_, +0.001);
//        ramp_calc(&mec_arm_control.s_joint_motor[1]._mec_s_mine_ramp_, -0.001);

        /* 未到达 进行取矿 */
        if(_gnd_mine_emulation_ <  GIMBAL_GND_MINE_LIMITATION * 0.9f)
        {
            ramp_calc(&mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_, +3.0);
            add_s_j1_angle = mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.out;

        }
        /* 到达 进行存矿*/
        else
        {
            /* 手上拿一个 */
            if(fabsf(_store_emulation_) < GIMBAL_GND_STORE_LIMITATION * 0.9f)
            {
                if(mec_arm_control.gimbal_control->gimbal_l_joint1.gimbal_motor_measure->pos > GIMBAL_L_JOINT1_MOTOR_MIN_ANGLE + 2.50f)
                {
                    /* 地矿垂直上升阶段 */
                    mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.max_value = +4.00f;
                    ramp_calc(&mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_, 0.8f);
                }
                else
                {
                    /* 地矿已脱离资源岛阶段 */
                    ramp_calc(&mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_, -3.0);
                }
                add_s_j1_angle = mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.out;
            }
            /* 存放在左侧吸盘 */
            else if(_store_emulation_ > GIMBAL_GND_STORE_LIMITATION * 0.9)
            {
                mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.max_value = 3.15f;
                ramp_calc(&mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_, +4.0);
                add_s_j1_angle = mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.out;
            }
            /* 存放在右侧吸盘 */
            else if(_store_emulation_ < -GIMBAL_GND_STORE_LIMITATION * 0.9)
            {
                mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.max_value = 3.15f;
                ramp_calc(&mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_, +4.0);
                add_s_j1_angle = mec_arm_control.s_joint_motor[2]._mec_s_mine_ramp_.out;
            }
        }
        /* 目标角度赋值 */
        set_control->s_joint_motor[0].angle_set = mec_arm_control.s_joint_motor[0]._mec_s_mine_ramp_.out;
        d_gear_angle_gnd_slove(&set_control->s_joint_motor[1].angle_set, &set_control->s_joint_motor[2].angle_set, add_s_j1_angle);
    }
}

static void mec_arm_control_loop(mec_arm_control_t *loop)
{
    if (loop == NULL)
    {
        return;
    }

    /* 根据不同的模式进行PID计算 */
    if (loop->mec_arm_mode == MEC_ARM_ZERO_FORCE_MODE)
    {
        for (int i = 0; i < 3; i++)
        {
            loop->s_joint_motor[i].give_current = 0;
        }

    }
    else if (loop->mec_arm_mode == MEC_ARM_ZERO_POSITION_MODE)
    {
        for (int i = 0; i < 3; i++)
        {
            mec_arm_zero_postion_control(&loop->s_joint_motor[i]);
        }
    }
    else if (loop->mec_arm_mode == MEC_ARM_ANGLE_CONTROL_MODE)
    {
        mec_arm_angle_control(&loop->s_joint_motor[0], DJI_MOTOR_6020);
        mec_arm_angle_control(&loop->s_joint_motor[1], DJI_MOTOR_2006);
        mec_arm_angle_control(&loop->s_joint_motor[2], DJI_MOTOR_2006);
    }
    else if (loop->mec_arm_mode == MEC_ARM_MOTIONLESS_MODE)
    {
        mec_arm_motionless_control(&loop->s_joint_motor[0], DJI_MOTOR_6020);
        mec_arm_motionless_control(&loop->s_joint_motor[1], DJI_MOTOR_2006);
        mec_arm_motionless_control(&loop->s_joint_motor[2], DJI_MOTOR_2006);

    }
    else if (loop->mec_arm_mode == MEC_ARM_CUSTOM_MODE)
    {
        mec_arm_custom_control(&loop->s_joint_motor[0], DJI_MOTOR_6020);
        mec_arm_custom_control(&loop->s_joint_motor[1], DJI_MOTOR_2006);
        mec_arm_custom_control(&loop->s_joint_motor[2], DJI_MOTOR_2006);
   }
    else if (loop->mec_arm_mode == MEC_ARM_CALI_MODE)
    {

        if(joint_cali.calibrate_finished_flag != CALIBRATE_FINISHED)
        {
            // 寻找零位
            mec_arm_control.s_joint_motor[1].give_current = PID_Calc(&mec_arm_control.s_joint_motor[1].mec_arm_motor_speed_pid, mec_arm_control.s_joint_motor[1].speed, mec_arm_control.s_joint_motor[1].speed_set);
            mec_arm_control.s_joint_motor[2].give_current = PID_Calc(&mec_arm_control.s_joint_motor[2].mec_arm_motor_speed_pid, mec_arm_control.s_joint_motor[2].speed, mec_arm_control.s_joint_motor[2].speed_set);
        }
        else
        {
            mec_arm_control.s_joint_motor[1].give_current = 0;
            mec_arm_control.s_joint_motor[2].give_current = 0;
        }

    }
    else if (loop->mec_arm_mode == MEC_AUTO_GET_GND_MINE_MODE)
    {
        mec_arm_angle_control(&loop->s_joint_motor[0], DJI_MOTOR_6020);
        mec_arm_angle_control(&loop->s_joint_motor[1], DJI_MOTOR_2006);
        mec_arm_angle_control(&loop->s_joint_motor[2], DJI_MOTOR_2006);
    }
}

/**
 * @brief          末端控制模式:MEC_ARM_ZERO_POSITION_MODE，电机编码归零
 * @param[out]     DJI_Motor_t:末端电机*3
 * @retval         none
 */
static void mec_arm_zero_postion_control(DJI_Motor_t *zero_pos)
{
    if(zero_pos == NULL)
    {
        return;
    }

    /* 角度环串速度环 */
    zero_pos->speed_set    = PID_Calc(&zero_pos->mec_arm_motor_angle_pid, zero_pos->angle, zero_pos->angle_set);
    zero_pos->give_current = (int16_t )PID_Calc(&zero_pos->mec_arm_motor_speed_pid, zero_pos->speed, zero_pos->speed_set);
}

/**
 * @brief          末端控制模式:MEC_ARM_ANGLE_CONTROL_MODE，电机编码跟随操作手
 * @param[out]     DJI_Motor_t:末端电机*3
 * @retval         none
 */
static void mec_arm_angle_control(DJI_Motor_t *angle_control, Motor_Type_e type)
{
    if(angle_control == NULL)
    {
        return;
    }

    if(type == DJI_MOTOR_6020)
    {
        angle_control->speed_set    = PID_Calc(&angle_control->mec_arm_motor_angle_pid, angle_control->angle, angle_control->angle_set);
        angle_control->give_current = (int16_t )PID_Calc(&angle_control->mec_arm_motor_speed_pid, angle_control->speed, angle_control->speed_set);
    }

    else if(type == DJI_MOTOR_2006)
    {
        angle_control->speed_set    = PID_Calc(&angle_control->mec_arm_motor_angle_pid, angle_control->angle, angle_control->angle_set);
        angle_control->give_current = (int16_t) PID_Calc(&angle_control->mec_arm_motor_speed_pid, angle_control->speed, angle_control->speed_set);
    }
}

/**
 * @brief          末端控制模式:MEC_ARM_MOTIONLESS_MODE，电机编码保持不动
 * @param[out]     DJI_Motor_t:末端电机*3
 * @retval         none
 */
static void mec_arm_motionless_control(DJI_Motor_t *motionless_control, Motor_Type_e type)
{
    if (motionless_control == NULL)
    {
        return;
    }

    /* 角度环串速度环 */
    if (type == DJI_MOTOR_6020)
    {
        motionless_control->speed_set = PID_Calc(&motionless_control->mec_arm_motor_angle_pid, motionless_control->angle,
                                                 motionless_control->angle_set);
        motionless_control->give_current = (int16_t) PID_Calc(&motionless_control->mec_arm_motor_speed_pid, motionless_control->speed,
                                                              motionless_control->speed_set);
    }
        /* 速度环 */
    else if (type == DJI_MOTOR_2006)
    {
        motionless_control->give_current = (int16_t) PID_Calc(&motionless_control->mec_arm_motor_speed_pid, motionless_control->speed,
                                                              motionless_control->speed_set);
    }
}

static void mec_arm_custom_control(DJI_Motor_t *motionless_control, Motor_Type_e type)
{
    if (motionless_control == NULL)
    {
        return;
    }

    /* 角度环串速度环 */
    if (type == DJI_MOTOR_6020)
    {
        motionless_control->speed_set = PID_Calc(&motionless_control->mec_arm_motor_angle_pid, motionless_control->angle,
                                                 motionless_control->angle_set);
        motionless_control->give_current = (int16_t) PID_Calc(&motionless_control->mec_arm_motor_speed_pid, motionless_control->speed,
                                                              motionless_control->speed_set);
    }
        /* 速度环 */
    else if (type == DJI_MOTOR_2006)
    {
        motionless_control->speed_set = PID_Calc(&motionless_control->mec_arm_motor_angle_pid, motionless_control->angle,
                                                 motionless_control->angle_set);
        motionless_control->give_current = (int16_t) PID_Calc(&motionless_control->mec_arm_motor_speed_pid, motionless_control->speed,
                                                              motionless_control->speed_set);
    }
}
static void detect_2006_current(DJI_Motor_t *s_joint1, DJI_Motor_t *s_joint2)
{
    /* 堵转产生尖峰电流,则不再继续旋转（速度环设定为0 不是直接无力状态） */
    if(abs(s_joint1->mec_arm_motor_measure->given_current) > 9500)
    {
        s_joint1->speed_set = 0;
    }

    if(abs(s_joint2->mec_arm_motor_measure->given_current) > 9500)
    {
        s_joint2->speed_set = 0;
    }
}

/**
 * @brief          解算末端差速器电机角度编码值
 *                 吸盘朝上建立坐标系 对pitch进行限位处理 yaw无限位
 * @param[out]     j1_angle   电机1角度设定值
 *                 j2_angle   电机2角度设定值
 *                 add_pitch  来自用户的角度增量
 *                 add_yaw    来自用户的角度增量
 * @retval         none
 */
static void d_gear_angle_slove(float *j1_angle, float *j2_angle, float add_pitch, float add_yaw)
{
    /* 停止信号，不需要缓慢停止，直接减速到零 */
    if (add_pitch < RC_DEADBAND * S_JOINT1_RC_SEN && add_pitch > -RC_DEADBAND * S_JOINT1_RC_SEN) // 进入摇杆死区
    {
        add_pitch = 0.0f;
    }

    if (add_yaw < RC_DEADBAND * S_JOINT2_RC_SEN && add_yaw > -RC_DEADBAND * S_JOINT2_RC_SEN) // 进入摇杆死区
    {
        add_yaw   = 0.0f;
    }

    /* 对增量进行限幅 */
    add_yaw = float_constrain(add_yaw, -300 * fabsf(S_JOINT2_RC_SEN),300 * fabsf(S_JOINT2_RC_SEN));


    if((mec_arm_control.s_joint_motor[2].angle - mec_arm_control.s_joint_motor[2].origin_angle) - (mec_arm_control.s_joint_motor[1].angle - mec_arm_control.s_joint_motor[1].origin_angle) > 6.28f && add_pitch > 0)
    {
        add_pitch = 0;
    }
    else if((mec_arm_control.s_joint_motor[2].angle - mec_arm_control.s_joint_motor[2].origin_angle) - (mec_arm_control.s_joint_motor[1].angle - mec_arm_control.s_joint_motor[1].origin_angle) <0.01f && add_pitch < 0)
    {
        add_pitch = 0;
    }

    dif_pitch_angle +=  add_pitch;
    dif_yaw_angle   +=  add_yaw * 2.0f;
//    dif_pitch_angle = float_constrain(dif_pitch_angle, 0, 3.00f);


    /* 差速器控制逻辑
     * 当两电机速度一样：仅yaw动   当两电机速度大小相等方向相反：仅pitch动
     * 可知 yaw受控于两电机角度和，pitch受控于两电机角度差
     * */

    /* 差速器减速比似乎是2 */
    *j1_angle = mec_arm_control.s_joint_motor[1].origin_angle - dif_pitch_angle + dif_yaw_angle;
    *j2_angle = mec_arm_control.s_joint_motor[2].origin_angle + dif_pitch_angle + dif_yaw_angle;
}

/**
 * @brief          解算末端差速器电机角度编码值
 *                 吸盘朝上建立坐标系 对pitch进行限位处理 yaw无限位
 * @param[out]     j1_angle   电机1角度设定值
 *                 j2_angle   电机2角度设定值
 *                 add_pitch  来自用户的角度增量
 *                 add_yaw    来自用户的角度增量
 * @retval         none
 */
static void d_gear_angle_gnd_slove(float *j1_angle, float *j2_angle, float pitch)
{
    dif_pitch_angle =  pitch;

    dif_pitch_angle = float_constrain(dif_pitch_angle, 0, 3.3f);

    /* 差速器控制逻辑
     * 当两电机速度一样：仅yaw动   当两电机速度大小相等方向相反：仅pitch动
     * 可知 yaw受控于两电机角度和，pitch受控于两电机角度差
     * */

    /* 差速器减速比似乎是2 */
    *j1_angle = mec_arm_control.s_joint_motor[1].origin_angle - dif_pitch_angle + dif_yaw_angle;
    *j2_angle = mec_arm_control.s_joint_motor[2].origin_angle + dif_pitch_angle + dif_yaw_angle;
}

/**
 * @brief          检测是否堵转，最初用于校准电机角度数据
 * @param[in]      none
 * @retval         返回1 代表已经堵转， 返回0 代表未堵转
 */
static bool detect_2006_block(void)
{
    // 标志位
    static int i = 0;

    /* 判断堵转 */
    if(abs(mec_arm_control.s_joint_motor[1].mec_arm_motor_measure->given_current) > 5000 &&
       abs(mec_arm_control.s_joint_motor[2].mec_arm_motor_measure->given_current) > 5000 &&
       abs(mec_arm_control.s_joint_motor[1].speed - mec_arm_control.s_joint_motor[2].speed) < 0.1f)
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

static uint8_t block_flag = 0;
static TickType_t continue_period = 0; // 记录校准时间


/**
 * @brief          电机校准计算，将校准记录的起点角度值返回
 * @param[in]      none
 * @retval         返回1 代表成功校准完毕， 返回0 代表未校准完
 */
bool cmd_cali_joint_hook(void)
{
    /* 记录初始角度 */
    static uint8_t tick = 0;
    if(tick == 0)
    {
        tick = 1;
        mec_arm_control.s_joint_motor[1].origin_angle = mec_arm_control.s_joint_motor[1].angle;
        mec_arm_control.s_joint_motor[2].origin_angle = mec_arm_control.s_joint_motor[2].angle;
        gimbal_control.gimbal_l_joint1.MIT_Origin_Angle = gimbal_control.gimbal_l_joint1.gimbal_motor_measure->pos;
        gimbal_control.gimbal_l_joint2.MIT_Origin_Angle = gimbal_control.gimbal_l_joint2.gimbal_motor_measure->pos;
        continue_period = xTaskGetTickCount();
    }

    /* 不断比较角度值 */
    if(mec_arm_control.s_joint_motor[1].origin_angle < mec_arm_control.s_joint_motor[1].angle)
    {
        mec_arm_control.s_joint_motor[1].origin_angle = mec_arm_control.s_joint_motor[1].angle;
    }

    if(mec_arm_control.s_joint_motor[2].origin_angle > mec_arm_control.s_joint_motor[2].angle)
    {
        mec_arm_control.s_joint_motor[2].origin_angle = mec_arm_control.s_joint_motor[2].angle;
    }

    if(gimbal_control.gimbal_l_joint1.MIT_Origin_Angle > gimbal_control.gimbal_l_joint1.gimbal_motor_measure->pos) // 不断反转寻找限位
    {
        gimbal_control.gimbal_l_joint1.MIT_Origin_Angle = gimbal_control.gimbal_l_joint1.gimbal_motor_measure->pos;
    }

    if(gimbal_control.gimbal_l_joint2.MIT_Origin_Angle < gimbal_control.gimbal_l_joint2.gimbal_motor_measure->pos) // 不断正转寻找限位
    {
        gimbal_control.gimbal_l_joint2.MIT_Origin_Angle = gimbal_control.gimbal_l_joint2.gimbal_motor_measure->pos;
    }

    /* 检测到两侧的限位之后之后将标志位置1 */
    if (!block_flag)
    {
        mec_arm_control.mec_arm_mode = MEC_ARM_CALI_MODE;
        gimbal_control.gimbal_motor_mode = GIMBAL_CALI_MODE;

        mec_arm_control.s_joint_motor[1].speed_set =  4;
        mec_arm_control.s_joint_motor[2].speed_set = -4;

        gimbal_control.gimbal_l_joint1.kd = 8.0f;
        gimbal_control.gimbal_l_joint1.target_torq = JOINT1_CALI_TORQ;
        gimbal_control.gimbal_l_joint1.target_velocity = JOINT1_CALI_Velocity;

        gimbal_control.gimbal_l_joint2.kd = 1.2f;
        gimbal_control.gimbal_l_joint2.target_torq = JOINT2_CALI_TORQ;
        gimbal_control.gimbal_l_joint2.target_velocity = -JOINT2_CALI_Velocity;

        if(detect_mit_j1_block() == 1 || xTaskGetTickCount() - continue_period > 1500) // 2.0s
        {
            gimbal_control.gimbal_l_joint2.kd = 4.8f;
            gimbal_control.gimbal_l_joint2.target_torq = JOINT2_CALI_TORQ;
            gimbal_control.gimbal_l_joint2.target_velocity = JOINT2_CALI_Velocity;
        }

        block_flag = detect_2006_block() && detect_mit_j1_block() && detect_mit_j2_block();
    }

    /* 超时，退出校准 */
    if(xTaskGetTickCount() - continue_period > 5000) // 5s
    {
        tick = 0;
        block_flag = 0;


        return 1;
    }

    /* 标志位为1，则校准完毕 */
    if(block_flag)
    {
        /* 标志位清零 */
        block_flag = 0;
        tick    = 0;

        return 1;
    }
    else
    {
        return 0;
    }


}