/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_MecArm.cpp
  * @brief      机械臂控制线程实现
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-10-2026      Light           1. done
  *  V1.1.0     04-13-2026      Codex           1. add MWL PID and DJI cascade control
  *
  @verbatim
  ==============================================================================
  * 实现机械臂控制逻辑，当前重点完成：
  * 1. 电机对象初始化
  * 2. 反馈数据更新
  * 3. 末端 6020/2006 的串级双环 PID 电流控制
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "APL_MecArm.h"

#include <string.h>
#include <math.h>
#include "MWL_Trajectory.h"
#include "MWL_Matrix.h"
#include "gravity_trig_model.h"

namespace
{
    /**
    * @brief 对归一化摇杆输入做平滑死区处理
    * @param input     原始输入，范围通常为 [-1, 1]
    * @param deadband  死区大小，范围 [0, 1)
    * @return          死区处理后的输出，仍然在 [-1, 1]
    */
    static float ApplyNormalizedDeadband(float input, float deadband)
    {
        float abs_input = fabsf(input);

        if (abs_input <= deadband)
        {
            return 0.0f;
        }

        float scaled = (abs_input - deadband) / (1.0f - deadband);

        if (input > 0.0f)
        {
            return scaled;
        }
        else
        {
            return -scaled;
        }
    }

}

/**
 * @brief 构造机械臂控制对象
 */
Class_MecArm MecArm;

/**
 * @brief 构造函数
 */
Class_MecArm::Class_MecArm()
    : is_init_(0U),
      mode(MecArm_Mode_Dead),
      dr16_control_(RC_Hub.dr16_control),
      remote_control_(RC_Hub.remote_control),
      vt_rc_control_(RC_Hub.vt_rc_control)
{
    // Joint Space
    memset(joint, 0, sizeof(joint));
    differential_origin_angle_[0] = 0.0f;
    differential_origin_angle_[1] = 0.0f;
    memset(rs_motor_target_position_, 0, sizeof(rs_motor_target_position_));
    memset(rs_motor_target_velocity_, 0, sizeof(rs_motor_target_velocity_));
    memset(rs_motor_target_torque_, 0, sizeof(rs_motor_target_torque_));
    // Cartesian Space
    memset(&cartesian, 0, sizeof(cartesian));
    MWL_Trajectory::InitCartesianPoseTrajectory(cartesian.traj);

    upper_pose_control_enable_ = 0U;
    upper_pose_cmd_pending_ = 0U;
    upper_pose_duration_s_ = 0.0f;
    MWL_Matrix::Vector_SetZero(upper_pose_target_);
    MWL_Matrix::MatIdentity3(upper_pose_R_target_);

}

/**
 * @brief 机械臂控制线程
 */
void _Thread_MecArm_(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(THREAD_MECARM_INIT_TIME);

    MecArm.Init();

    while (1)
    {
        MecArm.UpdateFeedback();

        MecArm.ModeTransit();

        MecArm.SetControl();

        MecArm.SendCommand();

        vTaskDelay(THREAD_MECARM_CONTROL_PERIOD);
    }
}

/**
 * @brief 初始化末端 DJI 电机 PID
 */
void Class_MecArm::InitDJIPid(void)
{
    dji_angle_pid_[APL_MECARM_CAN3_GM6020_INDEX].Init(MWL_PID_POSITION,
                                                      kDJI6020AnglePid,
                                                      150.0f * kRpmToRadPerSec,
                                                      0.0f);
    dji_speed_pid_[APL_MECARM_CAN3_GM6020_INDEX].Init(MWL_PID_POSITION,
                                                      kDJI6020SpeedPid,
                                                      16000.0f,
                                                      0.0f);

    dji_angle_pid_[APL_MECARM_CAN3_M2006_1_INDEX].Init(MWL_PID_POSITION,
                                                       kDJI2006AnglePid,
                                                       45.0f,
                                                       0.0f);
    dji_speed_pid_[APL_MECARM_CAN3_M2006_1_INDEX].Init(MWL_PID_POSITION,
                                                       kDJI2006SpeedPid,
                                                       9500.0f,
                                                       0.0f);

    dji_angle_pid_[APL_MECARM_CAN3_M2006_2_INDEX].Init(MWL_PID_POSITION,
                                                       kDJI2006AnglePid,
                                                       45.0f,
                                                       0.0f);
    dji_speed_pid_[APL_MECARM_CAN3_M2006_2_INDEX].Init(MWL_PID_POSITION,
                                                       kDJI2006SpeedPid,
                                                       9500.0f,
                                                       0.0f);
}

/**
 * @brief 将 DJI 目标值复位到当前位置
 */
void Class_MecArm::ResetDJITarget(void)
{
    const Struct_DJI_Motor_State &motor_1_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_M2006_1_INDEX);
    const Struct_DJI_Motor_State &motor_2_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_M2006_2_INDEX);

    /* 差速器两个 2006 的原始角度零位在应用层保存。
     * 这样 joint[4]/joint[5] 就表示整体机构的 pitch/yaw，
     * 而不是两个电机各自的原始转角。 */
    differential_origin_angle_[0] = motor_1_state.total_angle_rad;
    differential_origin_angle_[1] = motor_2_state.total_angle_rad;

    UpdateDifferentialJointFeedback();

    joint[APL_MECARM_CAN3_GM6020_JOINT_INDEX].target_position = joint[APL_MECARM_CAN3_GM6020_JOINT_INDEX].position;
    joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].target_position = joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].position;
    joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].target_position = joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].position;

    for (uint8_t i = 0U; i < APL_MECARM_CAN3_DJI_MOTOR_NUM; i++)
    {
        uint8_t joint_index = (uint8_t)(APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE + i);
        joint[joint_index].target_velocity = 0.0f;
        joint[joint_index].target_torque = 0.0f;
    }
}

/**
  * @brief          RS03 电机角度正解到应用层关节角
  * @note           电机侧为顺时针为正，应用层统一为逆时针为正；
  *                 同时叠加每个关节的零位偏置再进行减速比转换
  */
float Class_MecArm::DecodeRSJointPosition(uint8_t joint_index, float motor_position) const
{
    return (Joint_Home_Pos[joint_index] - motor_position) / Joint_Ration[joint_index];
}

/**
  * @brief          RS03 电机角速度正解到应用层关节角速度
  */
float Class_MecArm::DecodeRSJointVelocity(uint8_t joint_index, float motor_velocity) const
{
    return -motor_velocity / Joint_Ration[joint_index];
}

/**
  * @brief          RS03 电机力矩正解到应用层关节力矩
  */
float Class_MecArm::DecodeRSJointTorque(uint8_t joint_index, float motor_torque) const
{
    return -motor_torque * Joint_Ration[joint_index];
}

/**
  * @brief          应用层关节角逆解到 RS03 电机侧目标角
  */
float Class_MecArm::EncodeRSJointPosition(uint8_t joint_index, float joint_position) const
{
    return Joint_Home_Pos[joint_index] - joint_position * Joint_Ration[joint_index];;
}

/**
  * @brief          应用层关节角速度逆解到 RS03 电机侧目标角速度
  */
float Class_MecArm::EncodeRSJointVelocity(uint8_t joint_index, float joint_velocity) const
{
    return -joint_velocity * Joint_Ration[joint_index];
}

/**
  * @brief          应用层关节力矩逆解到 RS03 电机侧目标力矩
  */
float Class_MecArm::EncodeRSJointTorque(uint8_t joint_index, float joint_torque) const
{
    return -joint_torque / Joint_Ration[joint_index];
}

/**
  * @brief          更新前三个 RS03 关节的应用层反馈量
  */
void Class_MecArm::UpdateRSJointFeedback(void)
{
    for (uint8_t i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
    {
        const MWL_Motor_State_t &state = can2_rs_motor[i].GetState();

        joint[i].position = DecodeRSJointPosition(i, state.position);
        joint[i].velocity = DecodeRSJointVelocity(i, state.velocity);
        joint[i].torque = DecodeRSJointTorque(i, state.torque);
        joint[i].temperature = state.temperature;
    }
}

/**
  * @brief          将前三个 RS03 关节应用层目标量逆解回电机侧目标量
  * @note           位置、速度、力矩三者都要统一做坐标系反向处理。
  */
void Class_MecArm::UpdateRSJointCommandTarget(void)
{
    for (uint8_t i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
    {
        rs_motor_target_position_[i] = EncodeRSJointPosition(i, joint[i].target_position);
        rs_motor_target_velocity_[i] = EncodeRSJointVelocity(i, joint[i].target_velocity);
        rs_motor_target_torque_[i] = EncodeRSJointTorque(i, joint[i].target_torque);
    }
}

/**
  * @brief          根据两个 2006 原始反馈，解算差速器整体俯仰/偏航反馈
  * @note           两个 2006 的运动关系为：
  *                 motor1 = origin1 - pitch + yaw
  *                 motor2 = origin2 + pitch + yaw
  *                 因此可由两个原始电机角度反解整体机构的俯仰与偏航。
  */
void Class_MecArm::UpdateDifferentialJointFeedback(void)
{
    const Struct_DJI_Motor_State &motor_1_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_M2006_1_INDEX);
    const Struct_DJI_Motor_State &motor_2_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_M2006_2_INDEX);
    float motor_1_delta = motor_1_state.total_angle_rad - differential_origin_angle_[0];
    float motor_2_delta = motor_2_state.total_angle_rad - differential_origin_angle_[1];
    float motor_1_torque = motor_1_state.motor_torque;
    float motor_2_torque = motor_2_state.motor_torque;

    // 换算由 差速器结构 + 齿轮传动比 决定 可以由雅可比矩阵推导出以下结果
    joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].position = 0.5f * (motor_2_delta - motor_1_delta) / kGearPitchRatio;
    joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].velocity = 0.5f * (motor_2_state.output_velocity_rad_s - motor_1_state.output_velocity_rad_s) / kGearPitchRatio;
    joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].torque = kGearPitchRatio * (motor_2_torque - motor_1_torque);
    joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].temperature = 0.5f * ((float)motor_1_state.temperature + (float)motor_2_state.temperature);

    joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].position = -0.5f * (motor_1_delta + motor_2_delta) / kGearYawRatio;
    joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].velocity = -0.5f * (motor_1_state.output_velocity_rad_s + motor_2_state.output_velocity_rad_s) / kGearYawRatio;
    joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].torque = -kGearYawRatio * (motor_1_torque + motor_2_torque);
    joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].temperature = 0.5f * ((float)motor_1_state.temperature + (float)motor_2_state.temperature);
}

/**
  * @brief          根据差速器整体俯仰/偏航目标角，反解两个 2006 电机原始目标角
  * @param[out]     motor_1_target 2006-1 原始目标角
  * @param[out]     motor_2_target 2006-2 原始目标角
  */
void Class_MecArm::SolveDifferentialJointTarget(float *motor_1_target, float *motor_2_target) const
{
    if (motor_1_target == NULL || motor_2_target == NULL)
    {
        return;
    }

    *motor_1_target = differential_origin_angle_[0]
                      - kGearPitchRatio * joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].target_position
                      - kGearYawRatio * joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].target_position;

    *motor_2_target = differential_origin_angle_[1]
                      + kGearPitchRatio * joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].target_position
                      - kGearYawRatio * joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].target_position;
}

/**
 * @brief 纯力矩重力补偿模式
 * @note  只对 J2/J3 下发重力补偿力矩，不加位控、不加速控
 */
void Class_MecArm::SolveGravityTorqueControl(void)
{
    // 导出的重力模型使用的是 q2, q3, q4, q5
    // 在当前 joint[] 语义里分别对应 joint[1], joint[2], joint[3], joint[4]
    const float q2 = joint[1].position;
    const float q3 = joint[2].position;
    const float q4 = joint[3].position;
    const float q5 = joint[4].position;

    float tau_g_j2 = kGravityCompScaleJ2 * MecArm_Gravity_J2(q2, q3, q4, q5);
    float tau_g_j3 = kGravityCompScaleJ3 * MecArm_Gravity_J3(q2, q3, q4, q5);

    tau_g_j2 = MWL_Motor_Clamp(tau_g_j2, -kGravityCompTorqueLimitJ2, kGravityCompTorqueLimitJ2);
    tau_g_j3 = MWL_Motor_Clamp(tau_g_j3, -kGravityCompTorqueLimitJ3, kGravityCompTorqueLimitJ3);

    // J1~J3：纯力矩模式，不加位置/速度控制
    for (int i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
    {
        joint[i].kp = 0.0f;
        joint[i].kd = 0.5f;
        joint[i].target_position = joint[i].position;
        joint[i].target_velocity = 0.0f;
        joint[i].target_torque = 0.0f;
    }

    // 只给 J2 / J3 下发重力补偿力矩
    joint[1].target_torque = tau_g_j2;
    joint[2].target_torque = tau_g_j3;

    UpdateRSJointCommandTarget();

    // 末端保持当前姿态，避免测试时末端乱动
    for (int i = APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE; i < APL_MECARM_CAN3_CLAW_JOINT_INDEX; i++)
    {
        joint[i].target_position = joint[i].position;
        joint[i].target_velocity = 0.0f;
        joint[i].target_torque = 0.0f;
    }
}


/**
 * @brief 初始化机械臂控制对象
 */
void Class_MecArm::Init()
{
    uint8_t i = 0U;

    if (is_init_ != 0U)
    {
        return;
    }

    // 绑定 BUS 与 CAN 物理总线
    Motor_Rx.BindBus(Class_Motor_Rx::MOTOR_CAN_BUS_2, FDCAN2_Manage_Object.hfdcan, FDCAN2_Manage_Object.rx_fifo);
    Motor_Rx.BindBus(Class_Motor_Rx::MOTOR_CAN_BUS_3, FDCAN3_Manage_Object.hfdcan, FDCAN3_Manage_Object.rx_fifo);

    // 挂载电机对象到 Bus
    Motor_Rx.ClearBusDeviceRegistry(Class_Motor_Rx::MOTOR_CAN_BUS_2);
    Motor_Rx.ClearBusDeviceRegistry(Class_Motor_Rx::MOTOR_CAN_BUS_3);

    for (i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
    {
        Motor_Rx.AttachRSMotor(Class_Motor_Rx::MOTOR_CAN_BUS_2,
                               &can2_rs_motor[i],
                               (uint8_t)(3 - i),
                               MWL_MOTOR_MODEL_RS03);

//        can2_rs_motor[i].ClearFault();
        can2_rs_motor[1].SetZero(); // 上电固定J2姿态
        can2_rs_motor[i].Enable();
        vTaskDelay(2);
    }

    Motor_Rx.AttachDJIGroup(Class_Motor_Rx::MOTOR_CAN_BUS_3, &can3_dji_motor_group, 0x1FFU);
    can3_dji_motor_group.RegisterMotor(0U, 0x205U, MWL_MOTOR_MODEL_DJI_GM6020);
    can3_dji_motor_group.RegisterMotor(1U, 0x206U, MWL_MOTOR_MODEL_DJI_M2006);
    can3_dji_motor_group.RegisterMotor(2U, 0x207U, MWL_MOTOR_MODEL_DJI_M2006);

    for (i = 0U; i < APL_MECARM_CAN3_RS_MOTOR_NUM; i++)
    {
        Motor_Rx.AttachRSMotor(Class_Motor_Rx::MOTOR_CAN_BUS_3,
                               &can3_rs_motor[i],
                               (uint8_t)(APL_MECARM_CAN3_RS_MOTOR_ID_BASE + i),
                               MWL_MOTOR_MODEL_EL05);
//        can3_rs_motor[i].ClearFault();
        can3_rs_motor[i].Enable();
        vTaskDelay(2);
    }

    // Joint Space限幅
    for (int i = 0; i < DOF; i++)
    {
        joint[i].pos_max = Joint_Max_Pos[i];
        joint[i].pos_min = Joint_Min_Pos[i];
    }

    // Cartesian Space限幅
    for (int i = 0; i < 3; i++)
    {
        cartesian.x_max[i] = Cartesian_x_Max[i];
        cartesian.x_min[i] = Cartesian_x_Min[i];
    }

    for (int i = 0; i < 6; i++)
    {
        cartesian.q_dot_limit[i] = Joint_q_dot_Limit[i];
    }

    UpdateFeedback();

    InitDJIPid();
    ResetDJITarget();

    ResetCartesianTarget();

    is_init_ = 1U;
}

/**
 * @brief 更新机械臂反馈
 */
void Class_MecArm::UpdateFeedback(void)
{
    uint8_t i = 0U;
    uint8_t joint_index = 0U;

    can3_dji_motor_group.UpdateFeedback();

    // J1-J3 应用层更新 适配上层urdf
    UpdateRSJointFeedback();

    // J4-J6 应用层更新 适配上层urdf
    const Struct_DJI_Motor_State &state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_GM6020_INDEX);
    joint_index = APL_MECARM_CAN3_GM6020_JOINT_INDEX;

    joint[joint_index].position = state.total_angle_rad;
    joint[joint_index].velocity = state.output_velocity_rad_s;
    joint[joint_index].torque = state.motor_torque;
    joint[joint_index].temperature = (float)state.temperature;

    UpdateDifferentialJointFeedback();

    // 末端夹爪不进行应用层更新
    for (i = 0U; i < APL_MECARM_CAN3_RS_MOTOR_NUM; i++)
    {
        const MWL_Motor_State_t &state = can3_rs_motor[i].GetState();
        joint_index = (uint8_t)(APL_MECARM_CAN2_RS_MOTOR_NUM + APL_MECARM_CAN3_DJI_MOTOR_NUM + i);

        joint[joint_index].position = state.position;
        joint[joint_index].velocity = state.velocity;
        joint[joint_index].torque = state.torque;
        joint[joint_index].temperature = state.temperature;
    }

    // 更新笛卡尔空间反馈
    UpdateCartesianFeedback();
}


/**
 * @brief 机械臂控制模式切换
 */
void Class_MecArm::ModeTransit()
{
    uint8_t left_s  = dr16_control_.rc.s[1];
    uint8_t right_s = dr16_control_.rc.s[0];

    if(switch_is_down(left_s) && switch_is_down(right_s))
    {
        mode = MecArm_Mode_Dead;
    }
    else if(switch_is_mid(left_s) && switch_is_down(right_s))
    {
        mode = MecArm_Mode_JointSpace;
    }
    else if(switch_is_mid(left_s) && switch_is_mid(right_s))
    {
        mode = MecArm_Mode_JointSpace_Tool;
    }
    else if(switch_is_up(left_s) && switch_is_down(right_s))
    {
        mode = MecArm_Mode_Cartesian_Twist;
    }
//    else if (switch_is_up(left_s) && switch_is_up(right_s))
//    {
//        mode = MecArm_Mode_Cartesian_PTP;
//    }
    else if(switch_is_up(left_s) && switch_is_up(right_s))
    {
        mode = MecArm_Mode_GravityComp;
    }
}


/**
 * @brief 机械臂控制量计算
 */
void Class_MecArm::SetControl()
{
    static MecArm_Mode_e prev_mode = MecArm_Mode_Dead;

    if (mode == MecArm_Mode_Dead)
    {
        float kp_arr[3] = {0,0,0,};
        float kd_arr[3] = {2.0f, 3.0f, 5.0f};

        for (int i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
        {
            joint[i].kp = kp_arr[i];
            joint[i].kd = kd_arr[i];
            joint[i].target_position = joint[i].position;
            joint[i].target_velocity = 0.0f;
            joint[i].target_torque = 0.0f;
        }

        for (int i = 0; i < APL_MECARM_CAN3_DJI_MOTOR_NUM; i++)
        {
            joint[i + APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE].target_position = joint[i + APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE].position;
        }

        UpdateRSJointCommandTarget();
        prev_mode = mode;
        return;
    }
    else if (mode == MecArm_Mode_Home)
    {
        for (int i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
        {
            joint[i].kp = 5.0f;
            joint[i].kd = 1.0f;
            joint[i].target_position = joint[i].position;
            joint[i].target_velocity = 0.0f;
            joint[i].target_torque = 0.0f;
        }

        UpdateRSJointCommandTarget();
        prev_mode = mode;
        return;
    }
    else if (mode == MecArm_Mode_JointSpace)
    {
        /* 大臂三个受控量分别为：
         * joint[0] -> J1
         * joint[1] -> J2
         * joint[2] -> J3*/
        float increment_slope[3] =  {2.5e-3f, 2.5e-3f, 2.5e-3f}; // 位置增量斜率
        float increment_arr[3] = {dr16_control_.ch_normalized[2], dr16_control_.ch_normalized[3],
                                  dr16_control_.ch_normalized[1]}; // 位置增量数组

        float kp_arr[3] = {8.0f,12.0f,20.0f,};
        float kd_arr[3] = {1.5f, 2.0f, 3.0f};
        // 设置目标位置并位置限幅
        for (int i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
        {
            joint[i].target_position += increment_slope[i] * increment_arr[i];
            joint[i].target_position = MWL_Motor_Clamp(joint[i].target_position, joint[i].pos_min, joint[i].pos_max);

            joint[i].kp = kp_arr[i];
            joint[i].kd = kd_arr[i];
            joint[i].target_velocity = 0.0f;
            joint[i].target_torque = 0.0f;
        }

        UpdateRSJointCommandTarget();
        prev_mode = mode;
        return;
    }
    else if (mode == MecArm_Mode_JointSpace_Tool)
    {
        /* 末端四个受控量分别为：
         * joint[3] -> 6020 单轴角度
         * joint[4] -> 差速器整体 pitch
         * joint[5] -> 差速器整体 yaw
         * joint[6] -> EL05 夹爪 */
        float increment_slope[4] =  {2.5e-3f, 2.5e-3f, 2.5e-3f, 7.0e-3f,}; // 位置增量斜率
        float increment_arr[4] = {dr16_control_.ch_normalized[0], dr16_control_.ch_normalized[3],
                                  dr16_control_.ch_normalized[2],  dr16_control_.ch_normalized[4]}; // 位置增量数组

        // 设置目标位置并位置限幅
        for (int i = APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE; i < DOF; i++)
        {
            joint[i].target_position += increment_slope[i - APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE] * increment_arr[i - APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE];
            joint[i].target_position = MWL_Motor_Clamp(joint[i].target_position, joint[i].pos_min, joint[i].pos_max);
        }

        /* 将自由度目标速度回传 便于应用层调试观察。 */
//        joint[APL_MECARM_CAN3_GM6020_JOINT_INDEX].target_velocity = gm6020_speed_set;
//        joint[APL_MECARM_CAN3_GM6020_JOINT_INDEX].target_torque = (float)gm6020_current;
//        joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].target_velocity = 0.5f * (motor_2_speed_set - motor_1_speed_set);
//        joint[APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX].target_torque = 0.5f * ((float)motor_2_current - (float)motor_1_current);
//        joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].target_velocity = 0.5f * (motor_1_speed_set + motor_2_speed_set);
//        joint[APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX].target_torque = 0.5f * ((float)motor_1_current + (float)motor_2_current);

        UpdateRSJointCommandTarget();
        prev_mode = mode;
        return;
    }
    else if (mode == MecArm_Mode_Cartesian_Twist)
    {
        if(mode != prev_mode)   // 切换到笛卡尔空间控制模式时，重置笛卡尔空间目标为当前位置
        {
            ResetCartesianTarget();

        }
        SetCartesianTarget();
        SolveCartesianClosedLoop();
        UpdateRSJointCommandTarget();

        prev_mode = mode;
        return;
    }
    else if (mode == MecArm_Mode_Cartesian_PTP)
    {
        if(mode != prev_mode)   // 切换到笛卡尔空间控制模式时，重置笛卡尔空间目标为当前位置
        {
            ResetCartesianTarget();

        }
        SetCartesianTarget();
        SolveCartesianClosedLoop();
        UpdateRSJointCommandTarget();

        prev_mode = mode;
        return;
    }
    else if (mode == MecArm_Mode_GravityComp)
    {
        SolveGravityTorqueControl();

        prev_mode = mode;
        return;
    }
}

/**
 * @brief 设置机械臂测试控制命令
 * @note  当前保留空接口，后续由具体模式机填写各关节目标值
 */
void Class_MecArm::SetTestCommand(void)
{
}

/**
 * @brief 下发机械臂控制命令
 */
void Class_MecArm::SendCommand(void)
{
    if (is_init_ == 0U)
    {
        return;
    }

    // can2 rs
    for (int16_t i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++)
    {
        can2_rs_motor[i].SendMITCommand(rs_motor_target_position_[i],
                                        rs_motor_target_velocity_[i],
                                        joint[i].kp,
                                        joint[i].kd,
                                        rs_motor_target_torque_[i]);
    }

    // can3 dji
    if (mode == MecArm_Mode_Dead)
    {
        for (int i = 0; i < 3; i++)
        {
            can3_dji_motor_group.SetCurrentCommand(i, 0);
        }
    }
    else
    {
        const Struct_DJI_Motor_State &gm6020_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_GM6020_INDEX);
        const Struct_DJI_Motor_State &motor_1_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_M2006_1_INDEX);
        const Struct_DJI_Motor_State &motor_2_state = can3_dji_motor_group.GetState(APL_MECARM_CAN3_M2006_2_INDEX);

        // 由整体 pitch/yaw 目标角反解得到两个 2006 的原始目标角
        float motor_1_target = 0.0f;    float motor_2_target = 0.0f;
        SolveDifferentialJointTarget(&motor_1_target, &motor_2_target);

        // Roll
        float gm6020_speed_set = dji_angle_pid_[APL_MECARM_CAN3_GM6020_INDEX].Calc(gm6020_state.total_angle_rad,joint[APL_MECARM_CAN3_GM6020_JOINT_INDEX].target_position);
        float gm6020_current = (int16_t)dji_speed_pid_[APL_MECARM_CAN3_GM6020_INDEX].Calc(gm6020_state.output_velocity_rad_s,gm6020_speed_set);
        can3_dji_motor_group.SetCurrentCommand(APL_MECARM_CAN3_GM6020_INDEX, gm6020_current);

        // Pitch & Yaw  两个 2006 的控制对象仍然是原始电机，但应用层 joint[4]/joint[5] 表示差速器整体关节量

        float motor_1_speed_set = dji_angle_pid_[APL_MECARM_CAN3_M2006_1_INDEX].Calc(motor_1_state.total_angle_rad,motor_1_target);
        float motor_1_current = (int16_t)dji_speed_pid_[APL_MECARM_CAN3_M2006_1_INDEX].Calc(motor_1_state.output_velocity_rad_s,motor_1_speed_set);
        can3_dji_motor_group.SetCurrentCommand(APL_MECARM_CAN3_M2006_1_INDEX, motor_1_current);

        float motor_2_speed_set = dji_angle_pid_[APL_MECARM_CAN3_M2006_2_INDEX].Calc(motor_2_state.total_angle_rad,motor_2_target);
        float motor_2_current = (int16_t)dji_speed_pid_[APL_MECARM_CAN3_M2006_2_INDEX].Calc(motor_2_state.output_velocity_rad_s,motor_2_speed_set);
        can3_dji_motor_group.SetCurrentCommand(APL_MECARM_CAN3_M2006_2_INDEX, motor_2_current);
    }
    can3_dji_motor_group.SendCommand();

    // can3 rs
    for (int16_t i = 0U; i < APL_MECARM_CAN3_RS_MOTOR_NUM; i++)
    {
        uint8_t joint_index = (uint8_t)(APL_MECARM_CAN2_RS_MOTOR_NUM + APL_MECARM_CAN3_DJI_MOTOR_NUM + i);

        joint[joint_index].kp = 5.0f;
        joint[joint_index].kd = 1.0f;
        joint[joint_index].target_velocity = 0.0f;
        joint[joint_index].target_torque = 0.0f;

        can3_rs_motor[i].SendMITCommand(joint[joint_index].target_position,
                                        joint[joint_index].target_velocity,
                                        joint[joint_index].kp,
                                        joint[joint_index].kd,
                                        joint[joint_index].target_torque);
    }
}


/**
 * @brief   初始化机械臂笛卡尔空间 PID 参数
 *          Cartesian Space <--> Joint Space
 */
//void Class_MecArm::InitCartesianPid()
//{
//    for (int i = 0; i < 3; i++)
//    {
//        // 期望笛卡尔空间速度通过PID控制器限幅
//        cartesian_pid_[i].Init(MWL_PID_POSITION,
//                               kCartesianPid,
//                               0.5f,
//                               0.0f);
//    }
//
//}


/**
 * @brief 将 Cartesian Space MecArm_Mode_Cartesian_Twist
 */
void Class_MecArm::ResetCartesianTarget(void)
{
    /* 1. 位置目标复位为当前位置 */
    MWL_Matrix::Vector_Copy(cartesian.x, cartesian.x_target);

    /* 2. 姿态目标复位为当前姿态 */
    MWL_Matrix::Matrix_Copy(cartesian.R, cartesian.R_target);

    /* 3. 控制器内部量清零 */
    MWL_Matrix::Vector_SetZero(cartesian.twist_target);
    MWL_Matrix::Vector_SetZero(cartesian.pose_error);
    MWL_Matrix::Vector_SetZero(cartesian.q_dot_target);

    /* 4. 轨迹生成器复位，并把当前位姿写成参考起点/终点 */
    MWL_Trajectory::InitCartesianPoseTrajectory(cartesian.traj);

    MWL_Matrix::Vector_Copy(cartesian.x, cartesian.traj.p_start);
    MWL_Matrix::Vector_Copy(cartesian.x, cartesian.traj.p_goal);
    MWL_Matrix::Vector_Copy(cartesian.x, cartesian.traj.p_ref);

    MWL_Matrix::Matrix_Copy(cartesian.R, cartesian.traj.R_start);
    MWL_Matrix::Matrix_Copy(cartesian.R, cartesian.traj.R_goal);
    MWL_Matrix::Matrix_Copy(cartesian.R, cartesian.traj.R_ref);

    /* 5. 记录切入关节姿态，作为零空间参考 */
    MWL_Matrix::Vector_Copy(cartesian.q, cartesian.q_null_ref);
    MWL_Matrix::Vector_SetZero(cartesian.q_dot_null_raw);

}



/**
 * @brief  更新机械臂笛卡尔空间反馈
 *         Cartesian Space <--> Joint Space
 */
void Class_MecArm::UpdateCartesianFeedback(void)
{
    // Joint Space
    for (int i = 0; i < 6; i++)
    {
        cartesian.q[i] = joint[i].position;
        cartesian.q_dot[i] = joint[i].velocity;
    }

    // Cartesian Space
//    kinematics.FK(cartesian.q, cartesian.x);
//    kinematics.Calc_Jacobian(cartesian.q, cartesian.J);
//    kinematics.FK_and_Jacobian(cartesian.q, cartesian.x, cartesian.J);
//    kinematics.Calc_CartesianVel(cartesian.q, cartesian.q_dot, cartesian.x_dot);
    kinematics.FK_TCP_and_Jacobian6D(cartesian.q, cartesian.x, cartesian.R, cartesian.J6);  // TCP位姿与雅可比矩阵求解
    kinematics.Calc_TCP_Twist(cartesian.J6, cartesian.q_dot, cartesian.twist);  //  计算笛卡尔空间Twist

}


/**
 * @brief  设置笛卡尔空间控制目标  MecArm_Mode_Cartesian_Twist
 *         Cartesian Space <--> Joint Space
 */
void Class_MecArm::SetCartesianTarget(void)
{
    // 6D控制：只修改目标位置，目标姿态在切入模式时锁存
    const float increment_slope[3] = {0.3f,0.2f,0.3f};// 最大线速度
    float increment_raw[3] = {dr16_control_.ch_normalized[3], dr16_control_.ch_normalized[4],
                              dr16_control_.ch_normalized[1]}; // 位置增量数组

    // 摇杆死区滤波根据摇杆后直接生成前馈速度
    float input_filtered[3];
    for (int i = 0; i <3; i++)
    {
        input_filtered[i] = ApplyNormalizedDeadband(increment_raw[i], kCartesianRcDeadband);
        cartesian.traj.twist_ff[i] = increment_slope[i] * input_filtered[i];
        cartesian.traj.twist_ff[i + 3] = 0.0f; // 姿态不受遥控器输入影响
    }
    // 把前馈速度积分成目标位置，这样松杆后会保持当前位置
    for (int i = 0; i < 3; i++)
    {
        cartesian.x_target[i] += cartesian.traj.twist_ff[i] * THREAD_MECARM_CONTROL_PERIOD_S;

//        cartesian.x_target[i] = MWL_Vector_Clamp(cartesian.x_target[i],
//                                                 cartesian.x_min[i],
//                                                 cartesian.x_max[i]);
    }
}


void Class_MecArm::SetUpperCartesianPoseCommand(const float (&p_des)[3],
                                                const float (&R_des)[3][3],
                                                float duration_s)
{
    /* 只缓存命令，不在通信线程里直接启动轨迹 */
    MWL_Matrix::Vector_Copy(p_des, upper_pose_target_);
    MWL_Matrix::Matrix_Copy(R_des, upper_pose_R_target_);
    upper_pose_duration_s_ = duration_s;

    upper_pose_cmd_pending_ = 1U;
    upper_pose_control_enable_ = 1U;
}

void Class_MecArm::ExitUpperCartesianPoseMode(void)
{
    upper_pose_control_enable_ = 0U;
    upper_pose_cmd_pending_ = 0U;
}



/**
 * @brief 根据上位机下发的笛卡尔空间目标位姿命令，开启新的笛卡尔空间PTP轨迹
 */
void Class_MecArm::ApplyUpperPoseCommand(void)
{
    if (!upper_pose_cmd_pending_)
    {
        return;
    }

    /* 将上位机目标写入当前笛卡尔目标 */
    MWL_Matrix::Vector_Copy(upper_pose_target_, cartesian.x_target);
    MWL_Matrix::Matrix_Copy(upper_pose_R_target_, cartesian.R_target);

    /* 开启一条新的五次项位姿轨迹 */
    StartCartesianTrajectory(upper_pose_duration_s_);

    upper_pose_cmd_pending_ = 0U;
}



/**
 * @brief 开启笛卡尔空间轨迹生成
 */
void Class_MecArm::StartCartesianTrajectory(float duration_s)
{
    /* 从当前位姿重新起轨，避免命令漂移 */
    MWL_Trajectory::StartCartesianPoseTrajectory(
            cartesian.traj,cartesian.x,cartesian.x_target,
            cartesian.R,cartesian.R_target,duration_s);

}

/**
* @brief 更新笛卡尔空间轨迹
*/
void Class_MecArm::UpdateCartesianTrajectory(void)
{
    MWL_Trajectory::UpdateCartesianPoseTrajectory(cartesian.traj, THREAD_MECARM_CONTROL_PERIOD_S);
}

/**
 * @brief  笛卡尔空间闭环求解 映射回关节空间
 *         Cartesian Space <--> Joint Space
 */
void Class_MecArm::SolveCartesianClosedLoop(void)
{
    if (mode == MecArm_Mode_Cartesian_Twist)
    {
        // 计算6D位姿误差 基坐标下表达 前馈twist已在SetCartesianTarget生成
        kinematics.Calc_PoseError_6D(cartesian.x_target, cartesian.x, cartesian.R_target, cartesian.R, cartesian.pose_error);
    }
    else if(mode == MecArm_Mode_Cartesian_PTP)
    {
        // 更新当前轨迹参考位姿+前馈twist
        UpdateCartesianTrajectory();
        // 计算6D位姿误差 基坐标下表达
        kinematics.Calc_PoseError_6D(cartesian.traj.p_ref, cartesian.x, cartesian.traj.R_ref, cartesian.R, cartesian.pose_error);
    }

    // 由pose_error生成P反馈twist + 根据轨迹生成前馈twist    主任务
    for(int i = 0; i < 3; i++)
    {
       cartesian.twist_target[i] = cartesian.traj.twist_ff[i] + kCartesianKpPos6D[i] * cartesian.pose_error[i]; // 位置误差生成线速度
       cartesian.twist_target[i + 3] = cartesian.traj.twist_ff[i + 3] + kCartesianKpRot6D[i] * cartesian.pose_error[i + 3]; // 姿态误差生成角
    }

    // 期望twist限幅
    for (int i = 0; i < 6; i++)
    {
        cartesian.twist_target[i] = MWL_Vector_Clamp(cartesian.twist_target[i], -kCartesianTwistLimit[i], kCartesianTwistLimit[i]);
    }

    // 基于零空间优化 生成关节姿态保持副任务
    for (int i = 0; i < 6; i++)
    {
        cartesian.q_dot_null_raw[i] = kCartesianNullK[i] * (cartesian.q_null_ref[i] - cartesian.q[i]);

        cartesian.q_dot_null_raw[i] = MWL_Vector_Clamp(cartesian.q_dot_null_raw[i], -kCartesianNullQdotLimit[i], kCartesianNullQdotLimit[i]);
    }

    // 基于Weighted DLS 由期望twist求解期望关节空间速度
    kinematics.Solve_JointVel_WeightedDLS_NullSpace_6D(cartesian.J6, cartesian.twist_target, kCartesianWx, kCartesianWq, kCartesianDamping6D, cartesian.q_dot_null_raw, cartesian.q_dot_target);

    // 期望qdot限幅
    for (int i = 0; i < 6; ++i)
    {
        cartesian.q_dot_target[i] = MWL_Motor_Clamp(cartesian.q_dot_target[i], -cartesian.q_dot_limit[i], cartesian.q_dot_limit[i]);
    }

    // 欧拉积分得到期望q
    float kp_arr[3] = {8.0f,15.0f,50.0f,};
    float kd_arr[3] = {2.0f, 5.0f, 15.0f};

    // 将期望q和期望前馈速度作为PD控制量
    for (int i = 0U; i < APL_MECARM_CAN2_RS_MOTOR_NUM; i++) // can2 RS03
    {
        float qdot_cmd = cartesian.q_dot_target[i];

        float next_target_position =
                joint[i].target_position + qdot_cmd * THREAD_MECARM_CONTROL_PERIOD_S;

        float clamped_target_position = MWL_Motor_Clamp(next_target_position,joint[i].pos_min,joint[i].pos_max);

        /* 如果已经到关节限位，清零速度前馈 */
        if ((next_target_position > joint[i].pos_max && qdot_cmd > 0.0f) ||
            (next_target_position < joint[i].pos_min && qdot_cmd < 0.0f))
        {
            qdot_cmd = 0.0f;
        }

        joint[i].target_position = clamped_target_position;
        joint[i].kp = kp_arr[i];
        joint[i].kd = kd_arr[i];
        joint[i].target_velocity = qdot_cmd;
        joint[i].target_torque = 0.0f;
    }

    for (int i = APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE; i < APL_MECARM_CAN3_CLAW_JOINT_INDEX; i++) // can3 dji
    {
        joint[i].target_position += cartesian.q_dot_target[i] * THREAD_MECARM_CONTROL_PERIOD_S;
        joint[i].target_position = MWL_Motor_Clamp(joint[i].target_position, joint[i].pos_min, joint[i].pos_max);
    }
}

void Class_MecArm::GetIdentificationData(float (&q)[6], float (&tau)[6]) const
{
    for (int i = 0; i < 6; ++i)
    {
        q[i] = joint[i].position;
        tau[i] = joint[i].torque;
    }
}
