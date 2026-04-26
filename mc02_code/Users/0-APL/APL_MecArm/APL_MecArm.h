/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       APL_MecArm.h
  * @brief      机械臂控制线程头文件
  * @note       Application Layer
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     04-10-2026      Light           1. done
  *  V1.1.0     04-13-2026      Codex           1. add MWL PID and DJI cascade control
  *
  @verbatim
  ==============================================================================
  * 本线程负责机械臂应用层控制逻辑，主要包括：
  * 1. 关节电机初始化
  * 2. 机械臂状态更新
  * 3. 机械臂控制模式切换
  * 4. 机械臂控制命令下发
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

#pragma once

/**
 * @brief 头文件
 */
#include "main.h"
#include "cmsis_os.h"
#include "APL_RC_Hub.h"
#include "HDL_Motor_Rx.h"
#include "MWL_Motor.h"
#include "MWL_PID.h"
#include "MWL_Kinematics.h"
#include "MWL_Trajectory.h"

/**
 * @brief 宏定义
 */
#define THREAD_MECARM_INIT_TIME 3000U
#define THREAD_MECARM_CONTROL_PERIOD 1U
#define THREAD_MECARM_CONTROL_PERIOD_S 0.001f
#define APL_MECARM_DOF 7U
#define APL_MECARM_CAN2_RS_MOTOR_NUM  3U
#define APL_MECARM_CAN3_DJI_MOTOR_NUM 3U
#define APL_MECARM_CAN3_RS_MOTOR_NUM  1U
#define APL_MECARM_CAN3_RS_MOTOR_ID_BASE 0x04U
#define APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE APL_MECARM_CAN2_RS_MOTOR_NUM
#define APL_MECARM_CAN3_RS_JOINT_INDEX_BASE  APL_MECARM_CAN2_RS_MOTOR_NUM+APL_MECARM_CAN3_DJI_MOTOR_NUM
#define APL_MECARM_CAN3_GM6020_INDEX 0U
#define APL_MECARM_CAN3_M2006_1_INDEX 1U
#define APL_MECARM_CAN3_M2006_2_INDEX 2U
#define APL_MECARM_CAN3_GM6020_JOINT_INDEX (APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE + APL_MECARM_CAN3_GM6020_INDEX)
#define APL_MECARM_CAN3_DIFF_PITCH_JOINT_INDEX (APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE + APL_MECARM_CAN3_M2006_1_INDEX)
#define APL_MECARM_CAN3_DIFF_YAW_JOINT_INDEX (APL_MECARM_CAN3_DJI_JOINT_INDEX_BASE + APL_MECARM_CAN3_M2006_2_INDEX)
#define APL_MECARM_CAN3_CLAW_JOINT_INDEX (APL_MECARM_CAN3_RS_JOINT_INDEX_BASE)

/**
 * @brief 结构体
 */

typedef enum
{
    MecArm_Mode_Dead = 0,       // 无力模式
    MecArm_Mode_Home,           // 归位模式
    MecArm_Mode_JointSpace,         // 关节空间控制模式_大臂
    MecArm_Mode_JointSpace_Tool,    // 关节空间控制模式_末端
    MecArm_Mode_Teacher,            // 示教模式
    MecArm_Mode_Cartesian_Twist,    // 笛卡尔空间控制模式(twist)
    MecArm_Mode_Cartesian_PTP,      // 笛卡尔空间控制模式(Point to Point)
    MecArm_Mode_GravityComp,        // 重力补偿模式
}
MecArm_Mode_e;


typedef struct
{
    float position;
    float velocity;
    float torque;
    float temperature;

    float target_position;
    float target_velocity;
    float target_torque;
    float kp;
    float kd;

    float pos_max;
    float pos_min;
} MecArm_Joint_t;


/**
 * @brief 变量外部声明
 */


/**
 * @brief CPP部分
 */
#ifdef __cplusplus

namespace
{
    const float kRpmToRadPerSec = 0.10471975511965977f;
    const float kGearPitchRatio = 1.50f;
    const float kGearYawRatio   = 4.50f;

    // Joint Space限位
    const float Joint_Max_Pos[APL_MECARM_DOF]={+3.10f, 0,      4.00f, +3.10f, +3.10f, +1.57f, 1.97f};
    const float Joint_Min_Pos[APL_MECARM_DOF]={-3.10f, -2.60f, 0,     -3.10f, 0.0f,   -1.57f, 0};

    // RS03 电机角度零位偏移，单位 rad  joint2的记忆功能貌似失效 无法重设零点
    const float Joint_Home_Pos[APL_MECARM_DOF]={0, 0, 2.657f, 0,0,0,0};

    // RS03 电机减速比 仅J2有1:2的传动
const float Joint_Ration[APL_MECARM_DOF]={1.0f, 2.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

    // Cartesian Space限位
    const float Cartesian_x_Max[3]={0, 0, 0};
    const float Cartesian_x_Min[3]={0, 0, 0};

    // Joint Space速度限幅
    const float Joint_q_dot_Limit[6]={1.0f, 1.0f, 1.0f, 3.0f, 3.0f, 3.0f};

    /* 参考旧 App_Mechanical_Arm_Task.h 中的小关节 PID 参数。
 * 其中 6020 在旧框架里速度反馈单位是 rpm，新框架统一改成 rad/s，
 * 因此这里做了单位等效换算，保持旧参数的控制力度基本一致。 */
    const float kDJI6020AnglePid[3] = {
            150.0f * kRpmToRadPerSec,
            0.0f,
            0.0f
    };

    const float kDJI6020SpeedPid[3] = {
            150.0f / kRpmToRadPerSec,
            0.0f,
            10.0f / kRpmToRadPerSec
    };

    const float kDJI2006AnglePid[3] = {
            80.0f,
            0.0f,
            0.0f
    };

    const float kDJI2006SpeedPid[3] = {
            780.0f,
            0.0f,
            15.0f
    };

    // 3D Cartesian pose control 参数
    const float kCartesianPid[3] = {
            3.0f,
            0.0f,
            0.0f
    };

    // J2/J3 重力补偿缩放系数，第一版先保守一点
    const float kGravityCompScaleJ2 = 0.85f;
    const float kGravityCompScaleJ3 = 1.05f;

    // J2/J3 重力补偿力矩限幅
    const float kGravityCompTorqueLimitJ2 = 20.0f;
    const float kGravityCompTorqueLimitJ3 = 10.0f;



    // 6D Cartesian pose control 参数
    const float kCartesianKpPos6D[3] = {
            2.0f, 2.0f, 2.0f
    };

    const float kCartesianKpRot6D[3] = {
            5.0f, 5.0f, 5.0f
    };

    // twist限幅: [vx, vy, vz, wx, wy, wz]
    const float kCartesianTwistLimit[6] = {
            0.3f, 0.3f, 0.3f,
            1.0f, 1.0f, 1.00f
    };

    // W-DLS：笛卡尔空间权重
    const float kCartesianWx[6] = {
            1.0f, 1.0f, 1.0f,
            2.0f, 2.0f, 2.0f
    };

    // W-DLS：关节空间权重
    const float kCartesianWq[6] = {
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f
    };

    // W-DLS：阻尼系数
    const float kCartesianDamping6D = 0.05f;

    // 零空间关节姿态保持增益
    const float kCartesianNullK[6] = {
            0.30f, 0.60f, 0.60f,
            0.00f, 0.00f, 0.00f
    };

    // 零空间关节速度限幅
    const float kCartesianNullQdotLimit[6] = {
            0.10f, 0.10f, 0.10f,
            0.00f, 0.00f, 0.00f
    };


    const float kCartesianRcDeadband = 0.05f;   // 归一化摇杆死区，范围[0,1]


}


typedef struct
{
    /* 笛卡尔空间反馈量 */
    float x[3];         // TCP 当前位置
    float x_dot[3];     // TCP 当前线速度
    float R[3][3];      // TCP 当前姿态

    /* 笛卡尔空间目标量 */
    float x_target[3];      // TCP 目标位置
    float R_target[3][3];   // TCP 目标姿态

    /* 控制器内部量 */
    float twist[6];         // TCP 当前 twist = [v; w]
    float twist_target[6];  // TCP 目标 twist = 前馈 + 反馈
    float pose_error[6];    // TCP 6D 位姿误差

    /* 工作空间与雅可比 */
    float x_max[3];
    float x_min[3];
    float J[3][6];          // 旧 3D 位置雅可比，调试保留
    float J6[6][6];         // TCP 6D 雅可比

    /* 关节空间量 */
    float q[6];
    float q_dot[6];
    float q_dot_target[6];
    float q_dot_limit[6];

    float q_null_ref[6];       // 零空间偏好姿态参考
    float q_dot_null_raw[6];   // 零空间原始偏好速度

    /* 轨迹生成器 */
    MWL_Trajectory::CartesianPoseTrajectory_t traj;

} MecArm_Cartesian_t;


class Class_MecArm
{
public:
    /**
     * @brief 机械臂自由度数量
     */
    static const uint8_t DOF = APL_MECARM_DOF;

    /**
     * @brief 构造函数
     */
    Class_MecArm();

    /**
     * @brief 初始化机械臂
     */
    void Init(void);

    /**
     * @brief 更新机械臂反馈
     */
    void UpdateFeedback(void);

    /**
     * @brief 机械臂控制模式切换
     */
    void ModeTransit(void);

    /**
     * @brief 机械臂控制模式切换
     */
    void SetControl(void);

    /**
     * @brief 设置机械臂测试控制命令
     */
    void SetTestCommand(void);

    /**
     * @brief 下发机械臂控制命令
     */
    void SendCommand(void);
    void GetIdentificationData(float (&q)[6], float (&tau)[6]) const;

    /**
    * @brief  根据上位机下发的目标位姿，设置笛卡尔空间目标位姿
    */
    void SetUpperCartesianPoseCommand(const float (&p_des)[3],
                                      const float (&R_des)[3][3],
                                      float duration_s);

    /**
     * @brief 退出上位机笛卡尔空间控制模式，回到原有控制模式
     */
    void ExitUpperCartesianPoseMode(void);

private:

    /* 状态机 */
    MecArm_Mode_e mode;
    const dr16_control_t &dr16_control_;
    const remote_control_t &remote_control_;
    const vt_rc_control_t &vt_rc_control_;

    /* 电机空间 <-> 关节空间 */
    void InitDJIPid(void);
    void ResetDJITarget(void);
    void UpdateRSJointFeedback(void);
    void UpdateRSJointCommandTarget(void);
    float DecodeRSJointPosition(uint8_t joint_index, float motor_position) const;
    float DecodeRSJointVelocity(uint8_t joint_index, float motor_velocity) const;
    float DecodeRSJointTorque(uint8_t joint_index, float motor_torque) const;
    float EncodeRSJointPosition(uint8_t joint_index, float joint_position) const;
    float EncodeRSJointVelocity(uint8_t joint_index, float joint_velocity) const;
    float EncodeRSJointTorque(uint8_t joint_index, float joint_torque) const;
    void UpdateDifferentialJointFeedback(void);
    void SolveDifferentialJointTarget(float *motor_1_target, float *motor_2_target) const;
    void SolveGravityTorqueControl(void);

    uint8_t is_init_;
    MecArm_Joint_t joint[DOF];
    Class_PID dji_angle_pid_[APL_MECARM_CAN3_DJI_MOTOR_NUM];
    Class_PID dji_speed_pid_[APL_MECARM_CAN3_DJI_MOTOR_NUM];
    float differential_origin_angle_[2];
    float rs_motor_target_position_[APL_MECARM_CAN2_RS_MOTOR_NUM];
    float rs_motor_target_velocity_[APL_MECARM_CAN2_RS_MOTOR_NUM];
    float rs_motor_target_torque_[APL_MECARM_CAN2_RS_MOTOR_NUM];
    Class_RS_Motor can2_rs_motor[APL_MECARM_CAN2_RS_MOTOR_NUM];   // 关节电机，RS03*3
    Class_DJI_Motor_Group can3_dji_motor_group;                   // 末端电机组，6020*1 + 2006*2
    Class_RS_Motor can3_rs_motor[APL_MECARM_CAN3_RS_MOTOR_NUM];   // 夹爪电机，EL05*1

    /* 关节空间 <-> 笛卡尔空间 */

    /**
     * @brief 初始化笛卡尔空间 PID 参数
     */
//    void InitCartesianPid(void);

    /**
     * @brief 将 Cartesian Space 目标值复位到当前位置
     */
    void ResetCartesianTarget(void);

    /**
     * @brief 更新笛卡尔空间反馈
     */
    void UpdateCartesianFeedback(void);

    /**
     * @brief 设置笛卡尔空间控制目标
     */
    void SetCartesianTarget(void);

    /**
     * @brief 开启笛卡尔空间轨迹生成
     */
    void StartCartesianTrajectory(float duration_s = 0.0f);

    /**
     * @brief 更新笛卡尔空间轨迹
     */
    void UpdateCartesianTrajectory(void);

    /**
     * @brief 根据上位机下发的笛卡尔空间目标位姿命令，开启新的笛卡尔空间PTP轨迹
     */
    void ApplyUpperPoseCommand(void);

    /**
     * @brief 笛卡尔空间闭环求解 映射回关节空间
     */
    void SolveCartesianClosedLoop(void);

    MecArm_Cartesian_t cartesian;
    Class_PID cartesian_pid_[3];
    Class_Kinematics kinematics;

    /* 上位机笛卡尔离散目标命令 */
    uint8_t upper_pose_control_enable_;   // 上位机是否接管笛卡尔模式
    uint8_t upper_pose_cmd_pending_;      // 是否有新目标等待 APL 线程处理
    float upper_pose_target_[3];          // 上位机下发的位置目标
    float upper_pose_R_target_[3][3];     // 上位机下发的姿态目标
    float upper_pose_duration_s_;         // 上位机指定轨迹时间


};

extern Class_MecArm MecArm;

#endif

/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 机械臂控制线程
 */
extern void _Thread_MecArm_(void *pvParameters);

#ifdef __cplusplus
}
#endif
