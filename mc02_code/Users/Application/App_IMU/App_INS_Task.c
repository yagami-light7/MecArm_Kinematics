/**
  ****************************(C) COPYRIGHT 2025 ROBOT-Z****************************
  * @file       INS_task.c/h
  * @brief      2025赛季平衡步兵欧拉角计算任务
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dev-20-2024     冉文治           1. 添加卡尔曼滤波对零漂进行处理
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 ROBOT-Z****************************
  */

#include "App_INS_Task.h"
#include "QuaternionEKF.h"
#include "Alg_PID.h"
#include "Dev_BMI088_Driver.h"
#include "Board_IMU_PWM.h"
#include "Board_DWT.h"
#include "kalman_filter.h"
#include "App_Chassis_Task.h"

// PWM给定
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)

/**
  * @brief          控制bmi088的温度
  * @author         冉文治
  * @param          temp bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float temp);

/**
  * @brief          更新加速度和角速度
  * @author         冉文治
  * @param          gyro 角速度
  * @param          accel 加速度
  * @retval         none
  */
static void imu_feedback_update(float gyro[3], float accel[3]);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

KalmanFilter_t KalmanFilter;

static const float imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
static PidTypeDef_t imu_temp_pid;

extern Chassis_Move_t chassis_move;

uint32_t INS_DWT_Count = 0;
static float dt = 0;
static uint8_t first_temperate;

static float gravity_b[3];

// 角速度
static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
// 加速度
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
// 四元数
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// 欧拉角，单位rad
float INS_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};

/**
  * @brief          imu任务, 初始化 bmi088, 计算欧拉角
  * @author         冉文治
  * @param          pvParameters: NULL
  * @retval         none
  */
void INS_Task(void *pvParameters)
{
    // 四元数EKF初始化
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    // PID初始化
    PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
    const float gravity[3] = {0, 0, 9.81f};

    while(1)
    {
        // 设置时间间隔
        dt = DWT_GetDeltaT(&INS_DWT_Count);
        // 读取数据
        BMI088_Read(&BMI088);
        // 更新加速度和角速度
        imu_feedback_update(INS_gyro, INS_accel);
        // 计算欧拉角
        IMU_QuaternionEKF_Update(INS_gyro[0], INS_gyro[1], INS_gyro[2], INS_accel[0], INS_accel[1], INS_accel[2], dt, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET, INS_angle + INS_YAW_TOTAL_ADDRESS_OFFSET);
        memcpy(INS_quat, QEKF_INS.q, sizeof(QEKF_INS.q));
        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        EarthFrameToBodyFrame(gravity, gravity_b, INS_quat);
        // 同样过一个低通滤波
        for (uint8_t i = 0; i < 3; i++)
        {
            chassis_move.MotionAccel_b[i] = (INS_accel[i] - gravity_b[i]) * dt / (ACCELLPF + dt) + chassis_move.MotionAccel_b[i] * ACCELLPF / (ACCELLPF + dt);
        }
        // 转换回导航系n
        BodyFrameToEarthFrame(chassis_move.MotionAccel_b, chassis_move.MotionAccel_n, INS_quat);
        // 控制温度
        imu_temp_control(BMI088.temperature);
        // 任务延时
        vTaskDelay(IMU_CONTROL_TIME_MS);
    }
}

/**
  * @brief          控制bmi088的温度
  * @author         冉文治
  * @param          temp bmi088的温度
  * @retval         none
  */
static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if (first_temperate)
    {
        PID_Calc(&imu_temp_pid, temp, 40.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        if (temp > 40.0f)
        {
            temp_constant_time++;
            if (temp_constant_time > 200)
            {
                first_temperate = 1;
                imu_temp_pid.Iout = BMI088_TEMP_PWM_MAX / 2.0f;
            }
        }

        IMU_temp_PWM(BMI088_TEMP_PWM_MAX - 1);
    }
}

/**
  * @brief          更新加速度和角速度
  * @author         冉文治
  * @param          gyro 角速度
  * @param          accel 加速度
  * @retval         none
  */
static void imu_feedback_update(float gyro[3], float accel[3])
{
    for (uint8_t i = 0; i < 3; i++)
    {
        gyro[i] = BMI088.gyro[i];
        accel[i] = BMI088.accel[i];
    }
}

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
  * @brief          获取四元数
  * @author         冉文治
  * @param          none
  * @retval         INS_quat的指针
  */
const float *get_INS_quat_point(void)
{
    return INS_quat;
}

/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴, 3:total_yaw 单位 rad/s
  * @author         冉文治
  * @param          none
  * @retval         INS_gyro的指针
  */
const float *get_INS_angle_point(void)
{
    return INS_angle;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         INS_accel的指针
  */
const float *get_gyro_data_point(void)
{
    return INS_gyro;
}

/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @author         冉文治
  * @param          none
  * @retval         INS_accel的指针
  */
const float *get_accel_data_point(void)
{
    return INS_accel;
}
