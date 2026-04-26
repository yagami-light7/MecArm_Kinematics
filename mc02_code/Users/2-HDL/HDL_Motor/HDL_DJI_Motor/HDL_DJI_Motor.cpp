/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       HDL_DJI_Motor.cpp
  * @brief      DJI 电机驱动层实现
  * @note       Hardware Driver Layer 硬件驱动层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Light            1. done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "HDL_DJI_Motor.h"

#include <string.h>

/**
 * @brief DJI 电机相关常量和工具函数
 */
namespace
{
const float kTwoPiOver8192 = 0.0007669903939428206f;
const float kTwoPiOver60 = 0.10471975511965977f;
const float kC610MaxTorqueCurrentA = 10.0f;
const float kC620MaxTorqueCurrentA = 20.0f;
const float kGM6020MaxTorqueCurrentA = 3.0f;
const float kC610CurrentFeedbackFullScale = 10000.0f;
const float kC620CurrentFeedbackFullScale = 16384.0f;
const float kGM6020CurrentFeedbackFullScale = 16384.0f;
const float kGM6020VoltageControlFullScale = 25000.0f;
const float kM2006TorqueConstant = 0.18f;
const float kM3508TorqueConstant = 0.30f;
const float kGM6020TorqueConstant = 0.741f;

float GetCurrentFeedbackRangeAmp(MWL_Motor_Model_e model)
{
    if (model == MWL_MOTOR_MODEL_DJI_M2006)
    {
        return kC610MaxTorqueCurrentA;
    }

    if (model == MWL_MOTOR_MODEL_DJI_M3508)
    {
        return kC620MaxTorqueCurrentA;
    }

    if (model == MWL_MOTOR_MODEL_DJI_GM6020)
    {
        return kGM6020MaxTorqueCurrentA;
    }

    return 0.0f;
}

float GetCurrentFeedbackFullScale(MWL_Motor_Model_e model, uint16_t command_id)
{
    if (model == MWL_MOTOR_MODEL_DJI_M2006)
    {
        return kC610CurrentFeedbackFullScale;
    }

    if (model == MWL_MOTOR_MODEL_DJI_M3508)
    {
        return kC620CurrentFeedbackFullScale;
    }

    if (model == MWL_MOTOR_MODEL_DJI_GM6020)
    {
        if (command_id == 0x1FFU || command_id == 0x2FFU)
        {
            return kGM6020VoltageControlFullScale;
        }

        return kGM6020CurrentFeedbackFullScale;
    }

    return 0.0f;
}

float GetOutputTorqueConstant(MWL_Motor_Model_e model)
{
    if (model == MWL_MOTOR_MODEL_DJI_M2006)
    {
        return kM2006TorqueConstant;
    }

    if (model == MWL_MOTOR_MODEL_DJI_M3508)
    {
        return kM3508TorqueConstant;
    }

    if (model == MWL_MOTOR_MODEL_DJI_GM6020)
    {
        return kGM6020TorqueConstant;
    }

    return 0.0f;
}

float GivenCurrentToOutputTorque(int16_t given_current, MWL_Motor_Model_e model, uint16_t command_id)
{
    float current_range = GetCurrentFeedbackRangeAmp(model);    // 真实电流量程 单位 A
    float current_full_scale = GetCurrentFeedbackFullScale(model, command_id);  // 控制电流量程 单位 A 与given_current进行归一化
    float torque_constant = GetOutputTorqueConstant(model); // 力矩常数，单位 Nm/A

    if (current_range <= 0.0f || current_full_scale <= 0.0f || torque_constant <= 0.0f)
    {
        return 0.0f;
    }

    return ((float)given_current / current_full_scale) * current_range * torque_constant;
}

}

/**
 * @brief DJI电机组构造
 */
Class_DJI_Motor_Group::Class_DJI_Motor_Group()
    : FDCAN_Manage_Object_(NULL), command_id_(0U), motor_count_(0U)
{
    memset(motors_, 0, sizeof(motors_));
    memset(current_command_, 0, sizeof(current_command_));
}

/**
  * @brief          初始化电机组对象
  */
void Class_DJI_Motor_Group::Init(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint16_t command_id)
{
    FDCAN_Manage_Object_ = FDCAN_Manage_Object;
    command_id_ = command_id;
}

/**
  * @brief          注册组内某一个电机
  */
void Class_DJI_Motor_Group::RegisterMotor(uint8_t index, uint16_t can_id, MWL_Motor_Model_e model)
{
    if (index >= kMaxMotorNum)
    {
        return;
    }

    memset(&motors_[index], 0, sizeof(motors_[index]));
    motors_[index].model = model;
    motors_[index].can_id = can_id;

    if (motor_count_ < (index + 1U))
    {
        motor_count_ = index + 1U;
    }
}

/**
  * @brief          设置某个电机的目标电流
  */
void Class_DJI_Motor_Group::SetCurrentCommand(uint8_t index, int16_t current)
{
    if (index >= kMaxMotorNum)
    {
        return;
    }

    current_command_[index] = current;
}

void Class_DJI_Motor_Group::SetAngleOffset(uint8_t index, uint16_t offset_ecd)
{
    if (index >= kMaxMotorNum)
    {
        return;
    }

    motors_[index].offset_ecd = offset_ecd;
}

void Class_DJI_Motor_Group::UpdateFeedback(uint8_t index)
{
    Struct_DJI_Motor_State *motor = NULL;
    int32_t delta_ecd = 0;
    int32_t relative_ecd = 0;
    float reduction_ratio = 1.0f;

    if (index >= motor_count_)
    {
        return;
    }

    motor = &motors_[index];
    if (motor->online == 0U)
    {
        return;
    }

    reduction_ratio = GetReductionRatio(motor->model);
    motor->output_velocity_rad_s = RpmToRadPerSec((float)motor->speed_rpm) / reduction_ratio;
    motor->motor_torque = GivenCurrentToOutputTorque(motor->given_current, motor->model, command_id_);

    if (motor->model == MWL_MOTOR_MODEL_DJI_GM6020)
    {
        SetAngleOffset(0,5417);
        relative_ecd = NormalizeEcdDelta((int32_t)motor->ecd - (int32_t)motor->offset_ecd);
        motor->last_ecd = motor->ecd;
        motor->total_ecd = relative_ecd;
        motor->turn_count = 0;
        motor->total_angle_rad = EcdToRad(relative_ecd);
        motor->angle_init = 1U;
        return;
    }

    if (motor->angle_init == 0U)
    {
        motor->last_ecd = motor->ecd;
        motor->total_ecd = 0;
        motor->turn_count = 0;
        motor->total_angle_rad = 0.0f;
        motor->angle_init = 1U;
        return;
    }

    delta_ecd = NormalizeEcdDelta((int32_t)motor->ecd - (int32_t)motor->last_ecd);
    motor->last_ecd = motor->ecd;
    motor->total_ecd += delta_ecd;
    motor->turn_count = motor->total_ecd / 8192;
    motor->total_angle_rad = EcdToRad(motor->total_ecd) / reduction_ratio;
}

void Class_DJI_Motor_Group::UpdateFeedback(void)
{
    uint8_t i = 0U;

    for (i = 0U; i < motor_count_; i++)
    {
        UpdateFeedback(i);
    }
}

/**
  * @brief          发送一组 DJI 电机电流命令
  */
bool Class_DJI_Motor_Group::SendCommand()
{
    uint8_t tx_data[8];

    if (FDCAN_Manage_Object_ == NULL || command_id_ == 0U)
    {
        return false;
    }

    MWL_Motor_Write_BE_U16(&tx_data[0], (uint16_t)current_command_[0]);
    MWL_Motor_Write_BE_U16(&tx_data[2], (uint16_t)current_command_[1]);
    MWL_Motor_Write_BE_U16(&tx_data[4], (uint16_t)current_command_[2]);
    MWL_Motor_Write_BE_U16(&tx_data[6], (uint16_t)current_command_[3]);

    return FDCAN_Send_Std_Data(FDCAN_Manage_Object_, command_id_, tx_data, sizeof(tx_data));
}

/**
  * @brief          解析 DJI 电机反馈
  * @note           一帧反馈只对应一个电机，Identifier 即反馈 ID
  */
bool Class_DJI_Motor_Group::ParseFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length)
{
    Struct_DJI_Motor_State *motor = NULL;
    int32_t index = -1;

    if (data == NULL || length < 7U || header.IdType != FDCAN_STANDARD_ID)
    {
        return false;
    }

    index = FindIndex(header.Identifier);
    if (index < 0)
    {
        return false;
    }

    motor = &motors_[index];
    motor->ecd = MWL_Motor_Read_BE_U16(&data[0]);
    motor->speed_rpm = (int16_t)MWL_Motor_Read_BE_U16(&data[2]);
    motor->given_current = (int16_t)MWL_Motor_Read_BE_U16(&data[4]);
    motor->temperature = data[6];
    motor->online = 1U;
    motor->update_count++;

    return true;
}

/**
  * @brief          获取指定电机状态
  */
const Struct_DJI_Motor_State &Class_DJI_Motor_Group::GetState(uint8_t index) const
{
    return motors_[(index < kMaxMotorNum) ? index : 0U];
}

/**
  * @brief          获取当前组内注册的电机数量
  */
uint8_t Class_DJI_Motor_Group::GetMotorCount() const
{
    return motor_count_;
}

/**
  * @brief          将累计编码器值换算为弧度
  */
float Class_DJI_Motor_Group::EcdToRad(int32_t total_ecd)
{
    return (float)total_ecd * kTwoPiOver8192;
}

float Class_DJI_Motor_Group::RpmToRadPerSec(float speed_rpm)
{
    return speed_rpm * kTwoPiOver60;
}

int32_t Class_DJI_Motor_Group::NormalizeEcdDelta(int32_t delta_ecd)
{
    if (delta_ecd > 4096)
    {
        delta_ecd -= 8192;
    }
    else if (delta_ecd < -4096)
    {
        delta_ecd += 8192;
    }

    return delta_ecd;
}

float Class_DJI_Motor_Group::GetReductionRatio(MWL_Motor_Model_e model)
{
    if (model == MWL_MOTOR_MODEL_DJI_M2006)
    {
        return 36.0f;
    }

    if (model == MWL_MOTOR_MODEL_DJI_M3508)
    {
        return (3591.0f / 187.0f);
    }

    return 1.0f;
}

/**
  * @brief          根据反馈 ID 查找组内索引
  */
int32_t Class_DJI_Motor_Group::FindIndex(uint16_t identifier) const
{
    uint8_t i = 0U;

    for (i = 0U; i < motor_count_; i++)
    {
        if (motors_[i].can_id == identifier)
        {
            return (int32_t)i;
        }
    }

    return -1;
}
