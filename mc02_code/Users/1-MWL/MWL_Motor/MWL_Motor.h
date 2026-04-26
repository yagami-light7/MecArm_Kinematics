/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       MWL_Motor.h
  * @brief      电机中间件公共类型与公共工具函数
  * @note       Middleware Layer 中间件层
  *             1. 统一不同品牌电机的状态描述
  *             2. 提供 MIT 协议常用的浮点/整数映射函数
  *             3. 提供大端/小端字节读写工具，减少各驱动重复代码
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *
  @verbatim
  ==============================================================================
  * 本文件不直接操作 FDCAN 外设，也不感知具体电机品牌协议细节，
  * 只负责提供所有电机驱动都会复用的基础数据结构与工具函数。
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

#include "main.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 电机品牌枚举
 */
typedef enum
{
    MWL_MOTOR_BRAND_NONE = 0,
    MWL_MOTOR_BRAND_DJI,
    MWL_MOTOR_BRAND_DAMIAO,
    MWL_MOTOR_BRAND_RS,
} MWL_Motor_Brand_e;

/**
 * @brief 电机具体型号枚举
 */
typedef enum
{
    MWL_MOTOR_MODEL_UNKNOWN = 0,
    MWL_MOTOR_MODEL_DJI_M3508,
    MWL_MOTOR_MODEL_DJI_GM6020,
    MWL_MOTOR_MODEL_DJI_M2006,
    MWL_MOTOR_MODEL_DM4310,
    MWL_MOTOR_MODEL_RS03,
    MWL_MOTOR_MODEL_EL05,
} MWL_Motor_Model_e;

/**
 * @brief 电机当前运行模式
 */
typedef enum
{
    MWL_MOTOR_RUN_MODE_DISABLED = 0,
    MWL_MOTOR_RUN_MODE_MIT,
    MWL_MOTOR_RUN_MODE_POSITION,
    MWL_MOTOR_RUN_MODE_SPEED,
} MWL_Motor_Run_Mode_e;

/**
 * @brief MIT 模式下的目标指令
 * @note 该结构体仅用于表达“位置/速度/Kp/Kd/力矩”这五个核心参数
 */
typedef struct
{
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;
} MWL_MIT_Command_t;

/**
 * @brief 统一的电机状态结构体
 * @note 该结构体用于上层统一查看不同品牌电机的反馈信息
 */
typedef struct
{
    MWL_Motor_Brand_e brand;
    MWL_Motor_Model_e model;
    MWL_Motor_Run_Mode_e run_mode;
    uint8_t can_id;
    uint8_t online;
    uint8_t err_state;
    uint32_t update_count;
    uint32_t fault_code;
    float position;
    float velocity;
    float torque;
    float temperature;
    float auxiliary_temperature;
    float last_command_kp;
    float last_command_kd;
    uint16_t raw_position;
    uint16_t raw_velocity;
    uint16_t raw_torque;
    uint16_t raw_temperature;
} MWL_Motor_State_t;

/**
 * @brief 对输入值做上下限约束
 */
extern float MWL_Motor_Clamp(float value, float min_value, float max_value);

/**
 * @brief 将浮点量按指定量程映射为无符号整数
 * @param[in] value      输入浮点值
 * @param[in] min_value  量程下限
 * @param[in] max_value  量程上限
 * @param[in] bits       目标位宽
 * @retval 对应的无符号整数值
 */
extern uint16_t MWL_Motor_Float_To_Uint(float value, float min_value, float max_value, uint32_t bits);

/**
 * @brief 将无符号整数按指定量程反解为浮点量
 */
extern float MWL_Motor_Uint_To_Float(uint32_t value, float min_value, float max_value, uint32_t bits);

/**
 * @brief 读取大端 16 位无符号整数
 */
extern uint16_t MWL_Motor_Read_BE_U16(const uint8_t *data);

/**
 * @brief 写入大端 16 位无符号整数
 */
extern void MWL_Motor_Write_BE_U16(uint8_t *data, uint16_t value);

/**
 * @brief 写入小端 16 位无符号整数
 */
extern void MWL_Motor_Write_LE_U16(uint8_t *data, uint16_t value);

/**
 * @brief 读取小端 32 位无符号整数
 */
extern uint32_t MWL_Motor_Read_LE_U32(const uint8_t *data);

/**
 * @brief 按小端格式写入 float
 */
extern void MWL_Motor_Write_LE_Float(uint8_t *data, float value);

/**
 * @brief 按小端格式读取 float
 */
extern float MWL_Motor_Read_LE_Float(const uint8_t *data);

/**
 * @brief 将 FDCAN 的 DLC 编码转换为实际字节数
 */
extern uint32_t MWL_Motor_Dlc_To_Length(uint32_t dlc);

#ifdef __cplusplus
}
#endif
