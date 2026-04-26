/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_DJI_Motor.h
  * @brief      DJI 电机驱动层头文件
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 统一处理 M3508、GM6020、M2006 电机反馈
  *             2. 统一处理 C610/C620 电调电流控制帧下发
  *             3. 采用“一个对象管理一组同命令 ID 电机”的风格
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "HAL_FDCAN.h"
#include "HDL_Motor_Device_Base.h"
#include "MWL_Motor.h"

#include <stdint.h>

/**
 * @brief 宏定义
 */
#define HDL_DJI_MOTOR_GROUP_MAX_MOTOR_NUM 4U


/**
 * @brief 结构体
 */
typedef struct
{
    MWL_Motor_Model_e model;
    uint16_t can_id;    // 电调反馈报文标识符（0x200+电调id）
    uint8_t online;
    uint16_t ecd;
    uint16_t last_ecd;
    uint16_t offset_ecd;
    int16_t speed_rpm;  // 电调原始反馈转速，单位 rpm（电机转子侧）
    int16_t given_current;
    uint8_t temperature;
    int32_t total_ecd;
    int32_t turn_count;
    float total_angle_rad;        // 输出轴累计角度，单位 rad
    float output_velocity_rad_s;  // 输出轴角速度，单位 rad/s
    float motor_torque;           // 输出轴力矩，单位Nm
    uint8_t angle_init;
    uint32_t update_count;
} Struct_DJI_Motor_State;

/**
 * @brief 变量外部声明
 */

/**
 * @brief CPP部分
 */
#ifdef __cplusplus

/**
 * @brief DJI 电机组（一拖四）驱动类
 */
class Class_DJI_Motor_Group : public Class_Motor_Device_Base
{
public:
    static const uint8_t kMaxMotorNum = HDL_DJI_MOTOR_GROUP_MAX_MOTOR_NUM; // 消息格式一拖四

    /**
     * @brief 构造函数
     */
    Class_DJI_Motor_Group();

    /**
     * @brief 初始化电机组对象
     * @param[in] FDCAN_Manage_Object FDCAN 管理对象
     * @param[in] command_id          电流控制命令 ID
     */
    void Init(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint16_t command_id);

    /**
     * @brief 注册组内某一个电机
     */
    void RegisterMotor(uint8_t index, uint16_t can_id, MWL_Motor_Model_e model);

    /**
     * @brief 设置某个电机的目标电流
     */
    void SetCurrentCommand(uint8_t index, int16_t current);
    void SetAngleOffset(uint8_t index, uint16_t offset_ecd);
    void UpdateFeedback(uint8_t index);
    void UpdateFeedback(void);

    /**
     * @brief 按组下发 4 路电流命令
     */
    bool SendCommand();

    /**
     * @brief 解析一帧 DJI 反馈
     */
    bool ParseFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length) override;

    const Struct_DJI_Motor_State &GetState(uint8_t index) const;
    uint8_t GetMotorCount() const;

private:
    /**
      * @brief          将累计编码器值换算为弧度
      */
    static float EcdToRad(int32_t total_ecd);
    static float RpmToRadPerSec(float speed_rpm);
    static int32_t NormalizeEcdDelta(int32_t delta_ecd);
    static float GetReductionRatio(MWL_Motor_Model_e model);

    /**
      * @brief          根据反馈 ID 查找组内索引
      */
    int32_t FindIndex(uint16_t identifier) const;

    // 绑定的 FDCAN 管理对象
    FDCAN_Manage_Object_t *FDCAN_Manage_Object_;
    // 电调接收报文标识符（0x200/0x1FF）
    uint16_t command_id_;
    // 电机状态数组（电调反馈）
    Struct_DJI_Motor_State motors_[kMaxMotorNum];
    // 电机电流数组（电调接收）
    int16_t current_command_[kMaxMotorNum];
    // 当前组内注册的电机数量
    uint8_t motor_count_;
};

#endif


/**
 * @brief 函数外部声明
 */
#ifdef __cplusplus
extern "C" {
#endif


#ifdef __cplusplus
}
#endif
