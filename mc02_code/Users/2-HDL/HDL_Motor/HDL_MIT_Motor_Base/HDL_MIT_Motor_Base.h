/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_MIT_Motor_Base.h
  * @brief      MIT 协议电机公共基类
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 抽出 MIT 电机共有的使能/失能/标零/控制帧格式
  *             2. 抽出 MIT 回读共有的反馈解析流程
  *             3. 通过虚函数把品牌差异下放到子类处理
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

/**
 * @brief 宏定义
 */

/**
 * @brief 结构体
 */

/**
 * @brief 变量外部声明
 */

/**
 * @brief CPP部分
 */
#ifdef __cplusplus

/**
 * @brief MIT 协议电机公共基类
 * @note 适用于达妙 DM4310、灵足 RS03 这类都支持标准帧 MIT 指令的电机
 */
class Class_MIT_Motor_Base : public Class_Motor_Device_Base
{
public:
    /**
     * @brief 默认主机 ID
     * @note 目前按项目现有方案统一使用 0xFD
     */
    static const uint16_t kDefaultMasterId = 0x00FDU;

    /**
     * @brief 构造函数
     * @param[in] brand 电机品牌
     */
    Class_MIT_Motor_Base(MWL_Motor_Brand_e brand);
    virtual ~Class_MIT_Motor_Base() {}

    /**
     * @brief 初始化电机驱动对象
     */
    void Init(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint8_t can_id, uint16_t master_id = kDefaultMasterId);

    /**
     * @brief 设置当前电机具体型号
     * @note 构造阶段只确定协议家族/品牌，具体型号在 Attach/Init 阶段确定
     */
    void SetMotorModel(MWL_Motor_Model_e model);

    /**
     * @brief 发送电机使能帧
     */
    bool Enable();

    /**
     * @brief 发送电机失能帧
     */
    bool Disable();

    /**
     * @brief 发送设置零点帧
     */
    bool SetZero();

    /**
     * @brief 发送 MIT 五参数控制帧
     */
    bool SendMITCommand(float position, float velocity, float kp, float kd, float torque);

    /**
     * @brief 解析一帧 MIT 反馈数据
     */
    bool ParseFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length) override;

    /**
     * @brief 判断一帧反馈是否属于当前电机
     */
    virtual bool MatchesFeedback(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length) const;

    /**
     * @brief 获取当前电机状态
     */
    const MWL_Motor_State_t &GetState() const;

    /**
     * @brief 获取当前电机 CAN ID
     */
    uint8_t GetCanId() const;

protected:
    /**
     * @brief 以下纯虚函数由子类提供具体量程
     */
    virtual float PositionMin() const = 0;
    virtual float PositionMax() const = 0;
    virtual float VelocityMin() const = 0;
    virtual float VelocityMax() const = 0;
    virtual float KpMin() const;
    virtual float KpMax() const = 0;
    virtual float KdMin() const;
    virtual float KdMax() const = 0;
    virtual float TorqueMin() const = 0;
    virtual float TorqueMax() const = 0;

    /**
     * @brief 由子类决定如何从反馈帧中解析 CAN ID、错误码和故障码
     * @note 这是为了兼容 DM 和 RS 在反馈帧 Byte0 定义上的差异
     */
    virtual uint8_t DecodeCanId(const uint8_t *data) const;
    virtual uint8_t DecodeErrState(const uint8_t *data) const;
    virtual uint32_t DecodeFaultCode(const uint8_t *data, uint32_t length) const;

    /**
     * @brief 由子类解析温度字段
     */
    virtual void DecodeTemperature(const uint8_t *data, uint32_t length) = 0;

    /**
     * @brief 发送特殊命令帧，尾字节由调用者指定
     */
    bool SendSpecialFrame(uint8_t tail_code);

    /**
     * @brief 解析 MIT 反馈中的公共字段
     */
    void UpdateMITState(const uint8_t *data);

    // 电机状态
    MWL_Motor_State_t state_;
    // 绑定的 FDCAN 管理对象
    FDCAN_Manage_Object_t *FDCAN_Manage_Object_;
    // 主机ID 用于区分不同主机（默认FD 若只有一块MCU 不需进行区分）
    uint16_t master_id_;
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
