/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Motor_Rx.h
  * @brief      电机接收与总线管理头文件
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 以物理 CAN 总线为核心组织电机设备
  *             2. 每条总线维护自己的设备对象注册表
  *             3. 回调收到一帧后，先判断“这是 CAN 几”，再逐个询问设备“这帧是不是你的”
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2026     Codex           1. reconstruct by physical can bus
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "HDL_DJI_Motor.h"
#include "HDL_Damiao_Motor.h"
#include "HDL_RS_Motor.h"
#include "HDL_Motor_Bus.h"

/**
 * @brief 宏定义
 */
#define HDL_MOTOR_RX_CAN_BUS_NUM 3U

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

class Class_Motor_Rx
{
public:
    /**
     * @brief 物理 CAN 总线编号
     */
    typedef enum
    {
        MOTOR_CAN_BUS_1 = 0,
        MOTOR_CAN_BUS_2,
        MOTOR_CAN_BUS_3,
        MOTOR_CAN_BUS_NUM = HDL_MOTOR_RX_CAN_BUS_NUM,
    } Enum_Motor_CAN_Bus_ID;

    /**
     * @brief 当前工程支持的物理 CAN 总线数量
     */
    static const uint8_t BUS_CANNum = HDL_MOTOR_RX_CAN_BUS_NUM;

    /**
     * @brief 构造函数
     */
    Class_Motor_Rx();

    /**
     * @brief 绑定物理 CAN 总线
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     *            hfdcan           FDCAN 句柄
     *            rx_fifo          默认接收 FIFO
     * @retval none
     */
    void BindBus(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo);

    /**
     * @brief 清空指定物理总线的设备注册表
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     * @retval none
     */
    void ClearBusDeviceRegistry(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID);

    /**
     * @brief 向指定物理总线挂载一个通用设备对象
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     *            Motor_Device    设备对象指针
     * @retval true  挂载成功
     *         false 挂载失败
     */
    bool AttachDevice(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_Motor_Device_Base *Motor_Device);

    /**
     * @brief 向指定物理总线挂载一个 RS 电机对象
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     *            RS_Motor        RS 电机对象
     *            can_id          电机 CAN ID
     *            model           电机具体型号（RS03/EL05 等）
     *            master_id       主控接收 ID
     * @retval true  挂载成功
     *         false 挂载失败
     */
    bool AttachRSMotor(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_RS_Motor *RS_Motor, uint8_t can_id,
                       MWL_Motor_Model_e model = MWL_MOTOR_MODEL_RS03,
                       uint16_t master_id = Class_MIT_Motor_Base::kDefaultMasterId);

    /**
     * @brief 向指定物理总线挂载一个达妙电机对象
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     *            Damiao_Motor    达妙电机对象
     *            can_id          电机 CAN ID
     *            model           电机具体型号（当前默认 DM4310）
     *            master_id       主控接收 ID
     * @retval true  挂载成功
     *         false 挂载失败
     */
    bool AttachDamiaoMotor(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_Damiao_Motor *Damiao_Motor, uint8_t can_id,
                           MWL_Motor_Model_e model = MWL_MOTOR_MODEL_DM4310,
                           uint16_t master_id = Class_MIT_Motor_Base::kDefaultMasterId);

    /**
     * @brief 向指定物理总线挂载一个 DJI 电机组对象
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     *            DJI_Motor_Group DJI 电机组对象
     *            command_id      该组电流命令 ID
     * @retval true  挂载成功
     *         false 挂载失败
     */
    bool AttachDJIGroup(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_DJI_Motor_Group *DJI_Motor_Group,
                        uint16_t command_id);

    /**
     * @brief 从指定物理总线的 FIFO 读取一帧并分发
     * @param[in] hfdcan 触发中断的 FDCAN 句柄
     *            fifo   当前接收 FIFO
     * @retval true  当前帧已被处理
     *         false 当前帧未被处理
     */
    bool DispatchRx(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);

    /**
     * @brief 对已读取的一帧 CAN 数据做分发
     * @param[in] hfdcan 触发中断的 FDCAN 句柄
     *            header FDCAN 接收头
     *            data   数据区
     *            length 数据长度
     * @retval true  当前帧已被处理
     *         false 当前帧未被处理
     */
    bool DispatchRx(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef &header, const uint8_t *data,
                    uint32_t length);

    /**
     * @brief 获取指定物理总线对象
     * @param[in] Motor_CAN_Bus_ID 物理 CAN 总线编号
     * @retval 总线对象指针
     */
    Class_Motor_Bus *GetBus(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID);

private:
    /**
     * @brief 根据总线编号解析总线对象
     */
    Class_Motor_Bus *ResolveBus(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID);

    /**
     * @brief 根据 FDCAN 句柄解析总线对象
     */
    Class_Motor_Bus *ResolveBus(FDCAN_HandleTypeDef *hfdcan);

    Class_Motor_Bus motor_bus_[BUS_CANNum];   /*!< 三条物理电机总线对象 */
};

extern Class_Motor_Rx Motor_Rx;

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
