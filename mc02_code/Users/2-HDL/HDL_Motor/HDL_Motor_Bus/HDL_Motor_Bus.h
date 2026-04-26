/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Motor_Bus.h
  * @brief      物理 CAN 电机总线头文件
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 一个总线对象对应一条物理 FDCAN 总线
  *             2. 总线内部维护“挂载到本总线上的设备对象列表”
  *             3. 接收分发时逐个询问设备对象“这帧是不是你的”
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#pragma once

/**
 * @brief 头文件
 */
#include "HAL_FDCAN.h"
#include "HDL_Motor_Device_Base.h"

/**
 * @brief 宏定义
 */
#define HDL_MOTOR_BUS_MAX_DEVICE_NUM 8U

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

class Class_Motor_Bus
{
public:
    /**
     * @brief 单条 CAN 总线上最多挂载的设备对象数量
     */
    static const uint8_t BUS_DeviceNum = HDL_MOTOR_BUS_MAX_DEVICE_NUM;

    /**
     * @brief 构造函数
     */
    Class_Motor_Bus();

    /**
     * @brief 绑定物理 FDCAN 总线
     * @param[in] hfdcan  FDCAN 句柄
     *            rx_fifo 默认接收 FIFO
     * @retval none
     */
    void BindBus(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo);

    /**
     * @brief 清空本总线的设备注册表
     * @retval none
     */
    void ClearDeviceRegistry();

    /**
     * @brief 向本总线挂载一个设备对象
     * @param[in] Motor_Device 设备对象指针
     * @retval true  挂载成功
     *         false 挂载失败
     */
    bool AttachDevice(Class_Motor_Device_Base *Motor_Device);

    /**
     * @brief 从指定 FIFO 读取一帧并分发
     * @param[in] fifo 当前读取 FIFO
     * @retval true  当前帧已被处理
     *         false 当前帧未被处理
     */
    bool DispatchRx(uint32_t fifo);

    /**
     * @brief 对已读取的一帧 CAN 数据做分发
     * @param[in] header FDCAN 接收头
     *            data   数据区
     *            length 数据长度
     * @retval true  当前帧已被处理
     *         false 当前帧未被处理
     */
    bool DispatchRx(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length);

    /**
     * @brief 判断当前 FDCAN 句柄是否属于本总线
     * @param[in] hfdcan FDCAN 句柄
     * @retval true  属于本总线
     *         false 不属于本总线
     */
    bool IsThisBus(FDCAN_HandleTypeDef *hfdcan) const;

    /**
     * @brief 获取本总线的 FDCAN 管理对象
     * @retval FDCAN 管理对象指针
     */
    FDCAN_Manage_Object_t *GetBusManageObject();

private:
    FDCAN_Manage_Object_t fdcan_manage_object_;                 /*!< 本总线绑定的 FDCAN 管理对象 */
    Class_Motor_Device_Base *motor_device_[BUS_DeviceNum];      /*!< 本总线设备对象注册表 */
    uint8_t motor_device_num_;                                  /*!< 当前已挂载设备数量 */
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
