/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Motor_Bus.cpp
  * @brief      物理 CAN 电机总线实现
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 一个总线对象只关心自己绑定的是哪条物理 CAN 总线
  *             2. 接收到一帧数据后，逐个询问挂载设备“这帧是不是你的”
  *             3. 任意品牌电机只要实现统一设备接口，即可混挂到同一总线
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2026     Codex           1. create
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "HDL_Motor_Bus.h"

#include "MWL_Motor.h"

#include <string.h>

/**
  * @brief          构造函数
  * @retval         none
  */
Class_Motor_Bus::Class_Motor_Bus()
    : motor_device_num_(0U)
{
    memset(&fdcan_manage_object_, 0, sizeof(fdcan_manage_object_));
    memset(motor_device_, 0, sizeof(motor_device_));
}

/**
  * @brief          绑定物理 FDCAN 总线
  * @param[in]      hfdcan  FDCAN 句柄
  *                 rx_fifo 默认接收 FIFO
  * @retval         none
  */
void Class_Motor_Bus::BindBus(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo)
{
    FDCAN_Init(&fdcan_manage_object_, hfdcan, rx_fifo);
}

/**
  * @brief          清空本总线的设备注册表
  * @retval         none
  */
void Class_Motor_Bus::ClearDeviceRegistry()
{
    memset(motor_device_, 0, sizeof(motor_device_));
    motor_device_num_ = 0U;
}

/**
  * @brief          向本总线挂载一个设备对象
  * @param[in]      Motor_Device 设备对象指针
  * @retval         true  挂载成功
  *                 false 挂载失败
  */
bool Class_Motor_Bus::AttachDevice(Class_Motor_Device_Base *Motor_Device)
{
    if (Motor_Device == NULL || motor_device_num_ >= BUS_DeviceNum)
    {
        return false;
    }

    motor_device_[motor_device_num_] = Motor_Device;
    motor_device_num_++;

    return true;
}

/**
  * @brief          从指定 FIFO 读取一帧并分发
  * @param[in]      fifo 当前读取 FIFO
  * @retval         true  当前帧已被处理
  *                 false 当前帧未被处理
  */
bool Class_Motor_Bus::DispatchRx(uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef header = {};
    uint8_t rx_data[8] = {};

    if (!FDCAN_Read_Data_From_FIFO(&fdcan_manage_object_, fifo, &header, rx_data))
    {
        return false;
    }

    return DispatchRx(header, rx_data, MWL_Motor_Dlc_To_Length(header.DataLength));
}

/**
  * @brief          对已读取的一帧 CAN 数据做分发
  * @param[in]      header FDCAN 接收头
  *                 data   数据区
  *                 length 数据长度
  * @retval         true  当前帧已被处理
  *                 false 当前帧未被处理
  * @note           分发逻辑只关心“挂载顺序”和“设备对象是否认领此帧”
  */
bool Class_Motor_Bus::DispatchRx(const FDCAN_RxHeaderTypeDef &header, const uint8_t *data, uint32_t length)
{
    uint8_t i = 0U;

    for (i = 0U; i < motor_device_num_; i++)
    {
        if (motor_device_[i] != NULL && motor_device_[i]->ParseFeedback(header, data, length))
        {
            return true;
        }
    }

    return false;
}

/**
  * @brief          判断当前 FDCAN 句柄是否属于本总线
  * @param[in]      hfdcan FDCAN 句柄
  * @retval         true  属于本总线
  *                 false 不属于本总线
  */
bool Class_Motor_Bus::IsThisBus(FDCAN_HandleTypeDef *hfdcan) const
{
    return (fdcan_manage_object_.hfdcan == hfdcan);
}

/**
  * @brief          获取本总线的 FDCAN 管理对象
  * @retval         FDCAN 管理对象指针
  */
FDCAN_Manage_Object_t *Class_Motor_Bus::GetBusManageObject()
{
    return &fdcan_manage_object_;
}
