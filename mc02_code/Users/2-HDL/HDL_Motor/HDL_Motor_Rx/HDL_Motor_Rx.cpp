/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HDL_Motor_Rx.cpp
  * @brief      电机接收与总线管理实现
  * @note       Hardware Driver Layer 硬件驱动层
  *             1. 先根据 FDCAN 句柄判断“这是 CAN 几”
  *             2. 再把该帧交给对应物理总线对象
  *             3. 总线对象内部逐个询问挂载设备“这帧是不是你的”
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-12-2026     Codex           1. reconstruct by physical can bus
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */

/**
 * @brief 头文件
 */
#include "HDL_Motor_Rx.h"

/**
 * @brief 创建电机接收管理对象
 */
Class_Motor_Rx Motor_Rx;

/**
  * @brief          构造函数
  * @retval         none
  */
Class_Motor_Rx::Class_Motor_Rx()
{
}

/**
  * @brief          绑定物理 CAN 总线
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  *                 hfdcan           FDCAN 句柄
  *                 rx_fifo          默认接收 FIFO
  * @retval         none
  */
void Class_Motor_Rx::BindBus(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo)
{
    Class_Motor_Bus *motor_bus = ResolveBus(Motor_CAN_Bus_ID);

    if (motor_bus == NULL)
    {
        return;
    }

    motor_bus->BindBus(hfdcan, rx_fifo);
}

/**
  * @brief          清空指定物理总线的设备注册表
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  * @retval         none
  */
void Class_Motor_Rx::ClearBusDeviceRegistry(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID)
{
    Class_Motor_Bus *motor_bus = ResolveBus(Motor_CAN_Bus_ID);

    if (motor_bus == NULL)
    {
        return;
    }

    motor_bus->ClearDeviceRegistry();
}

/**
  * @brief          向指定物理总线挂载一个通用设备对象
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  *                 Motor_Device    设备对象指针
  * @retval         true  挂载成功
  *                 false 挂载失败
  */
bool Class_Motor_Rx::AttachDevice(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_Motor_Device_Base *Motor_Device)
{
    Class_Motor_Bus *motor_bus = ResolveBus(Motor_CAN_Bus_ID);

    if (motor_bus == NULL)
    {
        return false;
    }

    return motor_bus->AttachDevice(Motor_Device);
}

/**
  * @brief          向指定物理总线挂载一个 RS 电机对象
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  *                 RS_Motor        RS 电机对象
  *                 can_id          电机 CAN ID
  *                 master_id       主控接收 ID
  * @retval         true  挂载成功
  *                 false 挂载失败
  */
bool Class_Motor_Rx::AttachRSMotor(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_RS_Motor *RS_Motor, uint8_t can_id,
                                   MWL_Motor_Model_e model, uint16_t master_id)
{
    Class_Motor_Bus *motor_bus = ResolveBus(Motor_CAN_Bus_ID);

    if (motor_bus == NULL || RS_Motor == NULL)
    {
        return false;
    }

    RS_Motor->Init(motor_bus->GetBusManageObject(), can_id, master_id);
    RS_Motor->SetMotorModel(model);

    return motor_bus->AttachDevice(RS_Motor);
}

/**
  * @brief          向指定物理总线挂载一个达妙电机对象
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  *                 Damiao_Motor    达妙电机对象
  *                 can_id          电机 CAN ID
  *                 master_id       主控接收 ID
  * @retval         true  挂载成功
  *                 false 挂载失败
  */
bool Class_Motor_Rx::AttachDamiaoMotor(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_Damiao_Motor *Damiao_Motor,
                                       uint8_t can_id, MWL_Motor_Model_e model, uint16_t master_id)
{
    Class_Motor_Bus *motor_bus = ResolveBus(Motor_CAN_Bus_ID);

    if (motor_bus == NULL || Damiao_Motor == NULL)
    {
        return false;
    }

    Damiao_Motor->Init(motor_bus->GetBusManageObject(), can_id, master_id);
    Damiao_Motor->SetMotorModel(model);

    return motor_bus->AttachDevice(Damiao_Motor);
}

/**
  * @brief          向指定物理总线挂载一个 DJI 电机组对象
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  *                 DJI_Motor_Group DJI 电机组对象
  *                 command_id      该组电流命令 ID
  * @retval         true  挂载成功
  *                 false 挂载失败
  */
bool Class_Motor_Rx::AttachDJIGroup(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID, Class_DJI_Motor_Group *DJI_Motor_Group,
                                    uint16_t command_id)
{
    Class_Motor_Bus *motor_bus = ResolveBus(Motor_CAN_Bus_ID);

    if (motor_bus == NULL || DJI_Motor_Group == NULL)
    {
        return false;
    }

    DJI_Motor_Group->Init(motor_bus->GetBusManageObject(), command_id);

    return motor_bus->AttachDevice(DJI_Motor_Group);
}

/**
  * @brief          从指定物理总线的 FIFO 读取一帧并分发
  * @param[in]      hfdcan 触发中断的 FDCAN 句柄
  *                 fifo   当前接收 FIFO
  * @retval         true  当前帧已被处理
  *                 false 当前帧未被处理
  */
bool Class_Motor_Rx::DispatchRx(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef  header = {};
    uint8_t rx_data[8] = {};
    uint32_t length = 0;
    Class_Motor_Bus *motor_bus = NULL;

    if(hfdcan == NULL)
    {
        return false;
    }

    // 将当前这一帧从 FIFO 里读出来
    if (HAL_FDCAN_GetRxMessage(hfdcan, fifo, &header, rx_data) != HAL_OK)
    {
        return false;
    }

    length = MWL_Motor_Dlc_To_Length(header.DataLength);

    // 判断对应Bus
    motor_bus = ResolveBus(hfdcan);

    if (motor_bus == NULL)
    {
        return false; // 返回false 当前帧已经被读出（防止堵塞fifo）
    }

    // 已绑定总线，将这一帧交给Bus逐个分发
    return motor_bus->DispatchRx(header, rx_data, length);
}

/**
  * @brief          对已读取的一帧 CAN 数据做分发
  * @param[in]      hfdcan 触发中断的 FDCAN 句柄
  *                 header FDCAN 接收头
  *                 data   数据区
  *                 length 数据长度
  * @retval         true  当前帧已被处理
  *                 false 当前帧未被处理
  */
bool Class_Motor_Rx::DispatchRx(FDCAN_HandleTypeDef *hfdcan, const FDCAN_RxHeaderTypeDef &header, const uint8_t *data,
                                uint32_t length)
{
    Class_Motor_Bus *motor_bus = ResolveBus(hfdcan);

    if (motor_bus == NULL)
    {
        return false;
    }

    return motor_bus->DispatchRx(header, data, length);
}

/**
  * @brief          获取指定物理总线对象
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  * @retval         总线对象指针
  */
Class_Motor_Bus *Class_Motor_Rx::GetBus(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID)
{
    return ResolveBus(Motor_CAN_Bus_ID);
}

/**
  * @brief          根据总线编号解析总线对象
  * @param[in]      Motor_CAN_Bus_ID 物理 CAN 总线编号
  * @retval         总线对象指针
  */
Class_Motor_Bus *Class_Motor_Rx::ResolveBus(Enum_Motor_CAN_Bus_ID Motor_CAN_Bus_ID)
{
    if (Motor_CAN_Bus_ID >= MOTOR_CAN_BUS_NUM)
    {
        return NULL;
    }

    return &motor_bus_[Motor_CAN_Bus_ID];
}

/**
  * @brief          根据 FDCAN 句柄解析总线对象
  * @param[in]      hfdcan FDCAN 句柄
  * @retval         总线对象指针
  */
Class_Motor_Bus *Class_Motor_Rx::ResolveBus(FDCAN_HandleTypeDef *hfdcan)
{
    uint8_t i = 0U;

    for (i = 0U; i < BUS_CANNum; i++)
    {
        if (motor_bus_[i].IsThisBus(hfdcan))
        {
            return &motor_bus_[i];
        }
    }

    return NULL;
}
