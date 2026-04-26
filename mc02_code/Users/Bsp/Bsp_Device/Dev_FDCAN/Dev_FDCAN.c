/**
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  * @file       Dev_CAN_Receive.c
  * @brief      本文件中封装了CAN发送函数与CAN接收函数，包括电机、超电测量数据结构体
  * @note       在工程机器人框架_         FDCAN1:底盘DJI电机         3508*4
  *                                    FDCAN2:大关节XIAO_MI电机    root*1  shoulder*1  elbow*1
  *                                    FDCAN3:小关节DJI电机        wrist*3
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-28-2024     Light           1. done
  *
  @verbatim
  ==============================================================================
  *本文件编写参考了达妙mc02开发板与大疆c板开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  */

#include "Dev_FDCAN.h"
#include "App_Detect_Task.h"

/**
 * @brief 获取电机测量数据
 * @param ptr 指向电机测量数据结构体的指针
 * @param data 接收到的FDCAN数据
 */
#define GET_MOTOR_MEASURE(ptr, data)                                \
  {                                                                 \
    (ptr)->last_ecd = (ptr)->ecd;                                   \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
    (ptr)->temperate = (data)[6];                                   \
  }

/**
 * @brief 从CAN数据中获取超级电容器的测量值
 * @param ptr 指向超级电容器测量值结构体的指针
 * @param data CAN数据数组
 */
#define get_supercap_measure(ptr, data)                                  \
  {                                                                      \
    (ptr)->battery_voltage = (uint16_t)((data)[0] << 8 | (data)[1]);     \
    (ptr)->capacitance_voltage = (uint16_t)((data)[2] << 8 | (data)[3]); \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);       \
    (ptr)->capacitance_percentage = (data)[6];                           \
    (ptr)->power_set = (data)[7];                                        \
  }

/* 结构体数组存储电机数据 */
DJI_Motor_Measure_t motor_chassis[4]; // 底盘电机
MIT_Motor_Measure_t motor_l_joint[3]; // 大关节电机
DJI_Motor_Measure_t motor_s_joint[3]; // 小关节电机
SupCap_Measure_t    sup_cap[1];       // 超级电容

/* 底盘FDCAN发送变量 */
static FDCAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t                chassis_fdcan_send_data[8];

/* 大关节FDCAN发送变量 */
static FDCAN_TxHeaderTypeDef  l_joint_tx_message;
//static uint8_t                l_joint_fdcan_send_data[6];

/* 小关节FDCAN发送变量 */
static FDCAN_TxHeaderTypeDef  s_joint_tx_message;
static uint8_t                s_joint_fdcan_send_data[8];

/* 超电FDCAN发送变量 */
static FDCAN_TxHeaderTypeDef  sup_tx_message;
static uint8_t                sup_can_fdcan_send_data[2];


int32_t float_to_uint(float x_float, float x_min, float x_max, uint32_t bits);
float uint_to_float(uint32_t x_int, float x_min, float x_max, uint32_t bits);

/******************************* @FDCAN总线接收数据 获取被控对象数据 *******************************/

/**
************************************************************************
* @brief:      	获取小米电机反馈数据函数
* @param[in]:   motor:    指向motor_t结构的指针，包含电机相关信息和反馈数据
* @param[in]:   rx_data:  指向包含反馈数据的数组指针
* @param[in]:   data_len: 数据长度
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括电机ID、
*               状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void MI_Motor_Feedback_Data(MIT_Motor_Measure_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if(data_len == FDCAN_DLC_BYTES_8)
    {
        motor->id = (rx_data[0]) & 0x0F;
        motor->p_int = (rx_data[1] << 8) | rx_data[2];
        motor->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
        motor->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
        motor->pos = uint_to_float(motor->p_int, P_MIN, P_MAX, 16);   //(-12.5,12.5)
        motor->vel = uint_to_float(motor->v_int, V_MIN, V_MAX, 12);   //(-30.0,30.0)
        // 根据罗队长所言，电流和扭矩转化系数是0.089
        motor->tor = uint_to_float(motor->t_int, T_MIN, T_MAX, 12);   //(-10.0,10.0)
    }
}

/**
  * @brief          此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
  * @param[in]      hfdcan     FDCAN句柄指针
  * @retval         none
  */
void FDCAN_FIFOx_Callback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == &CHASSIS_CAN)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

        switch (rx_header.Identifier)
        {
            case FDCAN_3508_M1_ID:
            case FDCAN_3508_M2_ID:
            case FDCAN_3508_M3_ID:
            case FDCAN_3508_M4_ID:
            {
                static uint8_t i = 0;
                i = rx_header.Identifier - FDCAN_3508_M1_ID;
                GET_MOTOR_MEASURE(&motor_chassis[i], rx_data);

                detect_hook(CHASSIS_MOTOR1_TOE + i);

                break;
            }
            default:
            {
                break;
            }
        }
    }
    else if (hfdcan == &Large_Joint_CAN)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        static uint8_t rx_data[8];
        memset(rx_data, 0, sizeof(rx_data));

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);

        uint8_t rx_can_id = rx_data[0];
        switch (rx_can_id)
        {
            case FDCAN_LARGE_JOINT_1_ID:
            case FDCAN_LARGE_JOINT_2_ID:
            case FDCAN_LARGE_JOINT_3_ID:
            {
                static uint8_t i = 0;
                i = rx_can_id - FDCAN_LARGE_JOINT_1_ID;
                MI_Motor_Feedback_Data(&motor_l_joint[i], rx_data, rx_header.DataLength);

                detect_hook(GIMBAL_YAW_MOTOR_TOE + i);

                break;
            }
            default:
            {
                break;
            }
        }
    }
    else if (hfdcan == &Small_Joint_CAN)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);

        switch (rx_header.Identifier)
        {
            case FDCAN_SMALL_JOINT_1_ID:
            case FDCAN_SMALL_JOINT_2_ID:
            case FDCAN_SMALL_JOINT_3_ID:
            {
                static uint8_t i = 0;
                i = rx_header.Identifier - FDCAN_SMALL_JOINT_1_ID;
                GET_MOTOR_MEASURE(&motor_s_joint[i], rx_data);

                detect_hook(S_JOINT_MOTOR3_TOE + i);

                break;
            }
            default:
            {
                break;
            }
        }
    }
    else
    {
        Error_Handler();
    }
}

/***************************************** @FDCAN总线发送数据 控制被控对象 *****************************************/

/**
  * @brief          Joint1总是飘移，方案：初始化设置零点+寻找机械限位
  * @param[in]      none
  * @retval         none
  */
static void Large_Joint1_Init()
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;

    /* 主菜单 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFD;
        FDCAN_Send_Data(&Large_Joint_CAN, 002, data, 8);
        HAL_Delay(2);
    }

    /* 标定零点 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFE;
        FDCAN_Send_Data(&Large_Joint_CAN, 002, data, 8);
        HAL_Delay(2);
    }

    /* 主菜单 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFD;
        FDCAN_Send_Data(&Large_Joint_CAN, 002, data, 8);
        HAL_Delay(2);
    }

    /* 马达 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFC;
        FDCAN_Send_Data(&Large_Joint_CAN, 002, data, 8);
        HAL_Delay(2);
    }
}

/**
  * @brief          Joint1总是飘移，方案：初始化设置零点+寻找机械限位
  * @param[in]      none
  * @retval         none
  */
static void Large_Joint2_Init()
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;

    /* 主菜单 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFD;
        FDCAN_Send_Data(&Large_Joint_CAN, 003, data, 8);
        HAL_Delay(2);
    }

    /* 标定零点 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFE;
        FDCAN_Send_Data(&Large_Joint_CAN, 003, data, 8);
        HAL_Delay(2);
    }

    /* 主菜单 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFD;
        FDCAN_Send_Data(&Large_Joint_CAN, 003, data, 8);
        HAL_Delay(2);
    }

    /* 马达 */
    for (uint16_t i = 0; i < 20; ++i)
    {
        data[7] = 0xFC;
        FDCAN_Send_Data(&Large_Joint_CAN, 003, data, 8);
        HAL_Delay(2);
    }
}

/**                             小米电机
************************************************************************
* @brief:      	初始化电机 上电之后进入主菜单模式 电机只有在主菜单才会主动发送CAN消息
* @param[in]:   none
* @retval:     	void
************************************************************************
**/
void MIT_Motor_Init(void)
{
    uint8_t data[8];

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;

    /* 防止上电时间差导致关节电机不运行 */
    HAL_Delay(2500);

    Large_Joint1_Init();
    Large_Joint2_Init();

    for (uint16_t i = 0; i < 60; i++)
    {
        FDCAN_Send_Data(&Large_Joint_CAN, 001, data, 8);
        HAL_Delay(2);
    }


}

/**                             小米电机
************************************************************************
* @brief:      	Awake_MIT_Motor: 回复一帧CAN消息
* @param[in]:   hfdcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id:   电机ID，指定目标电机
* @retval:     	void
* @details:    	空命令，电机收到此消息不进行任何操作，但会
************************************************************************
**/
void Awake_MIT_Motor(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id)
{
    const unsigned char cmd_nothing[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0x01};
    FDCAN_Send_Data(hfdcan, motor_id, cmd_nothing, 8);
}

/**                             小米电机
************************************************************************
* @brief:      	enable_motor_mode: 开启电机模式函数，进入马达模式
* @param[in]:   hfdcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id:   电机ID，指定目标电机
* @param[in]:   mode_id:    模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送开启特定模式的命令
************************************************************************
**/
void Enable_MIT_Motor_Mode(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC; // 进入马达模式

    FDCAN_Send_Data(hfdcan, id, data, 8);
}

/**                             小米电机
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数,回到主菜单模式
 *              主菜单一直发送同一个数据回来（也许是电机初次上电时的数据）
* @param[in]:   hfdcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id:   电机ID，指定目标电机
* @param[in]:   mode_id:    模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void Disable_MIT_Motor_Mode(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t mode_id)
{
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD; // 退出当前状态，进入主菜单

    FDCAN_Send_Data(hfdcan, id, data, 8);
}

/**                             小米电机
************************************************************************
* @brief:      	MIT模式下的电机控制函数
* @param[in]:   hfdcan:			    FDCAN_HandleTypeDef，用于指定CAN总线
* @param[in]:   motor_id:	        电机ID，指定目标电机
* @param[in]:   pos:			    位置给定值
* @param[in]:   vel:			    速度给定值
* @param[in]:   kp:				    位置比例系数
* @param[in]:   kd:				    位置微分系数
* @param[in]:   torq:			    转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void MIT_Ctrl_L_Joint(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp  = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    FDCAN_Send_Data(hfdcan, id, data, 8);
}

/**                             小米电机
************************************************************************
* @brief:      	位置速度控制函数
* @param[in]:   hfdcan:			指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:	    电机ID，指定目标电机
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void Pos_Speed_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel)
{
    uint16_t id;
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    id = motor_id + POS_MODE;
    pbuf=(uint8_t *) & pos;
    vbuf=(uint8_t *) & vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);

    FDCAN_Send_Data(hfdcan, id, data, 8);
}

/**                             小米电机
************************************************************************
* @brief:      	速度控制函数
* @param[in]:   hfdcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:     电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void Speed_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float vel)
{
    uint16_t id;
    uint8_t *vbuf;
    uint8_t  data[4];

    id = motor_id + SPEED_MODE;
    vbuf=(uint8_t *) & vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    FDCAN_Send_Data(hfdcan, id, data, 4);
}

/**                             小米电机
************************************************************************
* @brief:      	设置当前位置为编码器零点
* @param[in]:   hfdcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:       电机ID，指定目标电机
* @retval:     	void
* @details:    	通过CAN总线向电机发送设置零点命令
************************************************************************
**/
void MIT_Motor_Set_Zero(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id)
{
    const uint8_t cmd_zero[8] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xfe};
    FDCAN_Send_Data(hfdcan, motor_id, cmd_zero, 8);
}

/**                 DJI电机
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
void FDCAN_Cmd_Chassis_Reset_ID(void)
{
    chassis_tx_message.Identifier  = FDCAN_CHASSIS_RESET_ID;
    chassis_tx_message.IdType      = FDCAN_STANDARD_ID;
    chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message.DataLength  = 0x08;

    chassis_fdcan_send_data[0] = 0;
    chassis_fdcan_send_data[1] = 0;
    chassis_fdcan_send_data[2] = 0;
    chassis_fdcan_send_data[3] = 0;
    chassis_fdcan_send_data[4] = 0;
    chassis_fdcan_send_data[5] = 0;
    chassis_fdcan_send_data[6] = 0;
    chassis_fdcan_send_data[7] = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&CHASSIS_CAN, &chassis_tx_message, chassis_fdcan_send_data);
}


/**                 DJI电机
  * @brief          发送底盘电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      dji_motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      dji_motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      dji_motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      dji_motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
void FDCAN_cmd_Chassis(int16_t dji_motor1, int16_t dji_motor2, int16_t dji_motor3, int16_t dji_motor4)
{
    chassis_tx_message.Identifier  = FDCAN_CHASSIS_ALL_ID;
    chassis_tx_message.IdType      = FDCAN_STANDARD_ID;
    chassis_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    chassis_tx_message.DataLength  = 0x08;

    chassis_fdcan_send_data[0] = dji_motor1 >> 8;
    chassis_fdcan_send_data[1] = dji_motor1;
    chassis_fdcan_send_data[2] = dji_motor2 >> 8;
    chassis_fdcan_send_data[3] = dji_motor2;
    chassis_fdcan_send_data[4] = dji_motor3 >> 8;
    chassis_fdcan_send_data[5] = dji_motor3;
    chassis_fdcan_send_data[6] = dji_motor4 >> 8;
    chassis_fdcan_send_data[7] = dji_motor4;

    HAL_FDCAN_AddMessageToTxFifoQ(&CHASSIS_CAN, &chassis_tx_message, chassis_fdcan_send_data);
}

/**                 DJI电机
  * @brief          发送电机控制电流(0x205,0x206,0x207)
  * @param[in]      wrist_6020:   (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      wrist_2006_l: (0x206) 6020电机控制电流, 范围 [-10000,10000]
  * @param[in]      wrist_2006_r: (0x207) 3508电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
void FDCAN_cmd_Small_Joint(int16_t wrist_6020 ,int16_t wrist_2006_l, int16_t wrist_2006_r)
{
    s_joint_tx_message.Identifier = FDCAN_SMALL_JOINT_ALL_ID;
    s_joint_tx_message.IdType = FDCAN_STANDARD_ID;
    s_joint_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    s_joint_tx_message.DataLength = 0x08;

    s_joint_fdcan_send_data[0] = wrist_6020 >> 8;
    s_joint_fdcan_send_data[1] = wrist_6020;
    s_joint_fdcan_send_data[2] = wrist_2006_l >> 8;
    s_joint_fdcan_send_data[3] = wrist_2006_l;
    s_joint_fdcan_send_data[4] = wrist_2006_r >> 8;
    s_joint_fdcan_send_data[5] = wrist_2006_r;
    s_joint_fdcan_send_data[6] = 0>>8;
    s_joint_fdcan_send_data[7] = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&Small_Joint_CAN, &s_joint_tx_message, s_joint_fdcan_send_data);
}

/**             超级电容
 * @brief       发送CAN命令以控制超级电容器电源。
 *              此函数设置发送CAN消息以控制超级电容器电源所需的参数。
 *              它配置CAN消息的ID、格式、类型、数据长度和数据负载。
 *              power_set参数指定超级电容器的期望功率级别。
 * @param       power_set 要设置给超级电容器的功率级别，单位0.01W，限额30-120W。
 * @retval      none
 */
void FDCAN_cmd_SuperCap(int16_t power_set)
{
    power_set = power_set *100;

    sup_tx_message.Identifier  = FDCAN_SUPERCAP_TX_ID;
    sup_tx_message.IdType      = FDCAN_STANDARD_ID;
    sup_tx_message.TxFrameType = FDCAN_DATA_FRAME;
    sup_tx_message.DataLength  = 0x02;

    sup_can_fdcan_send_data[0] = power_set >> 8;
    sup_can_fdcan_send_data[1] = power_set;

    HAL_FDCAN_AddMessageToTxFifoQ(&CHASSIS_CAN, &sup_tx_message, sup_can_fdcan_send_data);
}



/******************************* @指针函数 便于外部调用电机数据时解耦 *******************************/

/**
  * @brief          返回底盘电机 3508 * 4 电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const DJI_Motor_Measure_t *Get_Chassis_Motor_Measure_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          返回大关节电机 小米电机数据指针
  * @param[in]      i: 电机编号,范围[0,2]
  * @retval         电机数据指针
  */
const MIT_Motor_Measure_t *Get_Large_Joint_Motor_Measure_Point(uint8_t i )
{
    return &motor_l_joint[(i & 0x03)];
}

/**
  * @brief          返回小关节电机 6020 & 2006 电机数据指针
  * @param[in]      i: 电机编号,范围[0,2]
  * @retval         电机数据指针
  */
const DJI_Motor_Measure_t *Get_Small_Joint_Motor_Measure_Point(uint8_t i )
{
    return &motor_s_joint[(i & 0x03)];
}



/**
 * @brief           获取超级电容器的测量数据
 * @param[in]       none
 * @return          超级电容器的测量数据指针
 */
const SupCap_Measure_t *Get_Supercap_Measure_Point(void)
{
  return &sup_cap[0];
}

/******************************* @变量类型转换函数 用于MIT下发指令时将进行转换 *******************************/

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
int32_t float_to_uint(float x_float, float x_min, float x_max, uint32_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int32_t) ((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
float uint_to_float(uint32_t x_int, float x_min, float x_max, uint32_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

float Hex_To_Float(uint32_t *Byte, uint32_t num)  //十六进制到浮点数
{
    return *((float *)Byte);
}

uint32_t FloatTohex(float HEX)  //浮点数到十六进制转换
{
    return *( uint32_t *)&HEX;
}
