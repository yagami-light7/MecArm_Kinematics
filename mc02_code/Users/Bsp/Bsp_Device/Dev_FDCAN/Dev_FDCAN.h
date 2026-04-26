#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "main.h"
#include "string.h"
#include "HAL_FDCAN.h"
#include "cmsis_os.h"
#include "Alg_UserLib.h"

/*******************************CAN总线分配*******************************/
#define CHASSIS_CAN      hfdcan1
#define Large_Joint_CAN  hfdcan2
#define Small_Joint_CAN  hfdcan3

/*******************************小米关节电机宏定义*******************************/

#define MIT_MODE 0x000 // 马达模式
#define POS_MODE 0x100
#define SPEED_MODE 0x200

#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -20.5f
#define V_MAX   20.5f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN  -32.0f
#define T_MAX   32.0f

/*******************************CAN总线发送和接收ID*******************************/
typedef enum
{
    /* 在FDCAN_cmd_Chassis函数中被分配，使用0x700重置所有底盘电机的ID，使用hfdcan1发送 */
    FDCAN_CHASSIS_RESET_ID = 0x700,

    /* 在FDCAN_cmd_Chassis函数中被分配，使用0x200作为所有底盘电机的ID，使用hfdcan1发送 */
    FDCAN_CHASSIS_ALL_ID  = 0x200,  // 底盘所有电机的ID
    FDCAN_3508_M1_ID      = 0x201,
    FDCAN_3508_M2_ID      = 0x202,
    FDCAN_3508_M3_ID      = 0x203,
    FDCAN_3508_M4_ID      = 0x204,

    /* FDCAN_cmd_Joint_L(Large)中被分配，使用0x1ff作为所有大关节电机的ID，使用hfdcan2发送 */
    FDCAN_LARGE_JOINT_ALL_ID = 0x000, // 大关节(root/shoulder/elbow)所有电机的ID
    FDCAN_LARGE_JOINT_1_ID   = 0x001, // ID为1，回传0x001的电机数据(MI电机)
    FDCAN_LARGE_JOINT_2_ID   = 0x002, // ID为2，回传0x002的电机数据(MI电机)
    FDCAN_LARGE_JOINT_3_ID   = 0x003, // ID为3，回传0x003的电机数据(MI电机)

    /* FDCAN_cmd_Joint_S(Small)中被分配，使用0x1ff作为所有小关节电机的ID，使用hfdcan3发送 */
    FDCAN_SMALL_JOINT_ALL_ID = 0x1FF, // 小关节(wrist)所有DJI电机的ID
    FDCAN_SMALL_JOINT_1_ID   = 0x205, // ID为1，回传0x205的电机数据(6020)
    FDCAN_SMALL_JOINT_2_ID   = 0x206, // ID为2，回传0x206的电机数据(左侧2006)
    FDCAN_SMALL_JOINT_3_ID   = 0x207, // ID为3，回传0x207的电机数据(右侧2006)

    /* 在FDCAN_cmd_SuperCap函数中被分配，使用0x210作为超级电容的发送ID */
    FDCAN_SUPERCAP_TX_ID = 0x210,  //**>
    FDCAN_SUPERCAP_RX_ID = 0x211,  //<**
}can_msg_id_e;


/*******************************小米电机回传数据*******************************/
typedef enum
{
    MENU_MODE, // 菜单模式 FD 主动回传数据
    MADA_MODE, // 马达模式 FC 一收一发数据

}MIT_Motor_Mode; // 小米电机当前模式

typedef struct
{
    uint8_t id;
    uint8_t err_state;
    int32_t p_int;
    int32_t v_int;
    int32_t t_int;
    int32_t kp_int;
    int32_t kd_int;
    float pos; // 位置
    float vel; // 速度
    float tor; // 电流（扭矩）
    float Kp;
    float Kd;
    float Temp;
}MIT_Motor_Measure_t;

/*******************************DJI电机回传数据*******************************/
typedef struct
{
    uint16_t ecd;
    int16_t  speed_rpm;  // 原始速度数据 换算后为Rpm
    int16_t  given_current;
    uint8_t  temperate;
    int16_t  last_ecd;
    float    totalAngle;  //累积总共角度
    int      turnCount;     //转过的圈数
}DJI_Motor_Measure_t;


/*******************************超级电容回传数据*******************************/
typedef struct
{
    uint16_t battery_voltage;
    int16_t  capacitance_voltage;
    int16_t  given_current;
    uint8_t  capacitance_percentage;
    uint8_t  power_set;
}SupCap_Measure_t;


extern DJI_Motor_Measure_t motor_chassis[4];
extern DJI_Motor_Measure_t motor_s_joint[3];
extern MIT_Motor_Measure_t motor_l_joint[3];
extern SupCap_Measure_t    sup_cap[1];



#ifdef __cplusplus
extern "C"{
#endif

extern void MIT_Motor_Init(void);

/**                             小米电机
************************************************************************
* @brief:      	Awake_MIT_Motor: 开启电机模式函数，进入马达模式
* @param[in]:   hfdcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id:   电机ID，指定目标电机
* @retval:     	void
* @details:    	空命令，电机收到此消息不进行任何操作，但会回复一帧CAN消息
************************************************************************
**/
extern void Awake_MIT_Motor(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id);

/**                             小米电机
************************************************************************
* @brief:      	enable_motor_mode: 开启电机模式函数
* @param[in]:   hfdcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id:   电机ID，指定目标电机
* @param[in]:   mode_id:    模式ID，指定要开启的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送开启特定模式的命令
************************************************************************
**/
extern void Enable_MIT_Motor_Mode(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id, uint16_t mode_id);

/**                             小米电机
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hfdcan:     指向FDCAN_HandleTypeDef结构的指针
* @param[in]:   motor_id:   电机ID，指定目标电机
* @param[in]:   mode_id:    模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
extern void Disable_MIT_Motor_Mode(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, uint16_t mode_id);


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
extern void MIT_Ctrl_L_Joint(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);

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
extern void Pos_Speed_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float pos, float vel);


/**                             小米电机
************************************************************************
* @brief:      	速度控制函数
* @param[in]:   hfdcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:       电机ID，指定目标电机
* @param[in]:   vel: 			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
extern void Speed_Ctrl(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id, float vel);

/**                             小米电机
************************************************************************
* @brief:      	设置当前位置为编码器零点
* @param[in]:   hfdcan: 		指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @param[in]:   motor_id:       电机ID，指定目标电机
* @retval:     	void
* @details:    	通过CAN总线向电机发送设置零点命令
************************************************************************
**/
extern void MIT_Motor_Set_Zero(FDCAN_HandleTypeDef* hfdcan, uint16_t motor_id);

/**                 DJI电机
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void FDCAN_Cmd_Chassis_Reset_ID(void);


/**                 DJI电机
  * @brief          发送底盘电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      dji_motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      dji_motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      dji_motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      dji_motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void FDCAN_cmd_Chassis(int16_t dji_motor1, int16_t dji_motor2, int16_t dji_motor3, int16_t dji_motor4);

/**                 DJI电机
  * @brief          发送电机控制电流(0x205,0x206,0x207)
  * @param[in]      wrist_6020:   (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      wrist_2006_l: (0x206) 6020电机控制电流, 范围 [-10000,10000]
  * @param[in]      wrist_2006_r: (0x207) 3508电机控制电流, 范围 [-10000,10000]
  * @retval         none
  */
extern void FDCAN_cmd_Small_Joint(int16_t wrist_6020 ,int16_t wrist_2006_l, int16_t wrist_2006_r);

/**             超级电容
 * @brief       发送CAN命令以控制超级电容器电源。
 *              此函数设置发送CAN消息以控制超级电容器电源所需的参数。
 *              它配置CAN消息的ID、格式、类型、数据长度和数据负载。
 *              power_set参数指定超级电容器的期望功率级别。
 * @param       power_set 要设置给超级电容器的功率级别，单位0.01W，限额30-120W。
 * @retval      none
 */
extern void FDCAN_cmd_SuperCap(int16_t power_set);

/******************************* @指针函数 便于外部调用电机数据时解耦 *******************************/

/**
  * @brief          返回底盘电机 3508 * 4 电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
extern const DJI_Motor_Measure_t *Get_Chassis_Motor_Measure_Point(uint8_t i);

/**
  * @brief          返回大关节电机 小米电机数据指针
  * @param[in]      i: 电机编号,范围[0,2]
  * @retval         电机数据指针
  */
extern const MIT_Motor_Measure_t *Get_Large_Joint_Motor_Measure_Point(uint8_t i );

/**
  * @brief          返回小关节电机 6020 & 2006 电机数据指针
  * @param[in]      i: 电机编号,范围[0,2]
  * @retval         电机数据指针
  */
extern const DJI_Motor_Measure_t *Get_Small_Joint_Motor_Measure_Point(uint8_t i );

/**
 * @brief           获取超级电容器的测量数据
 * @param[in]       none
 * @return          超级电容器的测量数据指针
 */
extern const SupCap_Measure_t *Get_Supercap_Measure_Point(void);

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
extern int32_t float_to_uint(float x_float, float x_min, float x_max, uint32_t bits);

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
extern float uint_to_float(uint32_t x_int, float x_min, float x_max, uint32_t bits);

extern float Hex_To_Float(uint32_t *Byte, uint32_t num);  //十六进制到浮点数

extern uint32_t FloatTohex(float HEX);  //浮点数到十六进制转换


#ifdef __cplusplus
}
#endif

#endif
