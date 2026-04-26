/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c/h
  * @brief      校准2006电机，自定义控制器上电之后会自动进入校准模式，各关节记录
  *             角度限位值之后计算零点
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jan-18-2025     Light              1. done
  *
  @verbatim
  ==============================================================================
  *             使用遥控器进行开始校准
  *             第一步:遥控器的两个开关都打到下
  *             第二步:两个摇杆打成\../,保存两秒.\.代表左摇杆向右下打.
  *             第三步:摇杆打成./\. 开始陀螺仪校准
  *                    或者摇杆打成'\/' 开始云台校准
  *                    或者摇杆打成/''\ 开始底盘校准
  *
  *             数据在flash中，包括校准数据和名字 name[3] 和 校准标志位 cali_flag
  *             例如head_cali有八个字节,但它需要12字节在flash,如果它从0x080A0000开始
  *             0x080A0000-0x080A0007: head_cali数据
  *             0x080A0008: 名字name[0]
  *             0x080A0009: 名字name[1]
  *             0x080A000A: 名字name[2]
  *             0x080A000B: 校准标志位 cali_flag,当校准标志位为0x55,意味着head_cali已经校准了
  *             添加新设备
  *             1.添加设备名在calibrate_task.h的cali_id_e, 像
  *             typedef enum
  *             {
  *                 ...
  *                 //add more...
  *                 CALI_XXX,
  *                 CALI_LIST_LENGHT,
  *             } cali_id_e;
  *             2. 添加数据结构在 calibrate_task.h, 必须4字节倍数，像
  *
  *             typedef struct
  *             {
  *                 uint16_t xxx;
  *                 uint16_t yyy;
  *                 fp32 zzz;
  *             } xxx_cali_t; //长度:8字节 8 bytes, 必须是 4, 8, 12, 16...
  *             3.在 "FLASH_WRITE_BUF_LENGHT",添加"sizeof(xxx_cali_t)", 和实现新函数
  *             bool_t cali_xxx_hook(uint32_t *cali, bool_t cmd), 添加新名字在 "cali_name[CALI_LIST_LENGHT][3]"
  *             和申明变量 xxx_cali_t xxx_cail, 添加变量地址在cali_sensor_buf[CALI_LIST_LENGHT]
  *             在cali_sensor_size[CALI_LIST_LENGHT]添加数据长度, 最后在cali_hook_fun[CALI_LIST_LENGHT]添加函数
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "App_Calibrate_Task.h"
#include "string.h"
#include "cmsis_os.h"

#include "Board_ADC.h"
#include "Board_Buzzer.h"
#include "Board_Flash.h"
#include "Dev_FDCAN.h"
#include "Dev_Remote_Control.h"
#include "Dev_Custom.h"

#include "App_INS_Task.h"
#include "App_Gimbal_Task.h"
#include "App_Mechanical_Arm_Task.h"

//include head,gimbal,gyro,accel,mag. gyro,accel and mag have the same data struct. total 5(CALI_LIST_LENGHT) devices, need data lenght + 5 * 4 bytes(name[3]+cali)
//#define FLASH_WRITE_BUF_LENGHT  (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3  + CALI_LIST_LENGHT * 4)


/**
  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
  * @param[in]      none
  * @retval         none
  */
static void RC_cmd_to_calibrate(void);


/**
  * @brief          2006电机校准
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool cali_joint_motor_hook(bool cmd);


#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

joint_cali_t joint_cali;

//static const RC_Ctrl_t *calibrate_RC;   //remote control point
//static head_cali_t     head_cali;       //head cali data
//static gimbal_cali_t   gimbal_cali;     //gimbal cali data
//static imu_cali_t      accel_cali;      //accel cali data
//static imu_cali_t      gyro_cali;       //gyro cali data
//static imu_cali_t      mag_cali;        //mag cali data


//static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];

cali_sensor_t cali_sensor[CALI_LIST_LENGHT];

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"};

//cali data address
//static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] = {
//        (uint32_t *)&joint_cali.cali_motor[0].motor_data, (uint32_t *)&joint_cali.cali_motor[1].motor_data,
//        (uint32_t *)&joint_cali.cali_motor[2].motor_data, (uint32_t *)&joint_cali.cali_motor[3].motor_data};

//
//static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
//        {
//                sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
//                sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

void *cali_hook_fun = cali_joint_motor_hook;

static uint32_t calibrate_systemTick;

static bool skip_calibrate = 0;

/**
  * @brief          校准任务
  * @param[in]      pvParameters: 空
  * @retval         none
  */
void Calibrate_Task(void *pvParameters)
{
    /* 上电后延迟一段时间，等待其他数据初始化 */
    vTaskDelay(2500);

    /* 关节电机校准初始化 */
    Cali_Param_Init();

    while (1)
    {
        /* 对于其他机器人，此处应该由遥控器设置校准命令 */

        //RC_cmd_to_calibrate();
        // 由于2006电机上电之后编码值完全随机，所以一次上电必须执行一次校准记录角度

        /* 如果没有执行校准 */
        if(joint_cali.calibrate_finished_flag != CALIBRATE_FINISHED)
        {
            joint_cali.cali_motor.cali_hook(CALI_FUNC_CMD_ON);
        }

        osDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


///**
//  * @brief          使用遥控器开始校准，例如陀螺仪，云台，底盘
//  * @param[in]      none
//  * @retval         none
//  */
//static void RC_cmd_to_calibrate(void)
//{
//    static const uint8_t BEGIN_FLAG   = 1;
//    static const uint8_t GIMBAL_FLAG  = 2;
//    static const uint8_t GYRO_FLAG    = 3;
//    static const uint8_t CHASSIS_FLAG = 4;
//
//    static uint8_t  i;
//    static uint32_t rc_cmd_systemTick = 0;
//    static uint16_t buzzer_time       = 0;
//    static uint16_t rc_cmd_time       = 0;
//    static uint8_t  rc_action_flag    = 0;
//
//    //如果已经在校准，就返回
//    for (i = 0; i < CALI_LIST_LENGHT; i++)
//    {
//        if (cali_sensor[i].cali_cmd)
//        {
//            buzzer_time = 0;
//            rc_cmd_time = 0;
//            rc_action_flag = 0;
//
//            return;
//        }
//    }
//
//    if (rc_action_flag == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
//    {
//        rc_cmd_systemTick = xTaskGetTickCount();
//        rc_action_flag = BEGIN_FLAG;
//        rc_cmd_time = 0;
//    }
//    else if (rc_action_flag == GIMBAL_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
//    {
//        //gimbal cali,
//        rc_action_flag = 0;
//        rc_cmd_time = 0;
//        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
//        cali_buzzer_off();
//    }
//    else if (rc_action_flag == GYRO_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
//    {
//        //gyro cali
//        rc_action_flag = 0;
//        rc_cmd_time = 0;
//        cali_sensor[CALI_GYRO].cali_cmd = 1;
//        //update control temperature
//        head_cali.temperature = (int8_t)(cali_get_mcu_temperature()) + 10;
//        if (head_cali.temperature > (int8_t)(GYRO_CONST_MAX_TEMP))
//        {
//            head_cali.temperature = (int8_t)(GYRO_CONST_MAX_TEMP);
//        }
//        cali_buzzer_off();
//    }
//    else if (rc_action_flag == CHASSIS_FLAG && rc_cmd_time > RC_CMD_LONG_TIME)
//    {
//        rc_action_flag = 0;
//        rc_cmd_time = 0;
//        //发送CAN重设ID命令到3508
//        CAN_cmd_chassis_reset_ID();
//        CAN_cmd_chassis_reset_ID();
//        CAN_cmd_chassis_reset_ID();
//        cali_buzzer_off();
//    }
//
//    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag == 0)
//    {
//        //两个摇杆打成 \../,保持2s
//        rc_cmd_time++;
//    }
//    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
//    {
//        //两个摇杆打成'\/',保持2s
//        rc_cmd_time++;
//        rc_action_flag = GIMBAL_FLAG;
//    }
//    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
//    {
//        //两个摇杆打成./\.,保持2s
//        rc_cmd_time++;
//        rc_action_flag = GYRO_FLAG;
//    }
//    else if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_flag != 0)
//    {
//        //两个摇杆打成/''\,保持2s
//        rc_cmd_time++;
//        rc_action_flag = CHASSIS_FLAG;
//    }
//    else
//    {
//        rc_cmd_time = 0;
//    }
//
//    calibrate_systemTick = xTaskGetTickCount();
//
//    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
//    {
//        //超过20s,停止
//        rc_action_flag = 0;
//        return;
//    }
//    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_flag != 0)
//    {
//        rc_cali_buzzer_middle_on();
//    }
//    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_flag != 0)
//    {
//        rc_cali_buzzer_start_on();
//    }
//
//    if (rc_action_flag != 0)
//    {
//        buzzer_time++;
//    }
//
//    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_flag != 0)
//    {
//        buzzer_time = 0;
//    }
//    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_flag != 0)
//    {
//        cali_buzzer_off();
//    }
//}

/**
  * @brief          电机校准初始化
  * @param[in]      none
  * @retval         none
  */
void Cali_Param_Init(void)
{
    /* 上电默认进行校准 */
    joint_cali.calibrate_finished_flag = 0;

    /* 分配钩子函数 */
    joint_cali.cali_motor.cali_hook = (bool (*)(bool))cali_hook_fun;

    /* 读取flash */

    // 自定义控制器不需要

    /* 如果已经完成校准 */
    if(joint_cali.calibrate_finished_flag  == CALIBRATE_FINISHED)
    {
        if (joint_cali.cali_motor.cali_hook != NULL)
        {
            /* 上电之后默认没有校准，正常程序不会运行到这里 */
            joint_cali.cali_motor.cali_hook(CALI_FUNC_CMD_INIT);
        }
    }

}


/**
  * @brief          2006电机校准
  * @param[in][out] cali:指针指向电机数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准

  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool cali_joint_motor_hook(bool cmd)
{
    /* 如果已经校准过，直接从flash中拷贝校准值 */
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        // 对于工程机器人，不会执行到这里
        return 1;
    }
    else if(cmd == CALI_FUNC_CMD_ON)
    {
        /* 校准完毕 */
        if (cmd_cali_joint_hook())
        {
            cali_buzzer_off();
            joint_cali.calibrate_finished_flag = CALIBRATE_FINISHED;

            /* 之后电机应该进入无力模式 */
            return 1;
        }
            /* 校准未完成 */
        else
        {
            joint_cali_start_buzzer(5);

            return 0;
        }
    }
}
//static bool cali_gimbal_hook(uint32_t *cali, bool cmd)
//{
//
//    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
//
//    /* 已经校准过 从flash拷贝校准值 */
//    if (cmd == CALI_FUNC_CMD_INIT)
//    {
//        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
//                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
//                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
//
//        return 0;
//    }
//
//    /* 用户设置校准 */
//    else if (cmd == CALI_FUNC_CMD_ON)
//    {
//        /* 校准完毕 */
//        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
//                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
//                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
//        {
//            cali_buzzer_off();
//
//            return 1;
//        }
//        /* 校准未完成 */
//        else
//        {
//            gimbal_start_buzzer();
//
//            return 0;
//        }
//    }
//
//    return 0;
//}

///**
//  * @brief          获取关节电机校准数据结构体指针
//  * @param[in]      i:关节电机编号
//  * @retval         关节电机校准数据结构体指针
//  */
//const data_cali_t* get_joint_motor_cali_point(uint8_t i)
//{
//    if(i > JOINT_MOTOR_NUM) //防止数组越界
//    {
//        return NULL;
//    }
//    else
//    {
//        return &(joint_cali.cali_motor[i].motor_data);
//    }
//
//}