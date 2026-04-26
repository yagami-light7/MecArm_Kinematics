/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       calibrate_task.c/h
  * @brief      校准设备，包括云台,陀螺仪,加速度计,磁力计,底盘.云台校准是主要计算零点
  *             和最大最小相对角度.云台校准是主要计算零漂.加速度计和磁力计校准还没有实现
  *             因为加速度计还没有必要去校准,而磁力计还没有用.底盘校准是使M3508进入快速
  *             设置ID模式.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Oct-25-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis clabration
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
  *                 float zzz;
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


#ifndef INFANTRY_ROBOT_APP_CALIBRATE_TASK_H
#define INFANTRY_ROBOT_APP_CALIBRATE_TASK_H

#include "main.h"
#include "Board_Buzzer.h"

#define CALIBRATE_FINISHED 1

#define JOINT_MOTOR_NUM    2

//when imu is calibrating ,buzzer set frequency and strength. 当imu在校准,蜂鸣器的设置频率和强度
#define imu_start_buzzer()          Buzzer_On(95, 10000)
//when gimbal is calibrating ,buzzer set frequency and strength.当云台在校准,蜂鸣器的设置频率和强度
#define gimbal_start_buzzer()       Buzzer_On(31, 19999)
#define cali_buzzer_off()           Buzzer_OFF()            //buzzer off，关闭蜂鸣器


//get stm32 chip temperature, to calc imu control temperature.获取stm32片内温度，计算imu的控制温度
#define cali_get_mcu_temperature()  get_temprate()



#define cali_flash_read(address, buf, len)  flash_read((address), (buf), (len))                     //flash read function, flash 读取函数
#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len))     //flash write function,flash 写入函数
#define cali_flash_erase(address, page_num) flash_erase_address((address), (page_num))              //flash erase function,flash擦除函数


#define get_remote_ctrl_point_cali()        Get_Remote_Control_Point()  //get the remote control point，获取遥控器指针
#define gyro_cali_disable_control()         RC_Disable()                 //when imu is calibrating, disable the remote control.当imu在校准时候,失能遥控器
#define gyro_cali_enable_control()          RC_Restart(SBUS_RX_BUF_NUM)

// calc the zero drift function of gyro, 计算陀螺仪零漂
#define gyro_cali_fun(cali_scale, cali_offset, time_count)  INS_cali_gyro((cali_scale), (cali_offset), (time_count))
//set the zero drift to the INS task, 设置在INS task内的陀螺仪零漂
#define gyro_set_cali(cali_scale, cali_offset)              INS_set_cali_gyro((cali_scale), (cali_offset))


#define FLASH_USER_ADDR         ADDR_FLASH_SECTOR_9 //write flash page 9,保存的flash页地址

#define GYRO_CONST_MAX_TEMP     45.0f               //max control temperature of gyro,最大陀螺仪控制温度

#define CALI_FUNC_CMD_ON        1                   //尚未设置过，设置校准时执行对应的钩子函数
#define CALI_FUNC_CMD_INIT      0                   //已经校准过，设置校准值时直接从flash中拷贝

#define CALIBRATE_CONTROL_TIME  1                   //osDelay time,  means 1ms.1ms 系统延时

#define CALI_SENSOR_HEAD_LEGHT  1

#define SELF_ID                 0                   //ID
#define FIRMWARE_VERSION        12345               //handware version.
#define CALIED_FLAG             0x55                // means it has been calibrated
//you have 20 seconds to calibrate by remote control. 有20s可以用遥控器进行校准
#define CALIBRATE_END_TIME          20000
//when 10 second, buzzer frequency change to high frequency of gimbal calibration.当10s的时候,蜂鸣器切成高频声音
#define RC_CALI_BUZZER_MIDDLE_TIME  10000
//in the beginning, buzzer frequency change to low frequency of imu calibration.当开始校准的时候,蜂鸣器切成低频声音
#define RC_CALI_BUZZER_START_TIME   0


#define rc_cali_buzzer_middle_on()  gimbal_start_buzzer()
#define rc_cali_buzzer_start_on()   imu_start_buzzer()
#define RC_CMD_LONG_TIME            2000

#define RCCALI_BUZZER_CYCLE_TIME    400
#define RC_CALI_BUZZER_PAUSE_TIME   200
#define RC_CALI_VALUE_HOLE          600     //remote control threshold, the max value of remote control channel is 660.


#define GYRO_CALIBRATE_TIME         20000   //gyro calibrate time,陀螺仪校准时间


//cali device name
typedef enum
{
    CALI_S_J0 = 0,
    CALI_S_J1 = 1,
    CALI_S_J2 = 2,
    CALI_L_J3 = 3,
    //add more...
    CALI_LIST_LENGHT,
} cali_id_e;


typedef __packed struct
{
    uint8_t name[3];                                    //device name
    uint8_t cali_done;                                  //0x55 means has been calibrated
    uint8_t flash_len : 7;                              //buf length
    uint8_t cali_cmd : 1;                               //1 means to run cali hook function,
    uint32_t *flash_buf;                                //link to device calibration data
    bool (*cali_hook)(uint32_t *point, bool cmd);     //cali function
} cali_sensor_t;

//header device
typedef __packed struct
{
    uint8_t self_id;            // the "SELF_ID"
    uint16_t firmware_version;  // set to the "FIRMWARE_VERSION"
    //'temperature' and 'latitude' should not be in the head_cali, because don't want to create a new sensor
    //'temperature' and 'latitude'不应该在head_cali,因为不想创建一个新的设备就放这了
    int8_t temperature;         // imu control temperature
    float latitude;              // latitude
} head_cali_t;

///* 电机数据结构体 */
//typedef struct
//{
//    /* 角度最值 */
//    float max_angle;
//    float min_angle;
//
//    /* 角度中值 */
//    uint16_t offset_angle;
//
//}data_cali_t;

/* 单个关节电机校准结构体 */
typedef struct
{
    /* 校准数据结构体 */
//    data_cali_t motor_data;

    /* 电机校准函数 钩子函数 */
    bool (*cali_hook)(bool cmd);

} joint_motor_cali_t;

/* 总校准结构体 */
typedef struct
{
    // 校准完成标志位 完成1 未完成0
    uint8_t calibrate_finished_flag;

    // 电机校准结构体数组
    joint_motor_cali_t cali_motor;

} joint_cali_t;

extern joint_cali_t joint_cali;


/**
  * @brief          校准任务
  * @param[in]      pvParameters: 空
  * @retval         none
  */
extern void Calibrate_Task(void *pvParameters);

/**
  * @brief          电机校准初始化
  * @param[in]      none
  * @retval         none
  */
extern void Cali_Param_Init(void);


/**
  * @brief          获取imu控制温度, 单位℃
  * @param[in]      none
  * @retval         imu控制温度
  */
extern int8_t get_control_temperature(void);


/**
  * @brief          获取纬度,默认22.0f
  * @param[out]     latitude:fp32指针
  * @retval         none
  */
extern void get_flash_latitude(float *latitude);

///**
//  * @brief          获取关节电机校准数据结构体指针
//  * @param[in]      i:关节电机编号
//  * @retval         关节电机校准数据结构体指针
//  */
//extern const data_cali_t* get_joint_motor_cali_point(uint8_t i);

#endif
