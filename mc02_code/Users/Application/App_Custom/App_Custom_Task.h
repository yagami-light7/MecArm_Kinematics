#ifndef ENGINEERING_APP_CUSTOM_TASK_H
#define ENGINEERING_APP_CUSTOM_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "Dev_Custom.h"
#include "Dev_Remote_Control.h"
#include "string.h"

#define CUSTOM_TASK_INIT_TIME    201 // 任务初始化时间
#define CUSTOM_TASK_CONTROL_TIME 2  // 任务控制时间

#define JOINT_NUM 6 // 关节数量
#define FRAME_HEADER_LENGTH 5 // 帧头数据长度 包括 帧头0xA5 + 数据段长度30 + 包序号 + 两位CRC8校验
#define CMD_ID_LENGTH 2       // 命令码ID数据长度
#define DATA_LENGTH   30      // 数据段长度
#define FRAME_TAIL_LENGTH 2   // 帧尾数据长度

#define DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个数据帧的长度

#define CONTROLLER_CMD_ID 0x0309 // 自定义控制器（发给选手端）命令码

typedef struct
{
    /* 原始 小臂回传数据 */
    float custom_data_raw[6];

    /* 最终 关节电机的角度设定值 */
    float custom_angle_set[6];

} Custom_Controller_t;

typedef struct
{
    // 解析完毕数据
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;


    int8_t last_left_button_down;
    int8_t last_right_button_down;
    uint16_t last_keyboard_value;

    // 标志位
    uint32_t rc_analysis_flag;
}Remote_Controller_t;

#pragma pack(push, 1) // 开始紧凑对齐

typedef __packed struct
{
    __packed struct
    {
        uint8_t  sof;             // 起始字节，固定值为0xA5
        uint16_t data_length;     // 数据帧中data的长度
        uint8_t  seq;             // 包序号
        uint8_t  crc8;            // 帧头CRC8校验
    } frame_header;               // 帧头
    __packed uint16_t cmd_id;     // 命令码
    __packed uint8_t data[30];    // 自定义控制器的数据帧
    __packed uint16_t frame_tail; // 帧尾CRC16校验
} Data_Frame_t;                   // 自定义控制器数据包

#pragma pack(pop)   // 恢复默认对齐

#ifdef __cplusplus
extern "C"{
#endif


/**
 * @brief          自定义控制器通信任务
 * @param[in]      none
 * @retval         none
 */
extern void Custom_Controller_Task(void *pvParameters);

/**
 * @brief          返回自定义控制器数组结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
extern Custom_Controller_t *Get_Custom_Controller_Point(void);

/**
 * @brief          返回图传链路数据结构体指针
 * @param[in]      none
 * @retval         custom_controller的指针
 */
extern Remote_Controller_t *Get_Remote_Control_0x304_Point(void);

#ifdef __cplusplus
}
#endif

#endif
