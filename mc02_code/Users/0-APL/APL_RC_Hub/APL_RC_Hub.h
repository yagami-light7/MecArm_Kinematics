#pragma once

#include "main.h"
#include "HDL_DR16.h"
#include "HDL_VT.h"

#define JOINTS_NUM  6
#define RC_HUB_INIT_TIME    501
#define RC_HUB_CONTROL_TIME 5

#define RC_CH_VALUE_MIN         ((uint16_t)0364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

#define MOUSE_VALUE_MIN         ((int16_t)-32768)
#define MOUSE_VALUE_MAX         ((int16_t) 32767)

#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)

/**
 * @brief DR16
 */
typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
        char last_s[2];
    } rc;
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
        uint16_t last_v;
    } key;

    float ch_normalized[5]; // 归一化后的摇杆值，范围[-1.0, 1.0]

}dr16_control_t;


/**
 * @brief 自定义控制器
 */
typedef struct
{
    // 串口数据包
//    uint8_t data[30];

    // 解析完毕 机械臂关节角度
    float custom_angle_set[JOINTS_NUM];
}custom_control_t;


/**
 * @brief 0x304键鼠信息
 */
typedef __packed struct
{
    // 串口数据包
//    uint8_t data[10];

    // 解析完毕 键鼠数据
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

}remote_control_t;


/**
 * @brief VT13图传接收端
 */
typedef __packed struct
{
    // 串口数据包
//    uint8_t data[21];

    // 解析完毕 键鼠+摇杆数据
    uint8_t sof_1; //0xA9
    uint8_t sof_2; //0X53

    int16_t ch_0;
    int16_t ch_1;
    int16_t ch_2;
    int16_t ch_3;
    uint8_t mode_sw;
    uint8_t pause;
    uint8_t fn_1;
    uint8_t fn_2;
    int16_t wheel;
    uint8_t trigger;

    float mouse_x;
    float mouse_y;
    float mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint8_t mouse_middle;
    uint16_t key;

    uint16_t crc16;

    uint16_t chassis_key_last;  // 底盘线程中刷新
    uint16_t gimbal_key_last;   // 大臂线程中刷新
    uint16_t mec_arm_key_last;  // 小臂线程中刷新

    uint8_t last_fn_1;      // 大臂线程中刷新
    uint8_t last_fn_2;      // 大臂线程中刷新
    uint8_t last_pause;     // 大臂线程中刷新
    uint8_t last_trigger;   // 大臂线程中刷新

}vt_rc_control_t;

#ifdef __cplusplus

class Class_RC_Hub
{
public:
    /* Functions */
    void Init(Class_DR16 *_DR16_Module_Obj_, UART_Manage_Object_t *_DR16_UART_Manage_Obj_,
              Class_Video_Transmission *_VT_Module_Obj_, UART_Manage_Object_t *_VT_UART_Manage_Obj_);
    void Data_Analysis(void);
    void Data_Processing(void);
    void SendMecArmIdentificationTelemetry(void);

    /* Class */
    Class_DR16 *DR16_Moudle;
    Class_Video_Transmission *VT_Moudle;

    /* Struct */
    dr16_control_t dr16_control;        // DR16

    custom_control_t custom_control;    // 0x302
    remote_control_t remote_control;    // 0x304
    vt_rc_control_t  vt_rc_control;     // 0xA9 0x53

private:


protected:


};

/**
 * @brief 构造远程控制中心
 */
extern Class_RC_Hub RC_Hub;

#endif


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   远程控制中心线程
 */
extern void _Thread_RC_Hub_(void *pvParameters);


#ifdef __cplusplus
}

#endif
