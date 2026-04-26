/**
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  * @file       APL_RC_Hub.c
  * @brief      解析通信数据 接收队列数据进行解析，实现与ISR的解耦
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-28-2025     Light            1. done
  *
  @verbatim
  ==============================================================================
  *
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 Robot_Z ****************************
  */
#include "APL_RC_Hub.h"
#include "MWL_Data_Utils.h"
#include "HAL_USART.h"
#include "MWL_CRC.h"
#include "APL_MecArm.h"
#include "Dev_Remote_Control.h"
#include "App_Detect_Task.h"

/**
 * @brief 遥控器数据错误阈值
 */
#define RC_CHANNEL_ERROR_VALUE 700
#define MEC_ARM_TELEMETRY_SOF_1      0xA5U
#define MEC_ARM_TELEMETRY_SOF_2      0x5AU
#define MEC_ARM_TELEMETRY_SEND_DIV   2U

typedef struct __attribute__((packed))
{
    uint8_t sof_1;
    uint8_t sof_2;
    uint16_t seq;
    uint32_t tick_ms;
    float q[6];
    float tau[6];
    uint16_t crc16;
} mec_arm_identification_frame_t;

/**
 * @brief          绝对值函数
 * @param[in]      value 输入值
 * @retval         绝对值
 */
static int16_t RC_abs(int16_t value)
{
    if (value >= 0)
    {
        return value;
    }

    return (int16_t)(-value);
}

/**
 * @brief          遥控器协议解析
 * @param[in]      sbus_buf 原始18字节数据
 * @param[out]     rc_ctrl  解析结果结构体
 * @retval         none
 * @note           该函数从旧设备层迁移至APL层，配合队列实现ISR与解析解耦
 */
static void sbus_to_rc(const uint8_t *sbus_buf, dr16_control_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->key.last_v = rc_ctrl->key.v;
    rc_ctrl->rc.last_s[0] = rc_ctrl->rc.s[0];
    rc_ctrl->rc.last_s[1] = rc_ctrl->rc.s[1];

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07FF;
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07FF;
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07FF;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07FF;
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    rc_ctrl->mouse.x = (int16_t)(sbus_buf[6] | (sbus_buf[7] << 8));
    rc_ctrl->mouse.y = (int16_t)(sbus_buf[8] | (sbus_buf[9] << 8));
    rc_ctrl->mouse.z = (int16_t)(sbus_buf[10] | (sbus_buf[11] << 8));
    rc_ctrl->mouse.press_l = sbus_buf[12];
    rc_ctrl->mouse.press_r = sbus_buf[13];
    rc_ctrl->key.v = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    rc_ctrl->rc.ch[4] = (int16_t)(sbus_buf[16] | (sbus_buf[17] << 8));

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    rc_ctrl->ch_normalized[0] = (float)rc_ctrl->rc.ch[0] / 660.0f;
    rc_ctrl->ch_normalized[1] = (float)rc_ctrl->rc.ch[1] / 660.0f;
    rc_ctrl->ch_normalized[2] = (float)rc_ctrl->rc.ch[2] / 660.0f;
    rc_ctrl->ch_normalized[3] = (float)rc_ctrl->rc.ch[3] / 660.0f;
    rc_ctrl->ch_normalized[4] = (float)rc_ctrl->rc.ch[4] / 660.0f;


}

/**
 * @brief          判断遥控器数据是否出错
 * @param[in,out]  rc_ctrl 遥控器数据结构体
 * @retval         1 数据异常
 *                 0 数据正常
 */
static uint8_t RC_Data_Is_Error(dr16_control_t *rc_ctrl)
{
    if (rc_ctrl == NULL)
    {
        return 1U;
    }

    if (RC_abs(rc_ctrl->rc.ch[0]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[1]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[2]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[3]) > RC_CHANNEL_ERROR_VALUE ||
        RC_abs(rc_ctrl->rc.ch[4]) > RC_CHANNEL_ERROR_VALUE ||
        rc_ctrl->rc.s[0] == 0 ||
        rc_ctrl->rc.s[1] == 0)
    {
        rc_ctrl->rc.ch[0] = 0;
        rc_ctrl->rc.ch[1] = 0;
        rc_ctrl->rc.ch[2] = 0;
        rc_ctrl->rc.ch[3] = 0;
        rc_ctrl->rc.ch[4] = 0;
        rc_ctrl->rc.s[0] = RC_SW_DOWN;
        rc_ctrl->rc.s[1] = RC_SW_DOWN;
        rc_ctrl->mouse.x = 0;
        rc_ctrl->mouse.y = 0;
        rc_ctrl->mouse.z = 0;
        rc_ctrl->mouse.press_l = 0;
        rc_ctrl->mouse.press_r = 0;
        rc_ctrl->key.v = 0;
        return 1U;
    }

    return 0U;
}

/**
 * @brief 构造数据解析对象
 */
Class_RC_Hub RC_Hub;


/**
 * @brief   远程控制中心线程
 */
void _Thread_RC_Hub_(void *pvParameters)
{
    (void)pvParameters;

    vTaskDelay(RC_HUB_INIT_TIME);
    RC_Hub.Init(&DR16_Module, &UART5_Manage_Object, &VT_Module, &UART7_Manage_Object);

    while (1)
    {
        RC_Hub.Data_Analysis();
        RC_Hub.SendMecArmIdentificationTelemetry();

        vTaskDelay(RC_HUB_CONTROL_TIME);
    }
}


/**
 * @brief          初始化数据解析类
 * @param[in]      *_DR16_Module_Obj_       DR16模块指针
 *                 *_DR16_UART_Manage_Obj_  DR16串口实例指针
 *                 *_VT_Module_Obj_         图传模块指针
 *                 *_VT_UART_Manage_Obj_    图传串口实例指针
 * @retval         none
 */
void Class_RC_Hub::Init(Class_DR16 *_DR16_Module_Obj_, UART_Manage_Object_t *_DR16_UART_Manage_Obj_,
                        Class_Video_Transmission *_VT_Module_Obj_, UART_Manage_Object_t *_VT_UART_Manage_Obj_)
{
    _DR16_Module_Obj_->Init(_DR16_UART_Manage_Obj_);
    _VT_Module_Obj_->Init(_VT_UART_Manage_Obj_);
    this->DR16_Moudle = _DR16_Module_Obj_;
    this->VT_Moudle = _VT_Module_Obj_;
}


/**
 * @brief          解析机器人交互数据，更新对应控制结构体
 * @param[in]      none
 * @retval         none
 */
void Class_RC_Hub::Data_Analysis()
{
    // 临时缓冲区
    uint8_t dr16_data[DR16_FRAME_LENGTH];
    uint8_t custom_data[30];
    uint8_t remote_data[12];
    uint8_t vt_rc_data[21];

    // DR16遥控器
    if (DR16_Moudle != NULL && pdPASS == xQueueReceive(DR16_Moudle->dr16_robot.xdr16_queue, dr16_data, 0))
    {
        sbus_to_rc(dr16_data, &dr16_control);

        if (RC_Data_Is_Error(&dr16_control) == 0U)
        {
//            detect_hook(DBUS_TOE);
        }
    }

    // 自定义控制器
    if(pdPASS == xQueueReceive(VT_Moudle->custom_robot.xcustom_queue, custom_data, 0))
    {
        uint8_to_float(custom_data, custom_control.custom_angle_set, JOINTS_NUM);
//        detect_hook(VT_TOE);
//        detect_hook(CUSTOM_TOE);
    }

    // 图传链路键鼠
    if(pdPASS == xQueueReceive(VT_Moudle->remote_robot.xremote_queue, remote_data, 0))
    {
        remote_control.last_left_button_down  =  remote_control.left_button_down;
        remote_control.last_right_button_down =  remote_control.right_button_down;
        remote_control.last_keyboard_value    =  remote_control.keyboard_value;

        memcpy(&remote_control.mouse_x, &remote_data[0], sizeof(remote_control.mouse_x));
        memcpy(&remote_control.mouse_y, &remote_data[2], sizeof(remote_control.mouse_y));
        memcpy(&remote_control.mouse_z, &remote_data[4], sizeof(remote_control.mouse_z));
        memcpy(&remote_control.left_button_down, &remote_data[6], sizeof(remote_control.left_button_down));
        memcpy(&remote_control.right_button_down, &remote_data[7], sizeof(remote_control.right_button_down));
        memcpy(&remote_control.keyboard_value, &remote_data[8], sizeof(remote_control.keyboard_value));

//        detect_hook(VT_TOE);
    }

    // 新图传键鼠+遥控
    if(pdPASS == xQueueReceive(VT_Moudle->vt_rc_robot.xrc_vt_queue, vt_rc_data, 0))
    {
        //帧头 2bytes
        vt_rc_control.sof_1 = vt_rc_data[0];
        vt_rc_control.sof_2 = vt_rc_data[1];

        // 提取 8 字节的 bitfield 数据（data[2] ~ buf[9]）
        uint64_t bitfield = 0;
        for (int i = 0; i < 8; i++)
        {
            bitfield |= ((uint64_t)vt_rc_data[i + 2]) << (8 * i);
        }

        // 图传键位 8bytes
        vt_rc_control.ch_0     = (bitfield >> 0)  & 0x7FF;
        vt_rc_control.ch_1     = (bitfield >> 11) & 0x7FF;
        vt_rc_control.ch_2     = (bitfield >> 22) & 0x7FF;
        vt_rc_control.ch_3     = (bitfield >> 33) & 0x7FF;
        vt_rc_control.mode_sw  = (bitfield >> 44) & 0x03;
        vt_rc_control.pause    = (bitfield >> 46) & 0x01;
        vt_rc_control.fn_1     = (bitfield >> 47) & 0x01;
        vt_rc_control.fn_2     = (bitfield >> 48) & 0x01;
        vt_rc_control.wheel    = (bitfield >> 49) & 0x7FF;
        vt_rc_control.trigger  = (bitfield >> 60) & 0x01;

        // 键鼠键位 9bytes
        int16_t tmp_mouse_x = (vt_rc_data[10] | (vt_rc_data[11] << 8));
        int16_t tmp_mouse_y = (vt_rc_data[12] | (vt_rc_data[13] << 8));
        int16_t tmp_mouse_z = (vt_rc_data[14] | (vt_rc_data[15] << 8));

        vt_rc_control.mouse_x = (float) tmp_mouse_x;
        vt_rc_control.mouse_y = (float) tmp_mouse_y;
        vt_rc_control.mouse_z = (float) tmp_mouse_z;

        uint8_t mouse_byte = vt_rc_data[16];
        vt_rc_control.mouse_left   = (mouse_byte >> 0) & 0x03;
        vt_rc_control.mouse_right  = (mouse_byte >> 2) & 0x03;
        vt_rc_control.mouse_middle = (mouse_byte >> 4) & 0x03;

        vt_rc_control.key   = vt_rc_data[17] | (vt_rc_data[18] << 8);

        // CRC校验 2bytes
        vt_rc_control.crc16 = vt_rc_data[19] | (vt_rc_data[20] << 8);

//        detect_hook(VT_TOE);

        // 摇杆&拨轮归中
        vt_rc_control.ch_0   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.ch_1   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.ch_2   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.ch_3   -= RC_CH_VALUE_OFFSET;
        vt_rc_control.wheel  -= RC_CH_VALUE_OFFSET;

    }
}

/**
 * @brief          将控制结构体的原始数据进行加工
 * @param[in]      none
 * @retval         none
 */
void Class_RC_Hub::SendMecArmIdentificationTelemetry(void)
{
    static uint8_t send_divider = 0U;   // 间隔发送，降低通信频率 100HZ发送频率
    static uint16_t frame_seq = 0U;

    float q[6] = {0.0f};
    float tau[6] = {0.0f};
    mec_arm_identification_frame_t frame = {};

    if (++send_divider < MEC_ARM_TELEMETRY_SEND_DIV)
    {
        return;
    }
    send_divider = 0U;

    if (UART10_Manage_Object.huart->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    MecArm.GetIdentificationData(q, tau);

    frame.sof_1 = MEC_ARM_TELEMETRY_SOF_1;
    frame.sof_2 = MEC_ARM_TELEMETRY_SOF_2;
    frame.seq = frame_seq++;
    frame.tick_ms = (uint32_t)xTaskGetTickCount();

    memcpy(frame.q, q, sizeof(frame.q));
    memcpy(frame.tau, tau, sizeof(frame.tau));

    memcpy(UART10_Manage_Object.tx_buffer, &frame, sizeof(frame));
    MWL_CRC16_Append(UART10_Manage_Object.tx_buffer, sizeof(frame));

    (void)UART_Send_Data(&UART10_Manage_Object,UART10_Manage_Object.tx_buffer,(uint16_t)sizeof(frame));
}

void Class_RC_Hub::Data_Processing()
{
    // 预留接口：后续在此处完成按键边沿、摇杆死区、模式量化等应用层加工
}
