/**
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  * @file       HAL_FDCAN.cpp
  * @brief      FDCAN 外设再封装实现
  * @note       Hardware Abstract Layer 硬件抽象层
  *             1. 本文件风格对齐 HAL_USART.cpp，使用管理对象 + 函数接口形式
  *             2. 统一标准帧、扩展帧发送与接收读取
  *             3. 将原 Board_FDCAN 的滤波器配置、启动和通知使能逻辑迁移到 HAL 层
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Apr-10-2026     Codex           1. create
  *  V1.1.0     Apr-10-2026     Codex           2. merge board fdcan apis
  ****************************(C) COPYRIGHT 2026 Robot_Z ****************************
  */
#include "HAL_FDCAN.h"

/**
 * @brief FDCAN 管理对象实例化
 */
FDCAN_Manage_Object_t FDCAN1_Manage_Object = {&hfdcan1, FDCAN_RX_FIFO0};
FDCAN_Manage_Object_t FDCAN2_Manage_Object = {&hfdcan2, FDCAN_RX_FIFO1};
FDCAN_Manage_Object_t FDCAN3_Manage_Object = {&hfdcan3, FDCAN_RX_FIFO0};

static uint32_t FDCAN_Resolve_DLC(uint32_t length);
static bool FDCAN_Send_Frame(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint32_t id_type, uint8_t *data, uint32_t length);
static bool FDCAN_Config_Filter(FDCAN_HandleTypeDef *hfdcan, uint32_t filter_index, uint32_t rx_fifo);
static bool FDCAN_Start_And_Enable_IT(FDCAN_HandleTypeDef *hfdcan);

/**
  * @brief          FDCAN 总线初始化
  * @param[in]      none
  * @retval         none
  * @note           统一完成三路 FDCAN 的标准滤波器配置、启动与中断通知使能
  */
void FDCAN_Filter_Init(void)
{
//    FDCAN_Init(&FDCAN1_Manage_Object, &hfdcan1, FDCAN_RX_FIFO0);
//    FDCAN_Init(&FDCAN2_Manage_Object, &hfdcan2, FDCAN_RX_FIFO1);
//    FDCAN_Init(&FDCAN3_Manage_Object, &hfdcan3, FDCAN_RX_FIFO1);

    // 配置FDCAN滤波器参数
    if (!FDCAN_Config_Filter(FDCAN1_Manage_Object.hfdcan, 0U, FDCAN1_Manage_Object.rx_fifo))
    {
        Error_Handler();
    }

    if (!FDCAN_Config_Filter(FDCAN2_Manage_Object.hfdcan, 0U, FDCAN2_Manage_Object.rx_fifo))
    {
        Error_Handler();
    }

    if (!FDCAN_Config_Filter(FDCAN3_Manage_Object.hfdcan, 0U, FDCAN3_Manage_Object.rx_fifo))
    {
        Error_Handler();
    }

    // 启动FDCAN并使能接收中断
    if (!FDCAN_Start_And_Enable_IT(FDCAN1_Manage_Object.hfdcan))
    {
        Error_Handler();
    }

    if (!FDCAN_Start_And_Enable_IT(FDCAN2_Manage_Object.hfdcan))
    {
        Error_Handler();
    }

    if (!FDCAN_Start_And_Enable_IT(FDCAN3_Manage_Object.hfdcan))
    {
        Error_Handler();
    }
}

/**
  * @brief          FDCAN 管理对象初始化
  * @param[in]      FDCAN_Manage_Object FDCAN 管理对象
  *                 hfdcan              FDCAN 外设句柄
  *                 rx_fifo             默认接收 FIFO
  * @retval         none
  */
void FDCAN_Init(FDCAN_Manage_Object_t *FDCAN_Manage_Object, FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo)
{
    if (FDCAN_Manage_Object == NULL)
    {
        return;
    }

    FDCAN_Manage_Object->hfdcan = hfdcan;
    FDCAN_Manage_Object->rx_fifo = rx_fifo;
}


/**
 * @brief          发送标准帧数据
 * @param[in]      FDCAN_Manage_Object FDCAN 管理对象
 *                 identifier          标准帧 ID
 *                 data                数据区指针
 *                 length              数据长度
 * @retval         true  发送成功
 *                 false 发送失败
 */
bool FDCAN_Send_Std_Data(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint16_t identifier, uint8_t *data, uint32_t length)
{
    if (FDCAN_Manage_Object == NULL)
    {
        return false;
    }

    return FDCAN_Send_Frame(FDCAN_Manage_Object->hfdcan, identifier, FDCAN_STANDARD_ID, data, length);
}

/**
 * @brief          发送扩展帧数据
 */
bool FDCAN_Send_Ext_Data(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint32_t identifier, uint8_t *data, uint32_t length)
{
    if (FDCAN_Manage_Object == NULL)
    {
        return false;
    }

    return FDCAN_Send_Frame(FDCAN_Manage_Object->hfdcan, identifier, FDCAN_EXTENDED_ID, data, length);
}

/**
 * @brief          从默认 FIFO 读取一帧消息
 */
bool FDCAN_Read_Data(FDCAN_Manage_Object_t *FDCAN_Manage_Object, FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    if (FDCAN_Manage_Object == NULL)
    {
        return false;
    }

    return FDCAN_Read_Data_From_FIFO(FDCAN_Manage_Object, FDCAN_Manage_Object->rx_fifo, header, data);
}

/**
 * @brief          从指定 FIFO 读取一帧消息
 */
bool FDCAN_Read_Data_From_FIFO(FDCAN_Manage_Object_t *FDCAN_Manage_Object, uint32_t fifo, FDCAN_RxHeaderTypeDef *header, uint8_t *data)
{
    if (FDCAN_Manage_Object == NULL || FDCAN_Manage_Object->hfdcan == NULL || header == NULL || data == NULL)
    {
        return false;
    }

    return (HAL_FDCAN_GetRxMessage(FDCAN_Manage_Object->hfdcan, fifo, header, data) == HAL_OK);
}

/**
  * @brief          根据数据长度解析 DLC
  * @param[in]      length 数据长度
  * @retval         对应的 FDCAN DLC 编码
  */
static uint32_t FDCAN_Resolve_DLC(uint32_t length)
{
    switch (length)
    {
        case 0U:
            return FDCAN_DLC_BYTES_0;
        case 1U:
            return FDCAN_DLC_BYTES_1;
        case 2U:
            return FDCAN_DLC_BYTES_2;
        case 3U:
            return FDCAN_DLC_BYTES_3;
        case 4U:
            return FDCAN_DLC_BYTES_4;
        case 5U:
            return FDCAN_DLC_BYTES_5;
        case 6U:
            return FDCAN_DLC_BYTES_6;
        case 7U:
            return FDCAN_DLC_BYTES_7;
        case 8U:
            return FDCAN_DLC_BYTES_8;
        case 12U:
            return FDCAN_DLC_BYTES_12;
        case 16U:
            return FDCAN_DLC_BYTES_16;
        case 20U:
            return FDCAN_DLC_BYTES_20;
        case 24U:
            return FDCAN_DLC_BYTES_24;
        case 32U:
            return FDCAN_DLC_BYTES_32;
        case 48U:
            return FDCAN_DLC_BYTES_48;
        case 64U:
            return FDCAN_DLC_BYTES_64;
        default:
            return 0xFFFFFFFFU;
    }
}

/**
  * @brief          统一发送实现
  * @param[in]      hfdcan     FDCAN 外设句柄
  *                 identifier CAN ID
  *                 id_type    标识符类型
  *                 data       数据区
  *                 length     数据长度
  * @retval         true  发送成功
  *                 false 发送失败
  */
static bool FDCAN_Send_Frame(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint32_t id_type, uint8_t *data, uint32_t length)
{
    FDCAN_TxHeaderTypeDef tx_header = {};

    if (hfdcan == NULL)
    {
        return false;
    }

    if (length > 0U && data == NULL)
    {
        return false;
    }

    tx_header.Identifier = identifier;
    tx_header.IdType = id_type;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_Resolve_DLC(length);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0U;

    if (tx_header.DataLength == 0xFFFFFFFFU)
    {
        return false;
    }

    return (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, data) == HAL_OK);
}

/**
  * @brief          配置默认标准滤波器
  * @param[in]      hfdcan       FDCAN 外设句柄
  *                 filter_index 滤波器索引
  *                 rx_fifo      目标 FIFO
  * @retval         true  配置成功
  *                 false 配置失败
  */
static bool FDCAN_Config_Filter(FDCAN_HandleTypeDef *hfdcan, uint32_t filter_index, uint32_t rx_fifo)
{
    FDCAN_FilterTypeDef filter_config = {};

    if (hfdcan == NULL)
    {
        return false;
    }

    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = filter_index;
    filter_config.FilterType = FDCAN_FILTER_MASK;
    filter_config.FilterConfig = (rx_fifo == FDCAN_RX_FIFO0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
    filter_config.FilterID1 = 0x00000000U;
    filter_config.FilterID2 = 0x00000000U;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        return false;
    }

    return true;
}

/**
  * @brief          启动 FDCAN 并使能接收通知
  * @param[in]      hfdcan FDCAN 外设句柄
  * @retval         true  启动成功
  *                 false 启动失败
  */
static bool FDCAN_Start_And_Enable_IT(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t fdcan_rx_active_its;

    if (hfdcan == NULL)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigRxFifoOverwrite(hfdcan, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_ConfigRxFifoOverwrite(hfdcan, FDCAN_RX_FIFO1, FDCAN_RX_FIFO_OVERWRITE) != HAL_OK)
    {
        return false;
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK)
    {
        return false;
    }

    fdcan_rx_active_its = FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
                          FDCAN_IT_RX_FIFO0_FULL |
                          FDCAN_IT_RX_FIFO0_WATERMARK |
                          FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
                          FDCAN_IT_RX_FIFO1_NEW_MESSAGE |
                          FDCAN_IT_RX_FIFO1_FULL |
                          FDCAN_IT_RX_FIFO1_WATERMARK |
                          FDCAN_IT_RX_FIFO1_MESSAGE_LOST;

    if (HAL_FDCAN_ActivateNotification(hfdcan, fdcan_rx_active_its, 0U) != HAL_OK)
    {
        return false;
    }

    return true;
}
