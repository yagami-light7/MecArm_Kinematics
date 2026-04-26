/**
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  * @file       Board_FDCAN.c
  * @brief      在H7系列开发板下，将FDCAN配置成经典CAN模式并进行初始化，并封装了FDCAN发送函数
  * @note       在工程机器人框架       FDCAN1:底盘DJI电机
  *                                 FDCAN2:大关节XIAO_MI电机
  *                                 FDCAN3:小关节DJI电机
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-28-2024     Light           1. done
  *
  @verbatim
  ==============================================================================
  * 本文件编写参考了YueLu战队开源的基本电控框架、达妙例程、辽宁科技大学王草凡开源
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Robot_Z ****************************
  */
#include "Board_FDCAN.h"

void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);

/**
  * @brief          FDCAN总线初始化
  * @param[in]      none
  * @retval         none
  */
void FDCAN_Filter_Init(void)
{
    //可能不需要这么多中断
    uint32_t FDCAN_RXActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE|FDCAN_IT_RX_FIFO0_FULL\
			|FDCAN_IT_RX_FIFO0_WATERMARK|FDCAN_IT_RX_FIFO0_MESSAGE_LOST \
			|FDCAN_IT_RX_FIFO1_NEW_MESSAGE| FDCAN_IT_RX_FIFO1_FULL\
			|FDCAN_IT_RX_FIFO1_WATERMARK|FDCAN_IT_RX_FIFO1_MESSAGE_LOST;

    FDCAN1_Config();
    FDCAN2_Config();
    FDCAN3_Config();

    /* 启动FDCAN */
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_RXActiveITs, 0);

    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan2,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);

    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_RXActiveITs, 0);

    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO0,FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan3,FDCAN_RX_FIFO1,FDCAN_RX_FIFO_OVERWRITE);
    HAL_FDCAN_Start(&hfdcan3);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_RXActiveITs, 0);

}

/**
 * @brief           FDCAN发送函数
 * @param[in]       hfdcan         FDCAN控制器的句柄
 * @param[in]       hfdcan_id      CAN消息的标识符
 * @param[in]       mes_data      指向要发送的数据的指针
 * @param[in]       mes_len       要发送的数据长度
 * @retval          uint8_t 返回状态，0表示成功
 */
uint8_t FDCAN_Send_Data(FDCAN_HandleTypeDef *hfdcan, uint16_t hfdcan_id, uint8_t *mes_data, uint32_t mes_len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    // 配置消息头
    TxHeader.Identifier = hfdcan_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;

    // 根据数据长度选择合适的数据长度码
    if (mes_len <= 8)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    }
    else if (mes_len == 12)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_12;
    }
    else if (mes_len == 16)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_16;
    }
    else if (mes_len == 20)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_20;
    }
    else if (mes_len == 24)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_24;
    }
    else if (mes_len == 48)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_48;
    }
    else if (mes_len == 64)
    {
        TxHeader.DataLength = FDCAN_DLC_BYTES_64;
    }

    // 配置其他消息头参数
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    // 将消息添加到发送FIFO队列中，并进行错误处理
    if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, mes_data) != HAL_OK)
    {
        Error_Handler();
    }
    return 0;
}
/**
  * @brief          FDCAN1配置初始化
  *                 使用FIFO0缓冲区
  * @param[in]      none
  * @retval         none
  */
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

}

/**
  * @brief          FDCAN2配置初始化
  *                 使用FIFO1缓冲区
  * @param[in]      none
  * @retval         none
  */
void FDCAN2_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType =  FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0x00000000;
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();
    }

}

/**
  * @brief          FDCAN3配置初始化
  *                 使用FIFO0 & FIFO1 缓冲区
  * @param[in]      none
  * @retval         none
  */
void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef can_filter_st;
    can_filter_st.IdType = FDCAN_STANDARD_ID; // 配置标准ID
    can_filter_st.FilterIndex = 0;  // 滤波器索引为2
    can_filter_st.FilterType = FDCAN_FILTER_MASK; // 掩码匹配滤波器
    can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO1; // 接送到的数据将被送到FIFO0 与 FIFO1
    can_filter_st.FilterID1 = 0x00000000; // 所有数据都可以通过
    can_filter_st.FilterID2 = 0x00000000;

    /* 配置 FDCAN3 的滤波器，如果配置失败，则进入错误处理函数 */
    if(HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter_st) != HAL_OK)
    {
        Error_Handler();  // 调用错误处理函数
    }
    /* 配置全局过滤器，设置为拒绝所有远程帧消息（FDCAN_REJECT）*/
    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        Error_Handler();  // 配置失败，调用错误处理函数
    }

}