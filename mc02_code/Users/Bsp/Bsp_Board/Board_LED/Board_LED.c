#include "Board_LED.h"

#define WS2812_LowLevel    0xC0     // 0码
#define WS2812_HighLevel   0xF0     // 1码

/**
 * 控制WS2812 LED的颜色
 * 通过SPI接口发送控制信号，以设置LED的红、绿、蓝颜色分量
 *
 * @param r 红色分量值，范围0-255
 * @param g 绿色分量值，范围0-255
 * @param b 蓝色分量值，范围0-255
 */
void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b)
{
    // 定义一个24字节的传输缓冲区，用于存储RGB颜色数据
    uint8_t txbuf[24];
    // 定义一个用于传输的临时变量，初始值为0
    uint8_t res = 0;

    // 循环8次，分别处理RGB颜色的每一位
    for (int i = 0; i < 8; i++)
    {
        // 根据绿色分量的每一位设置txbuf数组的值
        txbuf[7-i]  = (((g>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        // 根据红色分量的每一位设置txbuf数组的值
        txbuf[15-i] = (((r>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
        // 根据蓝色分量的每一位设置txbuf数组的值
        txbuf[23-i] = (((b>>i)&0x01) ? WS2812_HighLevel : WS2812_LowLevel)>>1;
    }

    // 发送一个空数据以开始传输，这里不关心发送的具体内容，所以使用res变量
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 0, 0xFFFF);
    // 等待SPI接口空闲，确保数据传输完成
    while (WS2812_SPI_UNIT.State != HAL_SPI_STATE_READY);
    // 发送包含RGB颜色数据的缓冲区
    HAL_SPI_Transmit(&WS2812_SPI_UNIT, txbuf, 24, 0xFFFF);

    // 发送100次空数据，以确保所有颜色数据都已正确发送
    for (int i = 0; i < 100; i++)
    {
        HAL_SPI_Transmit(&WS2812_SPI_UNIT, &res, 1, 0xFFFF);
    }
}
