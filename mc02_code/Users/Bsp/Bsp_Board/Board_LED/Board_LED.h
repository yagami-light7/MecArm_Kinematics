#ifndef BSP_LED_H
#define BSP_LED_H

#include "main.h"

#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);


#endif
