#ifndef BSP_DELAY_H
#define BSP_DELAY_H
#include "main.h"

extern void Delay_Init(void);
extern void Delay_us(uint16_t nus);
extern void Delay_ms(uint16_t nms);
#endif

