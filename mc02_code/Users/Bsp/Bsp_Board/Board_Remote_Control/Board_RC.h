#ifndef ENGINEERING_BOARD_RC__H
#define ENGINEERING_BOARD_RC__H

#include "main.h"

extern void RC_Init(uint32_t *SrcAddress, uint8_t *rx1_buf, uint8_t *rx2_buf, uint8_t dma_buf_num);

extern void RC_Disable(void);

extern void RC_Restart(uint16_t dma_buf_num);

#endif
