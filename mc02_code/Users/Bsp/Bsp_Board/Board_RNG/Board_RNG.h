#ifndef __BSP_RNG_H__
#define __BSP_RNG_H__

#include "main.h"

extern uint32_t RNG_get_random_num(void);
extern int32_t RNG_get_random_rangle(int min, int max);

#endif
