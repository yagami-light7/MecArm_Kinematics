#ifndef INFANTRY_ROBOT_ALG_FILTER_H
#define INFANTRY_ROBOT_ALG_FILTER_H

#include "arm_math.h"  // 包含CMSIS-DSP库
#include <stdio.h>

extern void IIR_Filter_init(void);

extern void IIR_Filter_calc(void);

#endif
