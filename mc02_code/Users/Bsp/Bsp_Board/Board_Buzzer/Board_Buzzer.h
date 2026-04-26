#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "main.h"

extern void Buzzer_On(uint16_t psc, uint16_t pwm);
extern void Buzzer_OFF(void);
/**
  * @brief          蜂鸣器校准提示函数
  * @param[in]      num对应的关节ID数目，蜂鸣器播放对应的次数
  * @retval         None
  */
extern void joint_cali_start_buzzer(uint8_t num);

#endif
