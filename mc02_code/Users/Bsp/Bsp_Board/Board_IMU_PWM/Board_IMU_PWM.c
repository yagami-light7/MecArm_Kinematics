#include "Board_IMU_PWM.h"

extern TIM_HandleTypeDef htim3;
void imu_pwm_set(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, pwm);
}
