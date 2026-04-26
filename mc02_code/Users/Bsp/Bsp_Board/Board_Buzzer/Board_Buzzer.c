#include "Board_Buzzer.h"

extern TIM_HandleTypeDef htim12;

void Buzzer_On(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(&htim12, psc);
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, pwm);

}

void Buzzer_OFF(void)
{
    __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, 0);
}

/**
  * @brief          蜂鸣器校准提示函数
  * @param[in]      num对应的关节ID数目，蜂鸣器播放对应的次数
  * @retval         None
  */
void joint_cali_start_buzzer(uint8_t num)
{
    static uint8_t show_num = 0;
    static uint8_t stop_num = 100;
    if (show_num == 0 && stop_num == 0)
    {
        show_num = num + 1;
        stop_num = 100;
    }
    else if (show_num == 0)
    {
        stop_num--;
        Buzzer_OFF();
    }
    else
    {
        static uint8_t tick = 0;
        tick++;
        if (tick < 50)
        {
            Buzzer_OFF();
        }
        else if (tick < 100)
        {
            Buzzer_On(1, 30000);
        }
        else
        {
            tick = 0;
            show_num--;
        }
    }
}