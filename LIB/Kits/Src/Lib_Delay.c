#include "Lib_Delay.h"

void Delay(uint16_t DelayValue)  // us
{
    DelayValue *= 4;
    while (DelayValue--)
        ;
    return;
}

void delay_us(uint16_t nus)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_ENABLE(&htim3);
    while (__HAL_TIM_GET_COUNTER(&htim3) < nus)
    {
    }
    __HAL_TIM_DISABLE(&htim3);
}
