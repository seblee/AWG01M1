/* ----------------------- Platform includes --------------------------------*/
#include "DRV_TIME.h"
#include "Drv_Led.h"
/* ----------------------- Start implementation -----------------------------*/
extern TIM_HandleTypeDef htim14;
//定时1us
BOOL TimersInit_14(uint16_t timPeriod, uint8_t u8Type)
{
    uint16_t PrescalerValue = 0;
    PrescalerValue          = (uint16_t)(SystemCoreClock / 10000) - 1;  // 48MHZ

    __HAL_TIM_SET_AUTORELOAD(&htim14, timPeriod);
    __HAL_TIM_SET_PRESCALER(&htim14, PrescalerValue);

    if (u8Type == ENABLE)
    {
        TimersEnable_14();
    }
    else
    {
        __HAL_TIM_SET_COUNTER(&htim14, 0);
        TimersDisable_14();
    }

    return TRUE;
}

void TimersEnable_14(void)
{
    __HAL_TIM_SET_COUNTER(&htim14, 0);
    HAL_TIM_Base_Start_IT(&htim14);
}

// T0中断禁止
void TimersDisable_14(void)
{
    HAL_TIM_Base_Stop_IT(&htim14);
    __HAL_TIM_SET_COUNTER(&htim14, 0);
}
