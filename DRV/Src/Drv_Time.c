/* ----------------------- Platform includes --------------------------------*/
#include "DRV_TIME.h"
#include "Drv_DIO.h"
#include "Drv_Led.h"
/* ----------------------- Start implementation -----------------------------*/

//定时1us
BOOL TimersInit_14(uint16_t timPeriod, uint8_t u8Type)
{
    // 	uint16_t PrescalerValue = 0;
    // 	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    // 	NVIC_InitTypeDef NVIC_InitStructure;

    // 	/* TIM15 clock enable */
    // 	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    // 	/* Compute the prescaler value */
    // 	//SystemCoreClock = 48MHZ
    // //	PrescalerValue = 48 - 1;//48分频,计一个数1us
    // 	  PrescalerValue = (uint16_t) (SystemCoreClock / 1000000/6/2) - 1;//48MHZ
    // 	/* Time base configuration */
    // 	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
    // 	TIM_TimeBaseStructure.TIM_Period = timPeriod;//1us
    // //	TIM_TimeBaseStructure.TIM_Period = 999;//1us*999=999us
    // 	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    // 	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    // 	TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    // 	TIM_TimeBaseInit(TIM14,&TIM_TimeBaseStructure);
    // 	TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
    // 	if(u8Type==ENABLE)
    // 	{
    // 		TIM_Cmd(TIM14, ENABLE);

    // 		NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
    // 		NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
    // 		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    // 		NVIC_Init(&NVIC_InitStructure);
    // 	}
    // 	else
    // 	{
    // 		TimersDisable_14();
    // 	}

    return TRUE;
}

// T0中断允许
void TimersEnable_14(void)
{
    // TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    // TIM_ITConfig(TIM14, TIM_IT_Update, ENABLE);
    // TIM_SetCounter(TIM14, 0);
    // TIM_Cmd(TIM14, ENABLE);
    return;
}

// T0中断禁止
void TimersDisable_14(void)
{
    // TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
    // TIM_ITConfig(TIM14, TIM_IT_Update, DISABLE);
    // TIM_SetCounter(TIM14, 0);
    // TIM_Cmd(TIM14, DISABLE);
    return;
}

void TIM14_IRQHandler(void)
{
    // if (TIM_GetITStatus(TIM14, TIM_IT_Update) != RESET)
    // {
    //     TIM_ClearFlag(TIM14, TIM_FLAG_Update);        //清中断标记
    //     TIM_ClearITPendingBit(TIM14, TIM_IT_Update);  //清除定时器T3溢出中断标志位
    Sync_Di_timeout();  // DI采集
    // }
    return;
}
