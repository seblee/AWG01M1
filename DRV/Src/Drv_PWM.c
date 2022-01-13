/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //PWM相关驱动函数
  Version:         //V1.0
  Function List:   //PWM_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/15    1.0     build this moudle
***********************************************************/

#include "stm32g0xx_hal.h"
#include "stm32f0xx_misc.h"
#include "macro.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "Drv_PWM.h"

/**
 * @brief  PWM GPIO and timer RCC	 configuration
 * @param  none
 * @param  none
 * @retval none
 */
static void PWM_Rcc_Conf(void)
{
    /* TIM1 clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* GPIOA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO , ENABLE);
}

/**
 * @brief PWM GPIO configuration.
 * @param  None
 * @retval None
 */
static void PWM_GPIO_conf(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);

    GPIO_SetBits(GPIOA, GPIO_PIN_9);
    return;
}

/**
 * @brief  PWM timer parameter configuration
 * @param  none
 * @param  none
 * @retval none
 */
static void Drv_PWM_Conf(uint32_t Freq, uint16_t Dutycycle)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    uint16_t PrescalerValue = 0;
    uint16_t ccr_value      = 500;
    //	uint16_t Time_Period = 0;
    //	uint16_t Time_Pulse = 0;

    /* -----------------------------------------------------------------------
    TIM8 Configuration: generate 4 PWM signals with 4 different duty cycles:
    The TIM8CLK frequency is set to SystemCoreClock (Hz), to get TIM8 counter
    clock at 24 MHz the Prescaler is computed as following:
     - Prescaler = (TIM8CLK / TIM8 counter clock) - 1
    SystemCoreClock is set to 72 MHz for Low-density, Medium-density, High-density
    and Connectivity line devices and to 24 MHz for Low-Density Value line and
    Medium-Density Value line devices

    The TIM8 is running at 36 KHz: TIM8 Frequency = TIM8 counter clock/(ARR + 1)
                                                  = 24 MHz / 666 = 36 KHz
    TIM8 Channel1 duty cycle = (TIM8_CCR1/ TIM8_ARR)* 100 = 50%
    TIM8 Channel2 duty cycle = (TIM8_CCR2/ TIM8_ARR)* 100 = 37.5%
    TIM8 Channel3 duty cycle = (TIM8_CCR3/ TIM8_ARR)* 100 = 25%
    TIM8 Channel4 duty cycle = (TIM8_CCR4/ TIM8_ARR)* 100 = 12.5%
  ----------------------------------------------------------------------- */

    //	Time_Period=(uint16_t)
    /* Compute the prescaler value */
    //  PrescalerValue = (uint16_t) (SystemCoreClock / Freq) - 1;
    PrescalerValue = (uint16_t)(SystemCoreClock / Freq / 6) - 1;  // 8MHZ
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period            = 999;
    TIM_TimeBaseStructure.TIM_Prescaler         = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = ccr_value;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;

    // PWM channel 2 init
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    /* TIM1 enable counter */
    TIM_Cmd(TIM1, ENABLE);
    /* TIM1 主输出使能 */
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

/**
  * @brief  set designated channel's pwm wave duty cycle, range is [0,100]
  * @param  channel: range from 1 to 7
    * @param  duty_cycle:	range from 0%~100%, 1% step size
  * @retval
            @arg 1: success
            @arg 0: error
  */
uint16_t PWM_Conf(uint16_t duty_cycle)
{
    uint16_t ret = 0;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse        = duty_cycle * 10;
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_Low;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;

    if (duty_cycle > 100)
        return 0;

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    ret = 1;
    return ret;
}

/**
 * @brief  PWM timer parameter configuration
 * @param  none
 * @param  none
 * @retval none
 */
uint16_t PWM_Set_AO(uint16_t ao_data)
{
    if (ao_data > 100)
    {
        return 0;
    }
    else
    {
        PWM_Conf(100 - ao_data);
        return 1;
    }
}

//模拟输出执行函数
void PWM_AO_update(void)
{
    //    uint16_t target_output=0;
    //    uint16_t current_output=0;
    //    int16_t ret_val=0;
    //    int16_t delta;
    ////	g_sVariable.gPara.u16FanOut=70;
    //		target_output=g_sVariable.gPara.u16FanOut;
    //		current_output = g_sVariable.gPara.u16AO;
    //		delta = target_output - current_output;
    //
    //		if(g_sVariable.gPara.u16FanStep)
    //		{
    //			if(abs(delta) >= g_sVariable.gPara.u16FanStep)
    //			{
    //					if(delta > 0)
    //							ret_val = current_output + g_sVariable.gPara.u16FanStep;
    //					else
    //							ret_val = current_output - g_sVariable.gPara.u16FanStep;
    //			}
    //			else if(delta == 0)
    //			{
    //					ret_val = current_output;
    //			}
    //			else
    //			{
    //					ret_val = current_output + delta;
    //			}
    //		}
    //		else
    //		{
    //			ret_val=target_output;
    //		}
    //
    //		g_sVariable.gPara.u16AO=ret_val;
    //		PWM_Set_AO(g_sVariable.gPara.u16AO);

    //		PWM_Set_AO(g_sVariable.status.u16AO);

    return;
}

/*************************************************
  Function:       // Drv_PWM_Init
  Description:    // pwm初始化函数
  Calls:          // Drv_PWM_Init()

  Called By:      // Core_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void Drv_PWM_Init(void)
{
    PWM_Rcc_Conf();
    PWM_GPIO_conf();
    Drv_PWM_Conf(PWM_1KHZ, 500);  // set frequency to 1000Hz
    PWM_Set_AO(0);
    return;
}
