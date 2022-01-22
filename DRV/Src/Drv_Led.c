/******************** (C) COPYRIGHT 2011 青风电子 ********************
 * 文件名  ：led.c
 * 描述    ：
 * 实验平台：青风stm32f051开发板
 * 描述    ：led驱动函数
 * 作者    ：青风
 * 店铺    ：qfv5.taobao.com
 **********************************************************************************/

#include "Drv_Led.h"
#include "global.h"
// #include "Drv_PWM.h"
/*********************************************************
 * @name   led_init
 * @brief  led gpio and port bank clock initilization
 * @calls  gpio dirvers
 * @called BackGround_proc()
 * @param  None
 * @retval None
 *********************************************************/
void led_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin   = LED1_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
}

/*********************************************************
 * @name   led_open
 * @brief  led highlight
 * @calls  GPIO_ResetBits()
 * @called None
 * @param  None
 * @retval None
 *********************************************************/
void led_open(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_RESET);
}
/*********************************************************
 * @name   led_close
 * @brief  led shut
 * @calls  GPIO_SetBits()
 * @called None
 * @param  None
 * @retval None
 *********************************************************/
void led_close(void)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, GPIO_PIN_SET);
}

/*********************************************************
 * @name   led_toggle
 * @brief  led toggles each time called
 * @calls  GPIO_WriteBit()
 * @called None
 * @param  None
 * @retval None
 *********************************************************/
void led_toggle(void)
{
    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
}

void TEST_toggle(void)
{
    HAL_GPIO_TogglePin(LED1_PORT, LED1_PIN);
}
// LED指示风机状态
void LEDCtrlStatus(void)
{
    static uint8_t su8Flash[2] = {0};

    //	if(g_sVariable.gPara.u16RS==ABNORMAL_VALUE)//有输出,闪烁5次
    //	{
    //			led_open();
    //	}
    //	else
    // if (g_sVariable.status.alarm_bitmap[0])  //有输出,闪烁5次
    //    {
    //        led_toggle();
    //    }
    //    else  //闪烁1次
    {
        if (++su8Flash[0] >= 5)
        {
            su8Flash[0] = 0;
            led_toggle();
        }
    }
    return;
}
