#include "adc.h"
//#include "calc.h"
//#include "global_var.h"
#include "global.h"
#include "cmsis_os.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address 0x4001244C  // ADC1->DR  // 0x40012440  // 0x4001244C
//__IO uint16_t ADC1Buff[AI_MAX_CNT * MAX_ADBUFEVERY];
// uint16_t ADC1ConvertedValue[AI_MAX_CNT];
//#define GD32E230_CHIP
//__IO uint16_t ADC1Buff[NUM_2][ADC1_PER];
volatile uint16_t ADC1ConvertedValue[AI_MAX_CNT];

// uint16_t Filter[ADC1_PER][NUM_2];
extern ADC_HandleTypeDef hadc1;
void drv_adc_dma_init(void)
{
#ifdef STM32F030_CHIP
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStruct;
    DMA_InitTypeDef DMA_InitStruct;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    ADC_ClockModeConfig(ADC1, ADC_ClockMode_SynClkDiv4);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    /*ADC GPIO CONFIG*/
    //	GPIO_InitStruct.GPIO_Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5;
    GPIO_InitStruct.GPIO_Pin  = GPIO_PIN_0 | GPIO_PIN_5;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    ADC_DeInit(ADC1);
    ADC_StructInit(&ADC_InitStruct);
    ADC_InitStruct.ADC_Resolution           = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ContinuousConvMode   = ENABLE;
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign            = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_ScanDirection        = ADC_ScanDirection_Upward;  // CH0-CH16
    ADC_Init(ADC1, &ADC_InitStruct);
    ADC_ChannelConfig(ADC1, ADC_Channel_0 | ADC_Channel_5, ADC_SampleTime_239_5Cycles);
    ADC_GetCalibrationFactor(ADC1);
    ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY))
        ;
    ADC_StartOfConversion(ADC1);

    /* DMA1 Channel1 Config */
    DMA_DeInit(DMA1_Channel1);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;
    DMA_InitStruct.DMA_MemoryBaseAddr     = (uint32_t)ADC1Buff;
    DMA_InitStruct.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStruct.DMA_BufferSize         = AI_MAX_CNT * NUM_2;
    DMA_InitStruct.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStruct.DMA_Priority           = DMA_Priority_High;
    DMA_InitStruct.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStruct);
    /* DMA1 Channel1 enable */
    DMA_Cmd(DMA1_Channel1, ENABLE);
    /* Test DMA1 TC flag */
    while ((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET)
        ;
    /* Clear DMA TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);
#endif

#ifdef GD32E230_CHIP
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    /* Enable peripheral clocks --------------------------------------------------*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_DMA1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_ADCCLK_PCLK_Div4);
    /* Configure PC.01 and PC.04 (Channel11 and Channel14) as analog input -----*/
    GPIO_InitStructure.GPIO_Pin =
        GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    DMA_DeInit(DMA1_Channel1);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)ADC1ConvertedValue;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize         = AI_MAX_CNT;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel1, &DMA_InitStructure);

    /* Enable DMA1 channel1 */
    DMA_Cmd(DMA1_Channel1, ENABLE);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode       = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel       = AI_MAX_CNT;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channel11, channel14, channel16 and channel17 configurations */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 6, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 7, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 8, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 9, ADC_SampleTime_239Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 10, ADC_SampleTime_239Cycles5);
    /* Enable ADC1 DMA */

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);

    /* Enable TempSensor and Vrefint channels: channel16 and Channel17 */
    ADC_TempSensorVrefintCmd(ENABLE);

    /* Enable ADC1 reset calibaration register */
    ADC_ResetCalibration(ADC1);

    /* Check the end of ADC1 reset calibration register */
    while (ADC_GetResetCalibrationStatus(ADC1))
        ;
    /* Start ADC1 calibaration */
    ADC_StartCalibration(ADC1);

    /* Check the end of ADC1 calibration */
    while (ADC_GetCalibrationStatus(ADC1))
        ;
    ADC_DMACmd(ADC1, ENABLE);
    /* Start ADC1 Software Conversion */
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* Test on Channel 1 DMA1_FLAG_TC flag */
    while (!DMA_GetFlagStatus(DMA1_FLAG_TC1))
        ;
    /* Clear Channel 1 DMA1_FLAG_TC flag */
    DMA_ClearFlag(DMA1_FLAG_TC1);

#endif

    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC1ConvertedValue, AI_MAX_CNT);
}

// void ADCValProcess(uint16_t *ptrADCval, uint16_t *ptrADCbuf, uint8_t index)
//{
//     uint8_t i                     = 0;
//     volatile uint32_t ADC_VOL_ave = 0;
//     uint16_t ADC_Tmp[MAX_ADBUFEVERY];
//     for (i = 0; i < MAX_ADBUFEVERY; i++)
//     {
//         ADC_Tmp[i] = 0x0000;
//         ADC_Tmp[i] = ptrADCbuf[i * AI_MAX_CNT + index];
//     }
//     quick(ADC_Tmp, 0, 19);  // 0~35¡À¨ºo?,1236??
//     for (i = 2; i < 18; i++)
//     {
//         ADC_VOL_ave += ADC_Tmp[i];
//     }
//     ADC_VOL_ave >>= 4;
//     ptrADCval[index] = ADC_VOL_ave;
// }
