#ifndef __ADC_H
#define __ADC_H

#include "stm32g0xx_hal.h"
#define MAX_ADBUFEVERY 20
enum
{
    ADC_T1 = 0,
};
#define NUM_2 10  //滤波次数

enum
{
    AI_NTC1 = 0,
    AI_NTC2,
    AI_NTC3,
    AI_NTC4,
    AI_NTC5,
    AI_NTC6,
    AI_SENSOR1,
    AI_SENSOR2,
    AI_SENSOR3,
    AI_SENSOR4,
    AI_MAX_CNT
};
#define AI_NTC_MAX_CNT AI_SENSOR1  // NTC数量
#define ADC1_PER AI_MAX_CNT        //通道数

#define AI_NTC_Hot AI_NTC1   //热水
#define AI_NTC_Cold AI_NTC2  //冷水

#define AI_NTC_Defrost AI_NTC3  //除霜
#define AI_NTC_Warm AI_NTC4     //高温

#define AI_NTC_DTANK AI_NTC5  //饮水箱
#define AI_NTC_STANK AI_NTC6  //源水箱

#define AI_CT_I AI_SENSOR1      // CT
#define AI_LEDUV1_V AI_SENSOR2  // LEDUV1
#define AI_LEDUV3_V AI_SENSOR3  // LEDUV3
#define AI_TDS AI_SENSOR4       // TDS

#define OFFSET_V 6  //
extern volatile uint16_t ADC1ConvertedValue[AI_MAX_CNT];
extern void drv_adc_dma_init(void);
extern void ADCValProcess(uint16_t *ptrADCval, uint16_t *ptrADCbuf, uint8_t index);
#endif /* __ADC_H */
