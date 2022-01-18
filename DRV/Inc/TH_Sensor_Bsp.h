#ifndef _TH_SENSOR_BSP_H
#define _TH_SENSOR_BSP_H
#include "stm32g0xx_hal.h"
#include "stdint.h"
//#include "sys_conf.h"
#include "sys_def.h"
/******************************************************************************************************/
/******************************************************************************************************/
/*************************************IIC_AM2311 Temperature&Humidity**********************************/
/******************************************************************************************************/
/******************************************************************************************************/
#define AM2301B 1  // AM2301B双线IIC

#define II_AM_SDA_00_Pin GPIO_PIN_11
#define II_AM_SDA_00_GPIO GPIOA

#define II_AM_SDA_01_Pin GPIO_PIN_10
#define II_AM_SDA_01_GPIO GPIOA

#define II_AM_SDA_Pin GPIO_PIN_10
#define II_AM_SDA_GPIO GPIOA
#define II_AM_SCL_Pin GPIO_PIN_11
#define II_AM_SCL_GPIO GPIOA

#define IIC_SDA_READ() HAL_GPIO_ReadPin(II_AM_SDA_GPIO, II_AM_SDA_Pin)

#define ERROR_CNT_MAX 60  //异常次数

enum
{
    HAC01S1 = 0,
    HAC02S1,  // AM2311A
    HAC02S3,  // AM2301B
};

typedef struct
{
    uint16_t Temp;
    uint16_t Hum;
    uint16_t u16Dew;
} Com_tnh_st;

extern void AM_Init(void);
extern uint8_t Read_Sensor(uint16_t *u16TH_Buff, uint8_t u8SN);
extern void AM_Sensor_update(void);
extern uint8_t Get_TH(void);
extern void AHT20_Init(void);  //初始化AHT20
#endif
