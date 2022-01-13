#ifndef __APP_TEMPREATUREHUMIDITY_H
#define __APP_TEMPREATUREHUMIDITY_H

#include "stm32g0xx_hal.h"
#include "macro.h"

//温湿度结构体
typedef struct
{
    INT16U u16State;              //温湿度板状态
    INT16U_UNION u16Temperature;  //温度
    INT16U_UNION u16Humidity;     //湿度
} sTemperatureHumidity;

extern void VariableInit_TH(void);

#endif /* APP_TEMPREATUREHUMIDITY_H */
