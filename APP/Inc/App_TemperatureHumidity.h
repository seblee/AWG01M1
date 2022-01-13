#ifndef __APP_TEMPREATUREHUMIDITY_H
#define __APP_TEMPREATUREHUMIDITY_H

#include "stm32g0xx_hal.h"
#include "macro.h"

//��ʪ�Ƚṹ��
typedef struct
{
    INT16U u16State;              //��ʪ�Ȱ�״̬
    INT16U_UNION u16Temperature;  //�¶�
    INT16U_UNION u16Humidity;     //ʪ��
} sTemperatureHumidity;

extern void VariableInit_TH(void);

#endif /* APP_TEMPREATUREHUMIDITY_H */
