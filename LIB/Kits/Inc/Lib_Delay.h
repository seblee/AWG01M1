#ifndef _DELAY_H_
#define _DELAY_H_

#include "stm32g0xx_hal.h"
#include "tim.h"

extern void Delay(uint16_t DelayValue);
extern void delay_us(uint16_t nus);
#endif
