#ifndef __DRV_PWM_H
#define __DRV_PWM_H

#include "stm32g0xx_hal.h"
#include <macro.h>
#include <stdio.h>

#define PWM_1KHZ 1000000  // 1K HZ
#define MIN_AO 10
extern void Drv_PWM_Init(void);
extern void PWM_AO_update(void);

#endif
