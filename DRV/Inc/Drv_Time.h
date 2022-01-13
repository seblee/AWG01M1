#ifndef __DRV_TIME_H
#define __DRV_TIME_H

#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <macro.h>

extern BOOL TimersInit_14(uint16_t timPeriod, uint8_t u8Type);
extern void TimersEnable_14(void);
extern void TimersDisable_14(void);
void EnableT_14(void);

#endif /* __DRV_TIME_H */
