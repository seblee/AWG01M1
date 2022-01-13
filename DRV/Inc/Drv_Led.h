#ifndef __LED_H
#define __LED_H

#include "stm32g0xx_hal.h"
#define LED1_PIN GPIO_PIN_13
#define LED1_PORT GPIOC
void led_init(void);
void led_open(void);
void led_close(void);
void led_toggle(void);
extern void LEDCtrlStatus(void);
extern void TEST_toggle(void);
#endif /* __LED_H */
