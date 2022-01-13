#ifndef __DRV_IIC_H
#define __DRV_IIC_H

#include "stm32g0xx_hal.h"
#include <macro.h>
#include <stdio.h>

#define SCL_L GPIOA->BRR = GPIO_PIN_9
#define SCL_H GPIOA->BSRR = GPIO_PIN_9
#define SDA_L GPIOA->BRR = GPIO_PIN_10
#define SDA_H GPIOA->BSRR = GPIO_PIN_10
#define SDA_READ (GPIOA->IDR & GPIO_PIN_10) >> 10

#define I2C_SDA_GPIO GPIOA
#define I2C_SCL_GPIO GPIOA
#define I2C_SDA_PIN GPIO_PIN_10
#define I2C_SCL_PIN GPIO_PIN_9

//#define SCL_L          		      GPIO_ResetBits(I2C_SCL_GPIO, I2C_SCL_PIN)
//#define SCL_H          		      GPIO_SetBits(I2C_SCL_GPIO, I2C_SCL_PIN)
//#define SDA_L          		      GPIO_ResetBits(I2C_SDA_GPIO, I2C_SDA_PIN)
//#define SDA_H          		      GPIO_SetBits(I2C_SDA_GPIO, I2C_SDA_PIN)
//#define SDA_READ                GPIO_ReadInputDataBit(I2C_SDA_GPIO, I2C_SDA_PIN)
#define T_SCMD 0xE3   //温度、非主机命令
#define RH_SCMD 0xE5  //湿度、非主机命令
//#define T_SCMD 0xF3		//温度、非主机命令
//#define RH_SCMD 0xF5	//湿度、非主机命令

extern void TransducerInit(void);
extern INT16 ReadTempRh(INT8U TH);
extern void IIC_Init(void);

#endif /* __DRV_IIC_H */
