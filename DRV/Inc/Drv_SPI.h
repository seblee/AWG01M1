#ifndef __DRV_SPI_H
#define __DRV_SPI_H

#include "stm32g0xx_hal.h"
#include <macro.h>
#include <stdio.h>

#define SPI_CS_PIN GPIO_PIN_4
#define SPI_CS_PORT GPIOA

#define SPI_SCK_PIN GPIO_PIN_5
#define SPI_SCK_PORT GPIOA
#define SPI_SCK_SOURCE GPIO_PinSource5
#define SPI_SCK_AF GPIO_AF_0

#define SPI_MISO_PIN GPIO_PIN_6
#define SPI_MISO_PORT GPIOA
#define SPI_MISO_SOURCE GPIO_PinSource6
#define SPI_MISO_AF GPIO_AF_0

#define SPI_MOSI_PIN GPIO_PIN_7
#define SPI_MOSI_PORT GPIOA
#define SPI_MOSI_SOURCE GPIO_PinSource7
#define SPI_MOSI_AF GPIO_AF_0

#define SPI_CS_LOW() GPIO_ResetBits(SPI_CS_PORT, SPI_CS_PIN)
#define SPI_CS_HIGH() GPIO_SetBits(SPI_CS_PORT, SPI_CS_PIN)

#define SPI_SCLK_LOW() GPIO_ResetBits(SPI_SCK_PORT, SPI_SCK_PIN)
#define SPI_SCLK_HIGH() GPIO_SetBits(SPI_SCK_PORT, SPI_SCK_PIN)

#define SPI_MOSI_LOW() GPIO_ResetBits(SPI_MOSI_PORT, SPI_MOSI_PIN)
#define SPI_MOSI_HIGH() GPIO_SetBits(SPI_MOSI_PORT, SPI_MOSI_PIN)

#define SPI_MISO_READ GPIO_ReadInputDataBit(GPIOA, GPIO_PIN_6)

#define Dummy_Byte 0xFF

extern void SPI_Configuration(void);
extern INT8U SPI_SendByte(INT8U Byte);
extern INT8U SPI_ReadByte(void);
extern INT8U SPIWriteByte(INT8U Byte);
extern INT8U SPIReadByte(void);

#endif /* __DRV_SPI_H */
