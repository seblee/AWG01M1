/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //SPI相关驱动函数
  Version:         //V1.0
  Function List:   //IIC_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/26    1.0     build this moudle
***********************************************************/

#include "Drv_SPI.h"
#include "stm32g0xx_hal.h"
#include "stm32f0xx_misc.h"
#include "macro.h"
#include <stdio.h>
#include "systick.h"
#include <Lib_Check.h>
#include <Lib_Delay.h>
#include <global.h>

//#define HARDSPI 1
/*************************************************
  Function:       // SPI_Configuration
  Description:    // SPI初始化相关配置函数

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void SPI_Configuration(void)
{
#ifdef HARDSPI

    GPIO_InitTypeDef GPIO_InitStruct;
    SPI_InitTypeDef SPI_InitStruct;

    /*!< SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO
         and SD_SPI_SCK_GPIO Periph clock enable */
    // 	 RCC_AHBPeriphClockCmd(FLASH_CS_PIN_SCK|FLASH_SCK_PIN_SCK|FLASH_MISO_PIN_SCK | FLASH_MOSI_PIN_SCK, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    /*!< SD_SPI Periph clock enable */

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    /* Connect PXx to SD_SPI_SCK */
    GPIO_PinAFConfig(SPI_SCK_PORT, SPI_SCK_SOURCE, SPI_SCK_AF);

    /* Connect PXx to SD_SPI_MISO */
    GPIO_PinAFConfig(SPI_MISO_PORT, SPI_MISO_SOURCE, SPI_MISO_AF);

    /* Connect PXx to SD_SPI_MOSI */
    GPIO_PinAFConfig(SPI_MOSI_PORT, SPI_MOSI_SOURCE, SPI_MOSI_AF);

    /*!< Configure SD_SPI pins: SCK */
    GPIO_InitStruct.GPIO_Pin   = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(SPI_SCK_PORT, &GPIO_InitStruct);

    /*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
    GPIO_InitStruct.GPIO_Pin   = SPI_CS_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);

    SPI_CS_HIGH();

    /*!< SD_SPI Config */
    SPI_InitStruct.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL              = SPI_CPOL_High;
    SPI_InitStruct.SPI_CPHA              = SPI_CPHA_2Edge;
    SPI_InitStruct.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
    SPI_InitStruct.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI1, &SPI_InitStruct);
    SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);
    SPI_Cmd(SPI1, ENABLE); /*!< SD_SPI enable */

#else
    GPIO_InitTypeDef GPIO_InitStruct;

    /*!< SD_SPI_CS_GPIO, SD_SPI_MOSI_GPIO, SD_SPI_MISO_GPIO, SD_SPI_DETECT_GPIO
         and SD_SPI_SCK_GPIO Periph clock enable */
    // 	 RCC_AHBPeriphClockCmd(FLASH_CS_PIN_SCK|FLASH_SCK_PIN_SCK|FLASH_MISO_PIN_SCK | FLASH_MOSI_PIN_SCK, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    /*!< SD_SPI Periph clock enable */

    /*!< Configure SD_SPI pins: SCK */
    GPIO_InitStruct.GPIO_Pin   = SPI_SCK_PIN | SPI_MOSI_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    //  GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(SPI_SCK_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = SPI_MISO_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;  //
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*!< Configure SD_SPI_CS_PIN pin: SD Card CS pin */
    GPIO_InitStruct.GPIO_Pin   = SPI_CS_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_1;
    GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);

    SPI_CS_HIGH();

#endif
}

/*******************************************************************************
  Function:       // SPI_SendByte
  Description:    // Sends a byte through the SPI interface and return the byte
                                            received from the SPI bus.
  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // byte to send.
  Output:         // None
  Return:         // The value of the received byte.
  Others:         // None
*******************************************************************************/
INT8U SPI_SendByte(INT8U Byte)
{
    /* Loop while DR register in not emplty */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
        ;

    /* Send byte through the SPI1 peripheral */
    SPI_SendData8(SPI1, Byte);

    /* Wait to receive a byte */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
        ;

    /* Return the byte read from the SPI bus */
    return SPI_ReceiveData8(SPI1);
}
/*******************************************************************************
  Function:       // SPI_ReadByte
  Description:    // received byte from the SPI bus.

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // The value of the received byte.
  Others:         // None
*******************************************************************************/
INT8U SPI_ReadByte(void)
{
    //  return (SPI_SendByte(Dummy_Byte));
    unsigned char Data = 0;
    /*  ?????????? */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
    {
    }
    /*  ???? */
    SPI_SendData8(SPI1, 0xFF);

    /* ?????????? */
    while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
    {
    }
    /* ???????? */
    Data = SPI_ReceiveData8(SPI1);
    return Data;
}

INT8U SPIWriteByte(INT8U Byte)
{
    INT8U i;

    for (i = 0; i < 8; i++)
    {
        SPI_SCLK_HIGH();
        if (Byte & 0x80)
        {
            SPI_MOSI_HIGH();
        }
        else
        {
            SPI_MOSI_LOW();
        }

        // 50US
        Delay(10);
        SPI_SCLK_LOW();
        Byte <<= 1;
        // 5US
        Delay(10);
    }
    return TRUE;
}

INT8U SPIReadByte(void)
{
    INT8U i;
    INT8U j;
    INT8U RevHBit;
    INT8U Byte;

    Byte    = 0;
    RevHBit = 0;

    for (i = 0; i < 8; i++)
    {
        // SPI?????,????????G????i?
        SPI_SCLK_HIGH();

        // 50US
        Delay(10);
        for (j = 0; j < 6; j++)
        {
            if (SPI_MISO_READ)
            {
                RevHBit++;
            }
        }

        // SPI??????????????DOUT??
        SPI_SCLK_LOW();
        Byte <<= 1;
        // ???????GG??,??50%??
        if (RevHBit > 3)
        {
            Byte |= 0x01;
        }
        RevHBit = 0;
        // 5US
        Delay(10);
    }
    return Byte;
}
