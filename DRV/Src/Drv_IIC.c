/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //IIC相关驱动函数
  Version:         //V1.0
  Function List:   //IIC_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/15    1.0     build this moudle
***********************************************************/

#include "Drv_IIC.h"
#include "stm32g0xx_hal.h"
#include "stm32f0xx_misc.h"
#include "macro.h"
#include <stdio.h>

#define HTU21ADDR 0x80

/*************************************************
  Function:       // Delay_us
  Description:    // 微秒延时函数
  Calls:          // Delay_us

  Called By:      // IIC_Init
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void Delay_us(INT16U u8time)
{
    INT8U i;

    while (u8time--)
    {
        for (i = 0; i < 4; i++)
            ;
    }
}

/*************************************************
  Function:       // Delay_ms
  Description:    // 毫秒延时函数
  Calls:          // Delay_us

  Called By:      // IIC_Init
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void Delay_ms(INT16U u8time)
{
    INT32U u32Temp = 0;

    u32Temp = u8time * 1000;
    while (u32Temp--)
    {
        Delay_us(1);
    }
}
/*************************************************
  Function:       // Delay
  Description:    // 延时函数
  Calls:          // Delay_us

  Called By:      // IIC_Init
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_Delay(INT16U u8time)
{
    Delay_us(u8time);
}

/*************************************************
  Function:       // IIC_Configuration
  Description:    // IIC配置函数
  Calls:          // None

  Called By:      // IIC_Init
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void IIC_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*************************************************
  Function:       // IIC_Init
  Description:    // IIC初始化函数
  Calls:          // IIC_Configuration()

  Called By:      // TransducerInit
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void IIC_Init(void)
{
    IIC_Configuration();  // IIC 配置
}

/*************************************************
  Function:       // I2C_SDA_OUT
  Description:    //设置I2C输出
  Calls:          // GPIO_Init()

  Called By:      // I2C_Start
                                    //I2C_Stop
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_uInitStructure;

    GPIO_uInitStructure.GPIO_Pin   = GPIO_PIN_10;
    GPIO_uInitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_uInitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_uInitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
    GPIO_Init(GPIOA, &GPIO_uInitStructure);
}
/*************************************************
  Function:       // I2C_SDA_IN
  Description:    //设置I2C输入
  Calls:          // GPIO_Init()

  Called By:      // I2C_ReciveByte
                                    //I2C_Slave_ACK
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_uInitStructure;

    GPIO_uInitStructure.GPIO_Pin   = GPIO_PIN_10;
    GPIO_uInitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_uInitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_uInitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_uInitStructure.GPIO_Speed = GPIO_Speed_Level_3;  // 10M
    GPIO_Init(GPIOA, &GPIO_uInitStructure);
}

/*************************************************
  Function:       // I2C_Start
  Description:    //I2C启动
  Calls:          // I2C_SDA_OUT()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_Start(void)
{
    I2C_SDA_OUT();

    SDA_H;
    I2C_Delay(5);
    SCL_H;
    I2C_Delay(5);
    SDA_L;
    I2C_Delay(5);
    SCL_L;
}
/*************************************************
  Function:       // I2C_Stop
  Description:    //I2C停止
  Calls:          // I2C_SDA_OUT()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_Stop(void)
{
    I2C_SDA_OUT();

    SDA_L;
    I2C_Delay(5);
    SCL_H;
    I2C_Delay(5);
    SDA_H;
    I2C_Delay(5);
}

/*************************************************
  Function:       // I2C_Ack
  Description:    //I2C应答
  Calls:          // I2C_SDA_OUT()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_Ack(void)
{
    I2C_SDA_OUT();

    SCL_L;
    I2C_Delay(5);
    SDA_L;
    I2C_Delay(5);
    SCL_H;
    I2C_Delay(5);
    SCL_L;
}

/*************************************************
  Function:       // I2C_Nack
  Description:    //I2C应答
  Calls:          // I2C_SDA_OUT()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_Nack(void)
{
    I2C_SDA_OUT();

    SCL_L;
    I2C_Delay(5);
    SDA_H;
    I2C_Delay(5);
    SCL_H;
    I2C_Delay(5);
    SCL_L;
}

/*************************************************
  Function:       // I2C_SendByte
  Description:    //I2C发送数据
  Calls:          // I2C_SDA_OUT()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // u8TXDByte，发送数据
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
INT8U I2C_Slave_ACK(void)
{
    INT8U RecAck = 0;
    INT8U Rec    = 0;

    I2C_SDA_IN();
    SCL_L;
    SCL_H;
    I2C_Delay(5);
    Rec = (INT8U)SDA_READ;

    if (Rec == 0)
    {
        RecAck = TRUE;
    }
    else
    {
        RecAck = FALSE;
    }
    SCL_L;
    I2C_Delay(5);
    return (RecAck);
}

/*************************************************
  Function:       // I2C_SendByte
  Description:    //I2C发送数据
  Calls:          // I2C_SDA_OUT()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // u8TXDByte，发送数据
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void I2C_SendByte(INT8U u8TXDByte)
{
    INT8U i;

    I2C_SDA_OUT();

    for (i = 0; i < 8; i++)
    {
        if (u8TXDByte & 0x80)
        {
            SDA_H;
        }
        else
        {
            SDA_L;
        }

        u8TXDByte <<= 1;
        I2C_Delay(5);
        SCL_L;
        I2C_Delay(5);
        SCL_H;
        I2C_Delay(5);
        SCL_L;
    }
    //  return I2C_Slave_ACK();
}

/*************************************************
  Function:       // I2C_ReciveByte
  Description:    // 接收I2C数据
  Calls:          // I2C_SDA_IN()
                                          I2C_Delay()
  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // u8RDByte,
  Others:         // None
*************************************************/
INT8U I2C_ReciveByte(void)
{
    INT8U u8RDByte = 0;
    INT8U i;

    I2C_SDA_IN();

    for (i = 0; i < 8; i++)
    {
        SCL_H;
        I2C_Delay(5);
        u8RDByte <<= 1;
        u8RDByte |= SDA_READ;
        SCL_L;
        I2C_Delay(5);
    }
    return u8RDByte;
}

/*************************************************
  Function:       // TransducerInit
  Description:    // Transducer初始化函数
  Calls:          // IIC_Init()
                                          I2C_Start()
                                          I2C_SendByte()
                                          I2C_ReciveByte()
                                          I2C_Stop()
                                          I2C_Delay()
  Called By:      // TemperatureHumidity_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void TransducerInit(void)
{
    IIC_Init();                      // I2C初始化
    I2C_Start();                     //启动I2C
    I2C_SendByte(HTU21ADDR & 0xFE);  //?HTU21I2C??
    I2C_ReciveByte();
    I2C_Stop();      //停止I2C
    I2C_Delay(600);  //延时
}

/*************************************************
  Function:       // ReadRes
  Description:    // 用于读取HTU21用户寄存器值
  Calls:          //
                                          I2C_Start()
                                          I2C_SendByte()
                                          I2C_ReciveByte()
                                          I2C_Stop()
                                          I2C_Nack()
  Called By:      // None
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // 返回8位的HTU21用户寄存器值
  Others:         // None
*************************************************/
INT8U ReadRes(void)
{
    INT8U u8Temp = 0;

    I2C_Start();
    I2C_SendByte(0x80);
    if (TRUE == I2C_Slave_ACK())
    {
        I2C_SendByte(0xe7);
    }

    if (TRUE == I2C_Slave_ACK())
    {
        I2C_Start();
        I2C_SendByte(0x81);
    }
    if (TRUE == I2C_Slave_ACK())
    {
        u8Temp = I2C_ReciveByte();
    }
    I2C_Nack();
    I2C_Stop();
    return u8Temp;
}
/*************************************************
  Function:       // WriteRes
  Description:    // 用于写HTU21用户寄存器
  Calls:          //
                                          I2C_Start()
                                          I2C_SendByte()
                                          I2C_ReciveByte()
                                          I2C_Stop()
  Called By:      // None
  Table Accessed: // None
  Table Updated:  // None
  Input:          // u8Res，写HTU21寄存器的数据
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void WriteRes(INT8U u8Res)
{
    I2C_Start();
    I2C_SendByte(0x80);
    if (TRUE == I2C_Slave_ACK())
    {
        I2C_SendByte(0xe6);
    }
    if (TRUE == I2C_Slave_ACK())
    {
        I2C_SendByte(u8Res);
    }
    if (TRUE == I2C_Slave_ACK())
    {
        I2C_Stop();
    }
}
/*************************************************
  Function:       // SoftReset
  Description:    // 用于用于软复位HTU21
  Calls:          //
                                          I2C_Start()
                                          I2C_SendByte()
                                          I2C_ReciveByte()
                                          I2C_Stop()
  Called By:      // None
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void SoftReset(void)
{
    I2C_Start();
    I2C_SendByte(0x80);
    if (TRUE == I2C_Slave_ACK())
    {
        I2C_SendByte(0xFE);
    }
    if (TRUE == I2C_Slave_ACK())
    {
        I2C_Stop();
    }
    Delay_ms(30);
}

/*************************************************
  Function:       // CheckCrc8
  Description:    // CheckCrc8温湿度数据CRC8校验
  Calls:          // None

  Called By:      // ReadTempRh
  Table Accessed: // None
  Table Updated:  // None
  Input:          // pBuff，u8Length，u8Check
  Output:         // None
  Return:         // TRUE/FALSE?
  Others:         // None
*************************************************/
INT8U CheckCrc8(INT8U *pBuff, INT8U u8Length, INT8U u8Check)
{
    INT8U u8Crc = 0;
    INT8U j;
    INT8U i;

    for (j = 0; j < u8Length; ++j)
    {
        u8Crc ^= (pBuff[j]);
        for (i = 8; i > 0; --i)
        {
            if (u8Crc & 0x80)
            {
                u8Crc = (u8Crc << 1) ^ 0x131;
            }
            else
            {
                u8Crc = (u8Crc << 1);
            }
        }
    }
    if (u8Crc == u8Check)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

/*************************************************
  Function:       // ReadTempRh
  Description:    // ReadTempRh温湿度采集函数
  Calls:          // I2C_Start()
                                          I2C_SendByte()
                                          I2C_Slave_ACK()
                                          I2C_ReciveByte()
  Called By:      // TemperatureHumidity
  Table Accessed: // None
  Table Updated:  // None
  Input:          // u8Temprh，读取温度、湿度命令
  Output:         // None
  Return:         // i16TempRh,返回温湿度值
  Others:         // None
*************************************************/
INT16 ReadTempRh(INT8U u8Temprh)
{
    INT8U u8Temp[2]     = {0};
    INT8U u8Crctemp     = 0;
    INT16U u16Tempvalue = 0;
    INT16 i16TempRh     = 0x7FF;
    float fTempRh       = 0;

    if ((u8Temprh == T_SCMD) || (u8Temprh == RH_SCMD))
    {
        I2C_Start();
        I2C_SendByte(HTU21ADDR & 0xFE);  //写I2C地址

        if (TRUE == I2C_Slave_ACK())  //应当ACK
        {
            I2C_SendByte(u8Temprh);  //写I2C命令

            if (TRUE == I2C_Slave_ACK())
            {
                I2C_Start();
                I2C_SendByte(HTU21ADDR | 0x01);  //写I2C地址

                if (TRUE == I2C_Slave_ACK())
                {
                    Delay_ms(15);  //延时，等待测量数据

                    u8Temp[0] = I2C_ReciveByte();  //读I2C数据
                    I2C_Ack();
                    u8Temp[1] = I2C_ReciveByte();
                    I2C_Ack();
                    u8Crctemp = I2C_ReciveByte();  //读CRC8校验数据
                    I2C_Nack();
                    I2C_Stop();
                    if ((u8Temp[0] == 0) && (u8Temp[1] == 0) && (u8Crctemp == 0))
                    {
                        return i16TempRh;
                    }
                    if (TRUE == (CheckCrc8(u8Temp, 2, u8Crctemp)))  // crc8check
                    {
                        u16Tempvalue = u8Temp[0] << 8;
                        u16Tempvalue |= u8Temp[1];
                        u16Tempvalue &= ~0x0003;
                        //												//算法调整，使用浮点小数乘法会死机，很奇怪？
                        if (u8Temprh == T_SCMD)
                        {
                            //														temperature = -46.85 +
                            // 175.72/65536*(float)tempvalue;
                            // Uptemperature=temperature*10;
                            fTempRh = (float)u16Tempvalue;
                            fTempRh /= 65536;
                            fTempRh *= 17572;
                            fTempRh -= 4685;
                            fTempRh /= 10;
                            //														fTempRh	=-400;		//TEST
                            i16TempRh = fTempRh;
                        }
                        else if (u8Temprh == RH_SCMD)  //
                        {
                            //														fTempRh=-6+125.00/65536*(float)u16Tempvalue;
                            //														i16TempRh=fTempRh*10;
                            fTempRh = (float)u16Tempvalue;
                            fTempRh /= 65536;
                            fTempRh *= 12500;
                            fTempRh -= 600;
                            fTempRh /= 10;
                            i16TempRh = fTempRh;
                            if (i16TempRh >= 999)
                            {
                                i16TempRh = 999;
                            }
                        }
                    }
                }
            }
        }
    }
    //		else
    //		{
    //				i16TempRh = 0x01;
    //		}

    return i16TempRh;
}
