/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //计量相关操作函数
  Version:         //V1.0
  Function List:   //IIC_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/31    1.0     build this moudle
***********************************************************/

#include "Drv_SPI.h"
#include "stm32f0xx.h"
#include "stm32f0xx_misc.h"
#include "macro.h"
#include <stdio.h>
#include "Lib_Check.h"
#include "Hal_Measure.h"
#include "sys_def.h"
#include "Sys_MemoryMap.h"
#include "string.h"
#include "Lib_CRC.h"
#include "Lib_Delay.h"
#include "Lib_Memory.h"
#include "DRV_TIME.h"
#include "global.h"
#include "App_Power.h"
// ??????
static U32 CheckSum;
static U32 EnergyTickIndex;

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
void MeasureReset_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /*!< Measure Periph clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /*!< Configure MEASURE_RST_PIN,MEASURE_IRQ_PIN */
    GPIO_InitStruct.GPIO_Pin   = MEASURE_RST_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_Init(MEASURE_RST_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin   = MEASURE_IRQ_PIN;
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_Init(MEASURE_IRQ_PORT, &GPIO_InitStruct);
}

/*******************************************************************************
  Function:       // Read_MeasureResgster
  Description:    // reading 24-bits registers from ATT7026E
                                            Considering the compatibilities, reading the 24bits register as
                                            3*8bits, and reads 8bits once

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // The value of the received byte.
  Others:         // None
*******************************************************************************/
void Read_MeasureResgster(unsigned char Address, unsigned char *pValue, char Length)
{
#ifdef HARDSPI
    SPI_CS_HIGH();
    Delay(20);
    SPI_CS_LOW();
    Delay(200);
    SPI_SendByte(Address);  // write address
    Delay(20);
    while (Length--)
    {
        *pValue = SPI_ReadByte();
        pValue++;
    }
    SPI_CS_HIGH();
#else
    SPI_CS_HIGH();
    SPI_SCLK_LOW();
    Delay(20);
    SPI_CS_LOW();
    Delay(200);
    SPIWriteByte(Address);  // write address
    Delay(20);
    while (Length--)
    {
        *pValue = SPIReadByte();
        pValue++;
    }
    SPI_SCLK_LOW();
    SPI_CS_HIGH();
#endif
}
/*******************************************************************************
  Function:       // Write_MeasureResgster
  Description:    // reading 24-bits registers from ATT7026E
                                            Considering the compatibilities, reading the 24bits register as
                                            3*8bits, and reads 8bits once

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // The value of the Write byte.
  Others:         // None
*******************************************************************************/
void Write_MeasureResgster(unsigned char Address, unsigned char *pValue, char Length)
{
#ifdef HARDSPI
    SPI_CS_HIGH();
    Delay(20);
    SPI_CS_LOW();
    Delay(200);
    SPI_SendByte(Address | 0X80);  // write address
    //  SPI_SendData8
    NOP();
    Delay(20);
    while (Length--)
    {
        SPI_SendByte(*pValue);
        pValue++;
    }
    SPI_CS_HIGH();
#else
    SPI_SCLK_LOW();
    Delay(2);
    SPI_CS_HIGH();
    Delay(2);
    SPI_MOSI_LOW();
    Delay(2);
    SPI_CS_LOW();

    Delay(200);
    SPIWriteByte(Address | 0X80);  // write address
    //  SPI_SendData8
    NOP();
    Delay(10);
    while (Length--)
    {
        SPIWriteByte(*pValue);
        pValue++;
    }
    SPI_CS_HIGH();
#endif
}
/*******************************************************************************
  Function:       // ReadMeasureConsole
  Description:    // Control reading 24-bits registers from ATT7026E
                                            Considering the compatibilities, reading the 24bits register as
                                            3*8bits, and reads 8bits once

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // The value of the Rcceived byte.
  Others:         // None
*******************************************************************************/
BOOL ReadMeasureConsole(unsigned char u8Addr, unsigned char *pValue, unsigned char Length)
{
    // unsigned long ucPowerData1, ucPowerData2;
    // unsigned char i;

    // g_sVariable.sModbus_inst.u8Addr = u8Addr;
    // for (i = 0; i < 4; i++)
    // {
    //     Read_MeasureResgster(u8Addr, (unsigned char *)&ucPowerData1, Length);
    //     ucPowerData1 &= 0xFFFFFF;
    //     Read_MeasureResgster(0X2d, (unsigned char *)&ucPowerData2, Length);  //???????
    //     ucPowerData2 &= 0xFFFFFF;
    //     if (ucPowerData1 != ucPowerData2)
    //     {
    //         continue;
    //     }
    //     else
    //     {
    //         for (i = 0; i < 3; i++)
    //             *(pValue + i) = *(((unsigned char *)&ucPowerData1) + i);
    //         return TRUE;
    //     }
    // }
    // if (i == 4)
    // {
    //     pValue = 0;
    //     return FALSE;
    // }
    // return FALSE;
    INT8U pBuffer1[3] = {0}, pBuffer2[3] = {0};
    INT8U i;

    i = 3;
    while (i--)
    {
        Read_MeasureResgster(u8Addr, (U8 *)&pBuffer1, Length);
        //  Delay(100);
        Read_MeasureResgster(0X2d, (U8 *)&pBuffer2, Length);  //???????

        if (StringCompare(&pBuffer1[0], &pBuffer2[0], 3) == EQUAL)
        {
            // for (i = 0; i < Length; i++)
            // {
            // // *(pValue + 1 + i) = pBuffer1[i];  //????
            //     *(pValue + i) = pBuffer1[i];  //????
            // }
            MemoryReverseCopy(pValue, pBuffer1, 3);
            NOP();
            return TRUE;
        }
    }
    if (!i)
    {
        pValue = 0;
        g_sVariable.sPower_inst.u16State |= HARDERR;  //电源板故障
        return FALSE;                                 // ??
    }
    else
    {
        g_sVariable.sPower_inst.u16State &= ~HARDERR;
    }
    return TRUE;
}
/*******************************************************************************
  Function:       // WriteMeasureConsole
  Description:    // Control Writeing 24-bits registers to ATT7026E
                                            Considering the compatibilities, reading the 24bits register as
                                            3*8bits, and reads 8bits once

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // The value of the Write byte.
  Others:         // None
*******************************************************************************/
BOOL WriteMeasureConsole(unsigned char u8Addr, unsigned char *PValue, unsigned char Length)
{
    // unsigned long ucPowerData1, ucPowerData2;
    // unsigned char i;

    // g_sVariable.sModbus_inst.u8Addr = u8Addr;
    // ucPowerData1                    = *(unsigned long *)PValue;
    // for (i = 0; i < 4; i++)
    // {
    //     Write_MeasureResgster(u8Addr, (unsigned char *)&ucPowerData1, Length);
    //     ucPowerData1 &= 0xFFFFFF;
    //     ucPowerData2 = 0x00;
    //     Read_MeasureResgster(0X2d, (unsigned char *)&ucPowerData2, Length);  //???????
    //     ucPowerData2 &= 0xFFFFFF;
    //     if (ucPowerData1 != ucPowerData2)
    //     {
    //         continue;
    //     }
    //     else
    //     {
    //         return TRUE;
    //     }
    // }
    // if (i == 4)
    // {
    //     return FALSE;
    // }
    // return FALSE;
    INT8U pBuffer1[3] = {0}, pBuffer2[3] = {0};
    INT8U i;

    // pBuffer1 = *PValue;
    // for (i = 0; i < Length; i++)
    // {
    //     pBuffer2[i] = *(PValue + i);  //?G�????????
    // }
    // memcpy(pBuffer1, &PValue[0], 3);
    MemoryReverseCopy(pBuffer1, &PValue[0], 3);
    i = 3;
    while (i--)
    {
        Write_MeasureResgster(u8Addr, (unsigned char *)&pBuffer1, Length);
        //  Delay(100);
        memset(pBuffer2, 0x00, 3);
        Read_MeasureResgster(0x2D, (unsigned char *)&pBuffer2, Length);  //???????
        if (StringCompare(&pBuffer1[0], &pBuffer2[0], 3) == EQUAL)
        {
            break;
        }
    }
    if (!i)
    {
        return FALSE;  // ??
    }
    return TRUE;
}

BOOL Measure_HardReset(void)
{
    INT8U CheckCount;
    INT8U Reset_data[3];
    INT8U CheckBack_data[3];

    CheckCount = 3;
    while (CheckCount--)
    {
        //硬件复位
        MEASURE_RST_LOW();
        Delay(5000);  //约25ms
#ifdef ATT7026E
        if (MEASURE_IRQ_READ)  //检测到IRQ为高
        {
            MEASURE_RST_HIGH();
            Delay(5000);

            if (!(MEASURE_IRQ_READ))  //检测到IRQ变低
            {
                //        MEASURE_RST_HIGH();
                NOP();
                break;
            }
        }
#else
        MEASURE_RST_HIGH();
        NOP();
        break;
#endif
    }
    NOP();
    //读取芯片内部型号，判断是否复位成功
    if (CheckCount)
    {
        CheckCount = 3;
        while (CheckCount)
        {
            CheckCount--;
            memset(Reset_data, 0x00, 3);
            //      Reset_data[2]=0x5A;
            memset(CheckBack_data, 0x00, 3);
            Write_MeasureResgster(0xC6, (unsigned char *)&Reset_data[0], 3);
            Delay(500);
            Read_MeasureResgster(0X00, (unsigned char *)&CheckBack_data[0], 3);
            Delay(500);
            if ((CheckBack_data[0] == 0x71) && (CheckBack_data[1] == 0x26) && (CheckBack_data[2] == 0xA0))
            // if ((CheckBack_data[0] == 0x00) && (CheckBack_data[1] == 0xAA) && (CheckBack_data[2] == 0xAA))
            {
                NOP();
                return TRUE;
                //        break;
            }
        }
        if (CheckCount)
        {
            return TRUE;
        }
    }
    return FALSE;
}

//获取校正开关标志
static BOOL GetCalibrateFlag(void)
{
    INT8U_Check Bytes;

    //获取校正标志
    ////// if(Read(DEV_EEPROM,(U8*)(OFFSET(Calibrate_ATT,Calibrate_Flag)+EEP_ADDR_CALIBRATE_START),2,&Bytes.cValue))
    memcpy(&Bytes.cValue, &g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0], 2);
    // if (g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0] ==
    // ~g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[1])
    // {
    //     if (g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0] == 0xA5)
    //     {
    //         return TRUE;
    //     }
    // }
    if (Bytes.cValue == ~Bytes.CheckValue)
    {
        if (Bytes.cValue == 0xA5)
        {
            return TRUE;
        }
    }
    else
    {
        //  if(Read(DEV_EEPROM,(U8*)(OFFSET(Calibrate_ATT,Calibrate_Flag)+EEP_ADDR_CALIBRATE_BACKUP_START),2,&Bytes.cValue))
        Flash_Read(FLASH_ADDR_CALIBRATE_START, (u8 *)&Bytes.cValue, 2);  //读较表标志
        {
            if (Bytes.cValue == ~Bytes.CheckValue)
            {
                if (Bytes.cValue == 0xA5)
                {
                    return TRUE;
                }
            }
        }
    }
    return FALSE;
}

//校验和检查
BOOL Calibrate_CheckSum(U8 *pBuff)  //??????
{
    INT8U i;
    INT32U_UNION RegisterValue;

    //?????
    RegisterValue.lValue = 0x00;
    i                    = 0x03;

    while (i--)
    {
        ReadMeasureConsole(Check_CAL01, (unsigned char *)&RegisterValue.acBytes, 3);
        CheckSum = RegisterValue.lValue;  //??????
        if (StringCompare(&pBuff[0], &RegisterValue.acBytes[1], 3) == EQUAL)
        {
            RegisterValue.lValue = 0x00;
            ReadMeasureConsole(Check_CAL02, (unsigned char *)&RegisterValue.acBytes, 3);
            if (StringCompare(&pBuff[3], &RegisterValue.acBytes[1], 3) == EQUAL)
            {
                return TRUE;
            }
        }
        // TEST
        return TRUE;
    }
    return FALSE;
}

//校验和检查
BOOL Calibrate_CheckSum_Error(void)
{
    INT32U_UNION RegisterValue;

    if (!GetCalibrateFlag())  //未校正，直接返回
    {
        return FALSE;
    }
    RegisterValue.lValue = 0x00;
    if (ReadMeasureConsole(Check_CAL01, (unsigned char *)&RegisterValue.acBytes, 3))
    {
        g_sVariable.sPower_inst.u16PhaseAngle[0] = RegisterValue.lValue;
        if (CheckSum == RegisterValue.lValue)
        {
            return FALSE;
        }
    }
    return TRUE;
}

//清除校正数据
BOOL CalibrateClean(void)
{
    INT8U Buffer[PARA_CAL_LEN];

    //清除存储器中数据TEST
    memset(Buffer, 0x00, PARA_CAL_LEN);
    Flash_Read(FLASH_ADDR_PARAMETER_START, (u8 *)Buffer, PARA_LEN);
    NOP();
    Flash_Write(FLASH_ADDR_PARAMETER_START, (u16 *)Buffer, PARA_CAL_LEN);

    CalibrateInit();
    // Ioctl(DEV_EEPROM, CAL_LEN, EEP_ADDR_CALIBRATE_START);
    // _nop_();
    // //?????????
    // Ioctl(DEV_EEPROM, CAL_LEN, EEP_ADDR_CALIBRATE_BACKUP_START);

    return TRUE;
}

void CalibrateEnable(unsigned char Flag)
{
    INT32U_UNION RegisterValue;

    if (Flag)
    {
        RegisterValue.lValue = 0x5A;
    }
    else
    {
        RegisterValue.lValue = 0x00;
    }

    WriteMeasureConsole(0xC9, (unsigned char *)RegisterValue.lValue, 3);
}

const REG_Default Default_5_60A[] = {
    //
    {0x01, 0xB97E},         // mode
    {0x03, 0xF884},         // EMU???????
    {0x31, 0x3427},         //?????????
                            // {0x02,0x0000}, //ADC增益0x02
                            // {0x02,0x0054}, //ADC增益0x02
    {0x1E, HFconst_5_60A},  //??????HFConst=INT[2.592*10^10*1.163*1.163*0.2197*0.051/(400*220*5)]=2976
                            ////HFConst=INT[2.592*10^10*G*G*Vu*Vi/(EC*Un*Ib)],,G=1.163
    {0x35, 0x000F},         //?????
    // {0x1D,0x0075}, //?????G1????b*0.07%=5*0.07%=
    // {0x1F,0x02C0},
    // {0x1F,0x0016}, //???????G1????n*10%  //?????,???????????????10%(???????)?60%(???????)?
    // {0x37,0x0000}, //?????????G1????
    // {0x04,0x0000}, //A???????
    // {0x05,0x0000}, //B???????
    // {0x06,0x0000}, //C???????
    // {0x07,0x0000}, //A???????
    // {0x08,0x0000}, //B???????
    // {0x09,0x0000}, //C???????
    // {0x0A,0x0000}, //A???????
    // {0x0B,0x0000}, //B???????
    // {0x0C,0x0000}, //C???????
    // {0x0D,0x0000}, //A???GUi0
    // {0x0E,0x0000}, //B???GUi0
    // {0x0F,0x0000}, //C???GUi0
    // {0x10,0x0000}, //A???GUi1
    // {0x11,0x0000}, //B???GUi1
    // {0x12,0x0000}, //C???GUi1
    // {0x13,0x0000}, //A?????offsetGUi
    // {0x14,0x0000}, //B?????offsetGUi
    // {0x15,0x0000}, //C?????offsetGUi
    // {0x16,0x0000}, //??????GUi
    // {0x17,0x0000}, //A?????
    // {0x18,0x0000}, //B?????
    // {0x19,0x0000}, //C?????
    // {0x1A,0x0000}, //A?????
    // {0x1B,0x0000}, //B?????
    // {0x1C,0x0000}, //C?????

    // {0x21,0x0000}, //A?????offsetGUi
    // {0x22,0x0000}, //B?????offsetGUi
    // {0x23,0x0000}, //C?????offsetGUi
    // {0x21,0x0000}, //A????G??ffsetGUi
    // {0x22,0x0000}, //B?????offsetGUi
    // {0x23,0x0000}, //C?????offsetGUi
    // {0x21,0x0000}, //A?????offsetGUi
    // {0x22,0x0000}, //B?????offsetGUi
    // {0x23,0x0000}, //C?????offsetGUi
};

/*******************************************************************************
????:  ?????
????:
????:
????:
*******************************************************************************/
void CalibrateInit(void)
{
    INT8U i;
    INT8U u8Addr;
    INT32U_UNION RegisterValue;

    // RegisterValue.lValue = 0x00;
    // for (u8Addr = 0x01; u8Addr <= 0x71; u8Addr++)
    // //  for(u8Addr =0x01;u8Addr <=0x39;u8Addr++)
    // {
    //     //    WDTCLR();//清看门狗
    //     if (u8Addr == 0x3A)
    //     {
    //         u8Addr = 0x60;
    //     }
    //     WriteMeasureConsole(u8Addr, (U8 *)RegisterValue.acBytes, 3);
    // }
    memset(g_sVariable.sMeasureCalibrate_inst.Calibrate_Register, 0x00, CAL_REGLEN);
    for (i = 0; i < (sizeof(Default_5_60A) / 3); i++)  //加载初始化数据
    {
        Delay(500);
        u8Addr                                                            = (Default_5_60A + i)->REG_Addr;
        RegisterValue.lValue                                              = 0x00;
        RegisterValue.lValue                                              = (Default_5_60A + i)->DefaultValue;
        g_sVariable.sMeasureCalibrate_inst.Calibrate_Register[u8Addr * 2] = RegisterValue.lValue;
        memcpy(&g_sVariable.sMeasureCalibrate_inst.Calibrate_Register[u8Addr * 2], &RegisterValue.acBytes[0], 2);
        WriteMeasureConsole(u8Addr, (unsigned char *)&RegisterValue.lValue, 3);
    }
    // CalibrateEnable(0);//关闭校表
    RegisterValue.lValue = 0x00;
    ReadMeasureConsole(Check_CAL01, (unsigned char *)&RegisterValue.acBytes, 3);
    CheckSum = RegisterValue.lValue;  //保存校验数据
    NOP();
    return;
}

//??????
static BOOL PutMeasureCalibrate(void)
{
    INT8U i;
    INT8U CAL_Flag;
    INT8U pBuff[CAL_LEN];
    INT8U pBuffer[2];
    INT8U nAddr;
    INT32U_UNION RegisterValue;

    // 硬件复位
    if (!Measure_HardReset())
    {
        NOP();
        return FALSE;
    }
    CalibrateInit();  //???,?????????
                      // CalibrateClean();

    if (!GetCalibrateFlag())  // 未校正，直接返回
    {
        return FALSE;
    }

    CAL_Flag = 0x00;
    Cal_16CRC(g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag, CAL_CRC_LEN, pBuffer);
    NOP();
    if (StringCompare(&g_sVariable.sMeasureCalibrate_inst.Calibrate_CRC[0], &pBuffer[0], 2) == EQUAL)
    {
        CAL_Flag = 0x01;
    }
    else
    {
        Flash_Read(FLASH_ADDR_CALIBRATE_START, (u8 *)g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag, CAL_LEN);
        {
            // CRC??
            Cal_16CRC(g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag, CAL_CRC_LEN, pBuffer);
            if (StringCompare(g_sVariable.sMeasureCalibrate_inst.Calibrate_CRC, &pBuffer[0], 2) == EQUAL)
            {
                memcpy(&g_sVariable.sMeasureCalibrate_inst.Calibrate_Register[PHASE_A_GAIN],
                       g_sVariable.sMeasureCalibrate_inst.Phase_Register, 6);
                CAL_Flag = 0x02;
            }
        }
    }
    //已校正
    if (CAL_Flag)
    {
        for (i = 0; i <= 0x03; i++)  //只有ABC三相校正数据
        {
            nAddr                = PHASE_A_GAIN + i;
            RegisterValue.lValue = 0x00;
            RegisterValue.lValue = (g_sVariable.sMeasureCalibrate_inst.Phase_Register[i * 2] << 8) + pBuff[i * 2 + 1];
            WriteMeasureConsole(nAddr, (unsigned char *)&RegisterValue.lValue, 3);
        }

        if (Calibrate_CheckSum(g_sVariable.sMeasureCalibrate_inst.Calibrate_Sum1))  //?????
        {
            return TRUE;
        }
    }

    return TRUE;
}

/*************************************************
  Function:       // Measure_Init
  Description:    // Measure初始化

  Called By:      //
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void Measure_Init(void)
{
    INT8U RetryMax;

    MeasureReset_Configuration();  // Measure复位
    SPI_Configuration();           // SPI配置
    TimersInit_14(1000);           //

    // 准备装载校表数据
    RetryMax = 2;
    while (RetryMax--)
    {
        // 装载校表参数
        if (PutMeasureCalibrate())
        {
            // ????????,????????ATT7026E
            break;
        }
    }
    TimersEnable_14();
}

//检查地址
BOOL Calibrate_Check_Storage(INT8 Index, INT16U *pStorage)
{
    //检查地址
    if ((Index >= 0x00) && (Index <= 0x39))
    {
        // (*pStorage) = (U8)(OFFSET(sMeasureCalibrate, Calibrate_Register) +
        //                    (U32)&g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag) +
        //               (Index * 2);
    }
    else if ((Index >= 0x60) && (Index <= 0x71))
    {
        // (*pStorage) = (U8)(OFFSET(sMeasureCalibrate, Calibrate_Register) +
        //                    (U32)&g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag) +
        //               ((Index - (0x60 - 0x40)) * 2);
    }
    else
    {
        return FALSE;
    }
    return TRUE;
}

//写入校表数据(IC和存储器)
BOOL CalibrateWrite(U8 WriteRegister, U16 CAL_Value)
{
    // CheckU16 CaliValue;
    // INT8U pBuff[CAL_LEN];
    INT16U StorageAddress;
    INT32U_UNION u32Buffer;
    INT32U_UNION RegisterValue;
    INT8U Buffer[PARA_CAL_LEN];

    // ????????????
    // WDTCLR();//????
    RegisterValue.lValue = CAL_Value;
    if (Calibrate_Check_Storage(WriteRegister, &StorageAddress))
    {
        //写入数据到相应寄存器
        if (WriteMeasureConsole(WriteRegister, (unsigned char *)&RegisterValue.lValue, 3))  //??3???
        {
            //保存校表数据
            u32Buffer.lValue = CAL_Value;  // Big_Endian大端格式
            // if(Write(DEV_EEPROM,StorageAddress,2,pBuffer.acBytes))
            memcpy(&g_sVariable.sMeasureCalibrate_inst.Calibrate_Register[WriteRegister], u32Buffer.acBytes, 2);
            {
                // memset(pBuff,0x00,CAL_LEN);
                // NOP();
                // //校正标志
                // pBuff[0] = 0xA5;
                // pBuff[1] =~ pBuff[0];
                // Write(DEV_EEPROM,(U8*)(OFFSET(Calibrate_ATT,Calibrate_Flag)+EEP_ADDR_CALIBRATE_START),2,pBuff);
                g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0] = 0xA5;
                g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[1] =
                    ~g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0];
                NOP();
                //????????CRC
                // Read(DEV_EEPROM,(U8*)(OFFSET(Calibrate_ATT,Calibrate_Flag)+EEP_ADDR_CALIBRATE_START),CAL_LEN-2-6,pBuff);
                memcpy(g_sVariable.sMeasureCalibrate_inst.Phase_Register,
                       &g_sVariable.sMeasureCalibrate_inst.Calibrate_Register[WriteRegister], 6);
                Cal_16CRC(g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag, CAL_CRC_LEN,
                          g_sVariable.sMeasureCalibrate_inst.Calibrate_CRC);
                //读取计量校验和1
                RegisterValue.lValue = 0x00;
                ReadMeasureConsole(Check_CAL01, (unsigned char *)&RegisterValue.acBytes, 3);
                NOP();
                CheckSum = RegisterValue.lValue;  //??????
                memcpy(g_sVariable.sMeasureCalibrate_inst.Calibrate_Sum1, &RegisterValue.acBytes[1], 3);

                //清除存储器中数据TEST
                memset(Buffer, 0x00, PARA_CAL_LEN);
                Flash_Read(FLASH_ADDR_PARAMETER_START, (u8 *)Buffer, sizeof(sParameter) + CAL_LEN);
                NOP();
                memcpy(&Buffer[sizeof(sParameter)], &g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag, CAL_LEN);
                Flash_Write(FLASH_ADDR_PARAMETER_START, (u16 *)Buffer, sizeof(sParameter) + CAL_LEN);
                return TRUE;
            }
        }
    }
    return FALSE;
}

// 电压有效值
static void SyncPhaseVoltage(void)
{
    INT8U Phase;
    INT16U varVoltage;
    INT32U_UNION uVoltage;
    float fTemp;

    for (Phase = 0; Phase < PHASE; Phase++)
    //  for(Phase = 0;Phase < 1;Phase ++)
    {
        // if ((Phase == 0x01) && (g_VAR_RUN.Ex_Control_Word[0] & 0x02))  //三相三线，无B相
        // {
        //     Phase++;
        // }
        uVoltage.lValue = 0x00;
        if (!ReadMeasureConsole(READ_A_PHASE_VOLTAGE_RMS + Phase, (U8 *)uVoltage.acBytes, 3))
        {
            continue;
        }
        if ((uVoltage.lValue < VOL_THRESHOLD) || (uVoltage.lValue == 0xFFFFFF))
        {
            uVoltage.lValue = 0x00;
        }
        // ?????
        fTemp = (float)uVoltage.lValue / ATT_RMS;  //????????
        fTemp *= 10;                               //????,??10?
        fTemp *= VOL_DIFFERENCE;                   //????,??10? TEST

        varVoltage = fTemp;
        if ((fTemp - varVoltage) >= 0.5)  //??????
        {
            varVoltage += 1;
        }
        g_sVariable.sPower_inst.u16Voltage[Phase] = varVoltage;  //
        NOP();
    }
    if (g_sVariable.sPower_inst.u16Voltage[0] <= PHASEFALIURE)  // A相断相
    {
        g_sVariable.sPower_inst.u16State |= 0x02;
    }
    else
    {
        g_sVariable.sPower_inst.u16State &= ~0x02;
    }
    NOP();
    if (g_sVariable.sPower_inst.u16Voltage[1] <= PHASEFALIURE)  // B相断相
    {
        g_sVariable.sPower_inst.u16State |= 0x04;
    }
    else
    {
        g_sVariable.sPower_inst.u16State &= ~0x04;
    }
    NOP();
    if (g_sVariable.sPower_inst.u16Voltage[2] <= PHASEFALIURE)  // C相断相
    {
        g_sVariable.sPower_inst.u16State |= 0x08;
    }
    else
    {
        g_sVariable.sPower_inst.u16State &= ~0x08;
    }
    NOP();
    return;
}

//三相电流
static void SyncPhaseCurrent(void)
{
    INT8U Phase;
    INT8U Rated_CURR;
    u16 varCurrent;
    INT32U_UNION uCurrent;
    float fTemp;
    float N_rms;

    for (Phase = 0; Phase < PHASE; Phase++)
    {
        //  if((Phase==0x01)&&(g_VAR_RUN.Ex_Control_Word[0]&0x02)) //????,??B?
        //  {
        //   Phase++;
        //  }
        uCurrent.lValue = 0x00;
        if (!ReadMeasureConsole(READ_A_PHASE_CURRENT_RMS + Phase, (U8 *)uCurrent.acBytes, 3))
        {
            continue;
        }
        if (uCurrent.lValue < CUR_THRESHOLD)
        {
            uCurrent.lValue = 0x00;
        }
        // ?????
        //  Read(DEV_EEPROM,(U8*)(OFFSET(DI_2007Parameter,EEP_Extended_Rated_CURR)+EEP_ADDR_PARAMETER_START),1,&Rated_CURR);//????
        //  if((Rated_CURR==0xFF)||(!Rated_CURR))
        //  {
        N_rms = 6;  //变比12
                    //  }
                    //  else
                    //  {
        //????Ib???????????50mV?,?????????????Vrms,Vrms/2^13???60,??N=60/Ib, Ib=1.5A,N=60/1.5=40,Ib=6A,N=60/6=10
        //????????????25mV?,Vrms/2^13???30,Ib=1.5A,N=30/1.5=20,Ib=6A,N=30/6=5
        //   N_rms =600/Rated_CURR;
        //  }
        fTemp = (float)uCurrent.lValue / ATT_RMS / N_rms;  //????????  Irms = (Vrms/2^13)/N,N=60/Ib
                                                           //  fTemp =(float)uCurrent.lValue >> 13; //????????
                                                           //  fTemp *=1000;//放大1000倍
        fTemp *= CUROFFSET;                                //放大1000倍
        // IC转换电压显示
        if (Phase == (PHASE - 1))
        {
            fTemp /= ItoV;
        }
        varCurrent = (u16)fTemp;
        if ((fTemp - varCurrent) >= 0.5)  //四舍五入
        {
            varCurrent += 1;
        }
        g_sVariable.sPower_inst.u16Current[Phase] = varCurrent;  //
        //  g_sVariable.sPower_inst.u16Current[Phase] = uCurrent.lValue;//
    }
    return;
}

// 电压相角
static void SyncPhaseVoltageAngle(void)
{
    INT8U Phase;
    INT16U u16VoltageAngle;
    INT32U_UNION uVoltageAngle;
    float fTemp;

    for (Phase = 0; Phase < PHASE; Phase++)
    {
        uVoltageAngle.lValue = 0x00;
        if (!ReadMeasureConsole(READ_A_PHASE_VOLTAGE_ANGLE + Phase, (U8 *)uVoltageAngle.acBytes, 3))
        {
            continue;
        }
        if (uVoltageAngle.lValue == 0xFFFFFF)
        {
            uVoltageAngle.lValue = 0x00;
        }
        // ?????
        fTemp = (float)uVoltageAngle.lValue / VOL_ANGLE * 180;  //????????
        fTemp *= 10;                                            //????,??10?

        u16VoltageAngle = fTemp;
        if ((fTemp - u16VoltageAngle) >= 0.5)  //??????
        {
            u16VoltageAngle += 1;
        }
        g_sVariable.sPower_inst.u16PhaseAngle[Phase] = u16VoltageAngle;  //
        NOP();
    }
    return;
}
//标志状态寄存器
static U8 SyncStateRegister(void)
{
    INT32U_UNION uState;
    U8 u8Irq = FALSE;

    if (!ReadMeasureConsole(READ_STATE_REGISTER, (U8 *)uState.acBytes, 3))
    {
        NOP();
    }
    g_sVariable.sPower_inst.PTest.lValue = uState.lValue;
    if (uState.lValue & 0x00000008)  //相序错
    {
        g_sVariable.sPower_inst.u16State |= 0x01;
    }
    else
    {
        g_sVariable.sPower_inst.u16State &= ~0x01;
    }

    if (!(uState.lValue & 0x00000080))  //复位
    {
        u8Irq = TRUE;
    }
    else
    {
        //   MeasureInit(); //电源测量初始化
        SystemReset();  //系统复位
    }

    return u8Irq;
}

// 电网频率
static void SyncGridFrequency(void)
{
    U16 varGridFrequency;
    INT32U_UNION uGridFrequency;
    float fTemp;

    uGridFrequency.lValue = 0x00;
    if (!ReadMeasureConsole(READ_GRID_FREQUENCY, (U8 *)uGridFrequency.acBytes, 3))
    {
        return;
    }
    // ??????
    fTemp            = (float)uGridFrequency.lValue / ATT_RMS;  //????????
    varGridFrequency = fTemp * 10;                              //扩大10倍
    if ((g_sVariable.sPower_inst.u16Voltage[0] == 0x00) && (g_sVariable.sPower_inst.u16Voltage[0] == 0x00) &&
        (g_sVariable.sPower_inst.u16Voltage[0] == 0x00))  //三相电压为0
    {
        varGridFrequency = 0;
    }
    //  if(varGridFrequency<=100)
    //  {
    //    varGridFrequency=0;
    //  }
    //  else if(varGridFrequency<=450)
    //  {
    //    varGridFrequency=450;
    //  }
    //  else if(varGridFrequency>=550)
    //  {
    //    varGridFrequency=550;
    //  }

    g_sVariable.sPower_inst.u16GridFrequency = varGridFrequency;
    NOP();

    return;
}

// INT8U GetPowerMeasurement(void)
//{
//
// }

/*************************************************
Function:void Measure(void)// ????
Description:// ???????????
            ??????????
Author:xdp(??)
Calls:// ???????????
Called By:// ??????????
            main();
Input:// ??????,????????
    // ?????????????
Output:// ?????????
Return:// ????????
Others:// ????
*************************************************/
void Measure(void)
{
    // if((!(pEnergyTimer->TimerCount%(ENERGY_TIMER_LOAD/400)))||(g_VAR_RUN.Measure_Reset_CNT >10)) //???????
    // {
    //  if((Calibrate_CheckSum_Error())||(g_VAR_RUN.Measure_Reset_CNT >10))//????????
    //  {
    //   // ????????
    //   EnergyPutCalibrate();
    //   g_VAR_RUN.Measure_Reset_CNT =0x00;
    //  }
    // }
    // // ?????????????
    EnergyTickIndex++;
    if (EnergyTickIndex > 3)
    {
        EnergyTickIndex = 0x00;
        //三相电压
        SyncPhaseVoltage();
        //三相电流
        SyncPhaseCurrent();
        // //  WDTCLR();//????
        //   //相角
        //   SyncPhaseVoltageAngle();
        //   //电网频率
        SyncGridFrequency();
        //状态-相序
        SyncStateRegister();
    }

    return;
}
