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
#include "global.h"

//获取校正开关标志
BOOL GetCalibrateFlag(void)
{
    INT8U_Check Bytes;
    //	//获取校正标志
    ////	if(Read(DEV_EEPROM,(U8*)(OFFSET(Calibrate_ATT,Calibrate_Flag)+EEP_ADDR_CALIBRATE_START),2,&Bytes.cValue))
    //		memcpy(&Bytes.cValue,&g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0],2);
    //		if(g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0] ==
    //~g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[1])
    //		{
    ////				if(g_sVariable.sMeasureCalibrate_inst.Calibrate_Flag[0] == 0xA5)
    ////				{
    //////					return TRUE;
    ////				}
    //		}
    //		else
    //		{
    //		//
    //if(Read(DEV_EEPROM,(U8*)(OFFSET(Calibrate_ATT,Calibrate_Flag)+EEP_ADDR_CALIBRATE_BACKUP_START),2,&Bytes.cValue))
    //				STMFLASH_Read(FLASH_ADDR_CALIBRATE_START,(u8*)&Bytes.cValue,2);		//读较表标志
    //				{
    //						if(Bytes.cValue == ~Bytes.CheckValue)
    //						{
    //								if(Bytes.cValue == 0xA5)
    //								{
    //										return TRUE;
    //								}
    //						}
    //				}
    //		}
    return FALSE;
}
