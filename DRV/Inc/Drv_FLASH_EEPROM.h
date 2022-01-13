/**
 ******************************************************************************
 * @file    EEPROM_Emulation/inc/eeprom.h
 * @author  MCD Application Team
 * @version V3.1.0
 * @date    07/27/2009
 * @brief   This file contains all the functions prototypes for the EEPROM
 *          emulation firmware library.
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRV_FLASH_EEPROM_H
#define __DRV_FLASH_EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"
#include "macro.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////
////用户根据自己的需要设置
//#define STM32_FLASH_SIZE 	16 	 		//所选STM32的FLASH容量大小(单位为K)
//#define STM32_FLASH_WREN 	1              	//使能FLASH写入(0，不是能;1，使能)
////////////////////////////////////////////////////////////////////////////////////////////////////////

////FLASH起始地址
//#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址
////FLASH解锁键值
//#define FLASH_KEY1               0X45670123
//#define FLASH_KEY2               0XCDEF89AB
// extern void STMFLASH_Unlock(void);					  //FLASH解锁
// extern void STMFLASH_Lock(void);					  //FLASH上锁
// extern u8 STMFLASH_GetStatus(void);				  //获得状态
// extern u8 STMFLASH_WaitDone(u16 time);				  //等待操作结束
// extern u8 STMFLASH_ErasePage(u32 paddr);			  //擦除页
// extern u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);//写入半字
// extern u16 STMFLASH_ReadHalfWord(u32 faddr);		  //读出半字
// extern void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//指定地址开始写入指定长度的数据
// extern u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//指定地址开始读取指定长度数据

extern INT8U Flash_Write(u32 WriteAddr, const u16 *pBuffer, u16 NumToWrite);  //从指定地址开始写入指定长度的数据
extern void Flash_Read(u32 ReadAddr, u8 *pBuffer, u16 NumToRead);  //从指定地址开始读出指定长度的数据

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
