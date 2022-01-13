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
////�û������Լ�����Ҫ����
//#define STM32_FLASH_SIZE 	16 	 		//��ѡSTM32��FLASH������С(��λΪK)
//#define STM32_FLASH_WREN 	1              	//ʹ��FLASHд��(0��������;1��ʹ��)
////////////////////////////////////////////////////////////////////////////////////////////////////////

////FLASH��ʼ��ַ
//#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ
////FLASH������ֵ
//#define FLASH_KEY1               0X45670123
//#define FLASH_KEY2               0XCDEF89AB
// extern void STMFLASH_Unlock(void);					  //FLASH����
// extern void STMFLASH_Lock(void);					  //FLASH����
// extern u8 STMFLASH_GetStatus(void);				  //���״̬
// extern u8 STMFLASH_WaitDone(u16 time);				  //�ȴ���������
// extern u8 STMFLASH_ErasePage(u32 paddr);			  //����ҳ
// extern u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat);//д�����
// extern u16 STMFLASH_ReadHalfWord(u32 faddr);		  //��������
// extern void STMFLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	//ָ����ַ��ʼд��ָ�����ȵ�����
// extern u32 STMFLASH_ReadLenByte(u32 ReadAddr,u16 Len);						//ָ����ַ��ʼ��ȡָ����������

extern INT8U Flash_Write(u32 WriteAddr, const u16 *pBuffer, u16 NumToWrite);  //��ָ����ַ��ʼд��ָ�����ȵ�����
extern void Flash_Read(u32 ReadAddr, u8 *pBuffer, u16 NumToRead);  //��ָ����ַ��ʼ����ָ�����ȵ�����

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
