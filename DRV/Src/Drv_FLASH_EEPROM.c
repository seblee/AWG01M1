/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //STM32 FLASHģ��EEPROM��������
  Version:         //V1.0
  Function List:   //IIC_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/26    1.0     build this moudle
***********************************************************/

#include "DRV_FLASH_EEPROM.h"
#include "stm32g0xx_hal.h"
#include "stm32f0xx_misc.h"
#include "macro.h"
#include <stdio.h>
#include "Lib_Delay.h"
#include "stm32f0xx_flash.h"
#include "sys_def.h"
#include "Sys_MemoryMap.h"
#include "global.h"

typedef __IO uint16_t vu16;

//��ȡָ����ַ�İ���(16λ����)
// faddr:����ַ
//����ֵ:��Ӧ����.
INT16U STMFLASH_ReadHalfWord(INT32U u32Addr)
{
    return *(vu16 *)u32Addr;
}
//��ָ����ַ��ʼ����ָ�����ȵ�����
// ReadAddr:��ʼ��ַ
// pBuffer:����ָ��
// NumToWrite:����(16λ)��
void Flash_Read(INT32U u32Addr, INT8U *pBuffer, INT16U u16Length)
{
    INT16U i;
    //		for(i=0;i<u16Length;i++)
    //		{
    //				pBuffer[i]=STMFLASH_ReadHalfWord(u32Addr);//��ȡ2���ֽ�.
    //				u32Addr+=2;//ƫ��2���ֽ�.
    //		}
    while (i < u16Length)
    {
        *(pBuffer + i) = *(__IO uint8_t *)u32Addr++;
        i++;
    }
}

INT8U Flash_Write(INT32U u32Addr, const INT16U *pBuffer, INT16U u16Length)
{
    INT16U i;

    /* Unlock the Flash to enable the flash control register access *************/
    FLASH_Unlock();
    /* Clear pending flags (if any) */
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    if (FLASH_ErasePage(FLASH_USER_START) != FLASH_COMPLETE)  //�û��洢��
    {
        return FALSE;
    }
    else
    {
        for (i = 0; i < u16Length; i++)
        {
            if (FLASH_ProgramHalfWord(u32Addr + 2 * i, *(pBuffer + i)) != FLASH_COMPLETE)  //����
            {
                return FALSE;
            }
            else
            {
            }
        }
    }
    FLASH_Lock();  //����
    return TRUE;
}

////����STM32��FLASH
// void STMFLASH_Unlock(void)
//{
//   FLASH->KEYR=FLASH_KEY1;//д���������.
//   FLASH->KEYR=FLASH_KEY2;
// }
////flash����
// void STMFLASH_Lock(void)
//{
//   FLASH->CR|=1<<7;//����
// }
////�õ�FLASH״̬
// u8 STMFLASH_GetStatus(void)
//{
//	u32 res;
//	res=FLASH->SR;
//	if(res&(1<<0))return 1;		    //æ
//	else if(res&(1<<2))return 2;	//��̴���
//	else if(res&(1<<4))return 3;	//д��������
//	return 0;						//�������
// }
////�ȴ��������
////time:Ҫ��ʱ�ĳ���
////����ֵ:״̬.
// u8 STMFLASH_WaitDone(u16 time)
//{
//	u8 res;
//	do
//	{
//		res=STMFLASH_GetStatus();
//		if(res!=1)break;//��æ,����ȴ���,ֱ���˳�.
////		delay_us(1);
//		Delay(1);
//		time--;
//	 }while(time);
//	 if(time==0)res=0xff;//TIMEOUT
//	 return res;
//}
////����ҳ
////paddr:ҳ��ַ
////����ֵ:ִ�����
// u8 STMFLASH_ErasePage(u32 paddr)
//{
//	u8 res=0;
//	res=STMFLASH_WaitDone(0X5FFF);//�ȴ��ϴβ�������,>20ms
//	if(res==0)
//	{
//		FLASH->CR|=1<<1;//ҳ����
//		FLASH->AR=paddr;//����ҳ��ַ
//		FLASH->CR|=1<<6;//��ʼ����
//		res=STMFLASH_WaitDone(0X5FFF);//�ȴ���������,>20ms
//		if(res!=1)//��æ
//		{
//			FLASH->CR&=~(1<<1);//���ҳ������־.
//		}
//	}
//	return res;
// }
////��FLASHָ����ַд�����
////faddr:ָ����ַ(�˵�ַ����Ϊ2�ı���!!)
////dat:Ҫд�������
////����ֵ:д������
// u8 STMFLASH_WriteHalfWord(u32 faddr, u16 dat)
//{
//	u8 res;
//	res=STMFLASH_WaitDone(0XFF);
//	if(res==0)//OK
//	{
//		FLASH->CR|=1<<0;//���ʹ��
//		*(vu16*)faddr=dat;//д������
//		res=STMFLASH_WaitDone(0XFF);//�ȴ��������
//		if(res!=1)//�����ɹ�
//		{
//			FLASH->CR&=~(1<<0);//���PGλ.
//		}
//	}
//	return res;
// }
////��ȡָ����ַ�İ���(16λ����)
////faddr:����ַ
////����ֵ:��Ӧ����.
// u16 STMFLASH_ReadHalfWord(u32 faddr)
//{
//	return *(vu16*)faddr;
// }
//#if STM32_FLASH_WREN	//���ʹ����д
////������д��
////WriteAddr:��ʼ��ַ
////pBuffer:����ָ��
////NumToWrite:����(16λ)��
// void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
//{
//	u16 i;
//	for(i=0;i<NumToWrite;i++)
//	{
//			FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
//	    WriteAddr+=2;//��ַ����2.
//	}
// }
////��ָ����ַ��ʼд��ָ�����ȵ�����
////WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
////pBuffer:����ָ��
////NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
//#if STM32_FLASH_SIZE<256
//#define STM_SECTOR_SIZE 1024 //�ֽ�
//#else
//#define STM_SECTOR_SIZE	2048
//#endif
// u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�

// INT8U STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
//{
//		u32 secpos;	   //������ַ
//		u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
//		u16 secremain; //������ʣ���ַ(16λ�ּ���)
//		u16 i;
//		u32 offaddr;   //ȥ��0X08000000��ĵ�ַ

//	//	u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
//		u16 STMFLASH_BUF[CAL_LEN];//�����2K�ֽ�
//
//		if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
//		{
//				return FALSE;//�Ƿ���ַ
//		}
//		offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
//		secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
//		secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
//		secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С
//		if(NumToWrite<=secremain)
//		{
//				secremain=NumToWrite;//�����ڸ�������Χ
//		}
//		FLASH_Unlock();						//����
//		while(1)
//		{
//	//		Flash_Read	(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������
//			Flash_Read	(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,CAL_LEN);//��������У�����ݵ�����
//			for(i=0;i<secremain;i++)//У������
//			{
//				if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����
//			}
//			if(i<secremain)//��Ҫ����
//			{
//					/* Clear pending flags (if any) */
//					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
//					if(FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE)!=FLASH_COMPLETE)//�����������
//					{
//							return FALSE;
//					}
//					else
//					{
//							for(i=0;i<secremain;i++)//����
//							{
//									STMFLASH_BUF[i+secoff]=pBuffer[i];
//							}
////
///STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������
//							STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,CAL_LEN);//д����������

//					}
//			}
//			else
//			{
//					STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
//			}
//			if(NumToWrite==secremain)
//			{
//					break;//д�������
//			}
//			else//д��δ����
//			{
//					secpos++;				//������ַ��1
//					secoff=0;				//ƫ��λ��Ϊ0
//					pBuffer+=secremain;  	//ָ��ƫ��
//					WriteAddr+=secremain;	//д��ַƫ��
//					NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
//					if(NumToWrite>(STM_SECTOR_SIZE/2))
//					{
//							secremain=STM_SECTOR_SIZE/2;//��һ����������д����
//					}
//					else
//					{
//							secremain=NumToWrite;//��һ����������д����
//					}
//			}
//		};
//		FLASH_Lock();//����
//		return TRUE;
//}
//#endif
