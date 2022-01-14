///************************************************************
//  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
//  FileName: Drv_IIC.c
//  Author:        Version :          Date:
//  Description:     //Э�������غ��� 
//  Version:         //V1.0
//  Function List:   //
//    1. -------
//  History:         //
//      <author>  <time>   <version >   <desc>
//      xdp       14/12/15    1.0     build this moudle  
//***********************************************************/

//#include "cmsis_os.h"  
//#include "stm32f0xx.h"
//#include "uart.h"
//#include "flash.h"
//#include "fifo.h"
//#include "sys_def.h"
//#include "global.h"
//#include "string.h"
//#include "Lib_Memory.h"
//#include "Lib_AnalyseProtocol.h"
//#include "DRV_FLASH_EEPROM.h"
//#include "SYS_MemoryMap.h"
//#include "Lib_Memory.h"

////static INT8U gl_Address;			//uart transmitt data buffer
////INT8U      g_ComBuff[USART_NUM][PROTOCOL_FRAME_LEN+4];  //
////PROTOCOL_STATUS     g_ComStat[USART_NUM];                 //????????
////INT8U      g_ComGap[USART_NUM];                  //??????????
////INT8U      g_ComAddr[COM_PortMax];                 //???????
////INT8U      g_ComLen[COM_PortMax];                  //????????
////INT8U      g_ComBaud[COM_PortMax];                  //?????
////INT8U      g_Buffer[PARA_CAL_LEN];  //
////// ????????????
////ProtocolLayer ProtocolStatckLayer[USART_NUM];
////ProtocolLayer* Protocol[USART_NUM];

//static INT8U Device_Type[4] = {"SP"};//�豸����
////static INT8U Software_Version[12] = {"SPAC01A1-V10"};//���������
////static INT8U Hardware_Version[12] = {"SPAC01A1-A00"};//Ӳ���汾��


///* CRC���ֽڱ�*/   
//const INT8U  auchCRCHi[] = {    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,    
//		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,    
//		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,    
//		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,    
//		0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,    
//		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,    
//		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,    
//		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,    
//		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,    
//		0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,    
//		0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,    
//		0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,    
//		0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,    
//		0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,    
//		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40    
//} ;    
///* CRC���ֽڱ�*/  
//const INT8U  auchCRCLo[] = {    
//		0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,    
//		0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,    
//		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,    
//		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,    
//		0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,    
//		0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,    
//		0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,    
//		0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,    
//		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,    
//		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,    
//		0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,    
//		0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,    
//		0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,    
//		0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,    
//		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,    
//		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,    
//		0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,    
//		0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,    
//		0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,    
//		0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,    
//		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,    
//		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,    
//		0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,    
//		0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,    
//		0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,    
//		0x43, 0x83, 0x41, 0x81, 0x80, 0x40    
//} ;   
//   
////CRC16У��   
//INT16U CRC16(INT8U *puchMsg, INT16U usDataLen)
//{    
//    INT8U uchCRCHi = 0xFF ; /*���ֽ�CRC��ʼ��*/      
//    INT8U uchCRCLo = 0xFF ; /*���ֽ�CRC��ʼ��*/    
//    INT32U uIndex ;  /* CRCѭ���е�����*/    
//	
//    while (usDataLen--) /*������Ϣ������*/
//    {    
//        uIndex = uchCRCHi ^ *puchMsg++; /*����CRC */     
//        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];    
//        uchCRCLo = auchCRCLo[uIndex] ;    
//    }    
//    return (uchCRCHi << 8 | uchCRCLo) ;    
//} 

////CRC16У��   
//void Cal_16CRC(INT8U *pSrc, INT8U u16Length,INT8U *pDest)
//{     
//		INT16U CheckCRC;//CRCֵ
//	
//		CheckCRC = CRC16(pSrc,u16Length);
//		CheckCRC &= 0xFFFF;
//		pDest[1]=CheckCRC&0xFF;
//		pDest[0]=(CheckCRC>>8)&0xFF;	
//} 

////��ʼ��Э���
//void Register_Protocol_Solt(INT8U ID,ProtocolLayer** ppProtocolSolt)
//{
//		(*ppProtocolSolt) = NULL;

//		if(ppProtocolSolt == NULL)
//			return;

//		if(ID > USART_NUM-1)
//			return;
//		(*ppProtocolSolt) = &ProtocolStatckLayer[ID];
//		(*ppProtocolSolt)->StatckStatus = PROTOCOL_STACK_ADDRESS;
//		(*ppProtocolSolt)->StatckType = 0;
//		(*ppProtocolSolt)->DataCount = 0;
////		(*ppProtocolSolt)->SendProtocol = NULL;
////		(*ppProtocolSolt)->ResetProtocol = NULL;
////		(*ppProtocolSolt)->pfnHandlerSuccess = NULL;
//		(*ppProtocolSolt)->CommAddress = NULL;	
//		(*ppProtocolSolt)->ProtocolHandleStatus = 0;	
//		return;
//}
//////ͨ�ſڳ�ʼ��
////void Comm_Init(INT8U Port)
////{

////		uart1_init();								//uart initialization	
////		CommAddress_GPIO_Init();
////	
////		g_ComStat[Port] = INIT_Apply;
////		g_ComGap[Port]=0;

////		/* ???????? */
////		Register_Protocol_Solt(Port,&Protocol[Port]);
////		Protocol[Port]->pFrame = (ProtocolFrame*)g_ComBuff[Port];
////}


////INT8U ProtocolPrepare(void)
////{
////	
////		memset(g_Buffer,0x00,PARA_CAL_LEN);
////		g_Buffer[1] = PROTOCOL_REG_NUM-2;//�Ĵ�������	
////		//�Ӵ洢���ж���;Device_Type
////		memcpy(&g_Buffer[2*1],Device_Type,4);
////		g_Buffer[2*3] =0x00;				//����汾
////		g_Buffer[2*3+1] =0x01;
//////		STMFLASH_Read(FLASH_ADDR_PARAMETER_START+REG_LEN0,(u8*)&g_Buffer[REG_LEN0],REG_LEN1);		//������								
////		memcpy(&g_Buffer[20*2],g_sVariable.sPower_inst.u16Voltage,4*2);			//����Դ��������					
//////		STMFLASH_Read(FLASH_ADDR_CALIBRATE_START,(u8*)&g_Buffer[30*2],REG_LEN2);	//����ԴУ������	
////	
////	  return TRUE;
////}

///*************************************************
//  Function:       // Handle_ReadRegisters
//  Description:    // �������Ĵ�������
//  Calls:          // None
//								 
//  Called By:      // AnalyseProtocol
//  Table Accessed: // None
//  Table Updated:  // None
//  Input:          // ProtocolFrame����Ҫ����������
//  Output:         // ׼����������
//  Return:         // None
//  Others:         // None
//*************************************************/
//static void Handle_ReadRegisters(ProtocolLayer* ProtocolFrame)
//{
//		sProtocolDIInfo sDIInfo;
//		INT16U_UNION	uAddress;
//	
//    if((ProtocolFrame->StatckType)&PROTOCOL_COMMON_ADDRESS)//ͨ�õ�ֱַ�ӷ��ص�ַ����
//    {
//				ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_ILLEGAL_FUNCTION_ERR;
//        return;
//    } 
//		
//		//��������
//		MemoryReverseCopy(sDIInfo.uAddress.acBytes,ProtocolFrame->pFrame->pFrameData,2);  //���ƼĴ�����ַ
//		MemoryReverseCopy(sDIInfo.uData.acBytes,ProtocolFrame->pFrame->pFrameData+2,2);  //�������ݳ���
//		uAddress.Value = BCDToU16((U8*)&sDIInfo.uAddress.acBytes);

//		switch (uAddress.Value) //�Ĵ�����ַ4000X-40001
//		{
//				case 0x0000://��ַ����
//				case 0x0001://�豸����
//				case 0x0003://����汾
//				case 0x0004://Ӳ���汾
//				case 0x0005://���к�
//				case 0x0009://��������
//				case 0x0011://
//				case 0x0012://
//				case 0x0013://
//					
//				case 0x0019://
//				case 0x0020://
//				case 0x0021://
//				case 0x0022://

//				case 0x0029://
//				case 0x0030://
//				case 0x0031://
//				case 0x0032://			
//						if((sDIInfo.uAddress.Value+sDIInfo.uData.Value)>PROTOCOL_REG_NUM)
//						{
//								ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_ILLEGAL_DATA_ERR;		//�Ƿ�����
//								return;						
//						}
//						else
//						{
//								ProtocolPrepare();//�������׼��			
//								ProtocolFrame->OpsType =OPS_READ;//������
//								ProtocolFrame->REGAddress =sDIInfo.uAddress.Value;//�Ĵ�����ַ
//								ProtocolFrame->Length = 2*sDIInfo.uData.Value;		//���ݳ���
//						}
//						break;
//				default://���������޷�ʶ��
//						ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_ILLEGAL_ADDRSEE_ERR;		
//						return;
//		}

//		ProtocolFrame->ProtocolHandleStatus = PROTOCOL_HANDLE_SUCCEED;//����ɹ�
//		return;
//}


///*************************************************
//  Function:       // Handle_SetSingleRegister
//  Description:    // �������õ����Ĵ�������
//  Calls:          // None
//								 
//  Called By:      // AnalyseProtocol
//  Table Accessed: // None
//  Table Updated:  // None
//  Input:          // ProtocolFrame����Ҫ����������
//  Output:         // ׼����������
//  Return:         // None
//  Others:         // None
//*************************************************/
//static void Handle_SetSingleRegister(ProtocolLayer* ProtocolFrame)
//{

//		sProtocolDIInfo sDIInfo;
//		INT16U_UNION	uAddress;
//		INT32U u32Addr;
//		INT8U u8Register;
//		INT8U u8Buffer[2];
//	
//    if((ProtocolFrame->StatckType)&PROTOCOL_COMMON_ADDRESS)//ͨ�õ�ֱַ�ӷ��ص�ַ����
//    {
//				ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_ILLEGAL_FUNCTION_ERR;
//        return;
//    } 
//		
//		//��������
//		MemoryReverseCopy(sDIInfo.uAddress.acBytes,ProtocolFrame->pFrame->pFrameData,2);  //���ƼĴ�����ַ
//		MemoryReverseCopy(sDIInfo.uData.acBytes,ProtocolFrame->pFrame->pFrameData+2,2);  //��������
//		uAddress.Value = BCDToU16((U8*)&sDIInfo.uAddress.acBytes);

//		switch (uAddress.Value) //�Ĵ�����ַ4000X-40001
//		{
////				case 0x0000://��ַ����
////				case 0x0001://�豸����
////				case 0x0003://����汾
//				case 0x0004://Ӳ���汾
//				case 0x0011://
//				case 0x0012://
//				case 0x0013://
//						
//						u32Addr = FLASH_ADDR_PARAMETER_START+sDIInfo.uAddress.Value*2;
//						MemoryReverseCopy(u8Buffer,sDIInfo.uData.acBytes,2);  //��������					
//						STMFLASH_Write(u32Addr,(u16*)&u8Buffer,1);		//д����	
//					
//						ProtocolFrame->OpsType =OPS_WRITE;//д����					
//						ProtocolFrame->Length = 4;				//���ݳ���		
//						break;								
//				case 0x0029://У���־
//						if(sDIInfo.uData.Value !=0x5A)
//						{
//								return;
//						}
//						else
//						{
//								CalibrateClean();//���У������
//						}
//				case 0x0030://
//						u8Register = PHASE_A_GAIN;
//				case 0x0031://
//						u8Register = PHASE_A_GAIN+1;
//				case 0x0032://	
//						u8Register = PHASE_A_GAIN+2;
//						if(CalibrateWrite(u8Register,sDIInfo.uData.Value))//У��
//						{
//								return;
//						}
////						u32Addr = FLASH_ADDR_PARAMETER_START+sDIInfo.uAddress.Value*2;
////						STMFLASH_Write(u32Addr,(u16*)&sDIInfo.uData.acBytes,1);		//д����					
//						ProtocolFrame->OpsType =OPS_WRITE;//д����					
//						ProtocolFrame->Length = 4;				//���ݳ���		
//						break;					
////				case 0x0005://���к�				
////				case 0x0009://��������
//					
//				default://���������޷�ʶ��
//						ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_ILLEGAL_ADDRSEE_ERR;		
//						return;
//		}

//		ProtocolFrame->ProtocolHandleStatus = PROTOCOL_HANDLE_SUCCEED;//����ɹ�
//		return;
//	
//	
//}

///*************************************************
//  Function:       // FramePacket
//  Description:    // �����������
//  Calls:          // None
//								 
//  Called By:      // 
//  Table Accessed: // None
//  Table Updated:  // None
//  Input:          // ProtocolFrame����Ҫ���ص�����
//  Output:         // �������ݵ�����
//  Return:         // None
//  Others:         // None
//*************************************************/
//static BOOL FramePacket(ProtocolLayer* ProtocolFrame)
//{
//		INT16U_UNION  uCheckCRC;
//		INT8U  u8Addr;
//		INT8U  u8Length;
//	
//		if(ProtocolFrame->ProtocolHandleStatus & PROTOCOL_HANDLE_SUCCEED)
//		{	
//				if(ProtocolFrame->OpsType ==OPS_READ)//������
//				{
//						memcpy(&g_ComBuff[1][0],&ProtocolFrame->pFrame->Address,2);	
//						memcpy(&g_ComBuff[1][2],&ProtocolFrame->Length,1);	
//						u8Addr = 2 * ProtocolFrame->REGAddress;
//						memcpy(&g_ComBuff[1][3],&g_Buffer[u8Addr],ProtocolFrame->Length);	
//						u8Length = ProtocolFrame->Length +0x03;
//				}
//				else//д����
//				{
//						u8Length = ProtocolFrame->Length +0x02;					
//				}
//				
//		}
//		else
//		{
//				if(ProtocolFrame->ProtocolHandleStatus & PROTOCOL_ILLEGAL_ADDRSEE_ERR)
//				{
//						//��ַ����
//						ProtocolFrame->pFrame->pFrameData[0] = PROTOCOL_ILLEGAL_ADDRSEE_ERR;
//				}
//				else
//				{
//						//���ܴ���
//						ProtocolFrame->pFrame->pFrameData[0] = PROTOCOL_ILLEGAL_FUNCTION_ERR;
//				}
//				//׼�����ش�������
//				ProtocolFrame->pFrame->FrameCMD |= PROTOCOL_CMD_RESPONSE_ERR;
//				u8Length = 0x03;		
//				memcpy(&g_ComBuff[1][0],&ProtocolFrame->pFrame->Address,u8Length);			
//		}

//		uCheckCRC.Value = CRC16(&g_ComBuff[1][0],u8Length);//����CRCУ��
//		MemoryReverseCopy(&g_ComBuff[1][u8Length],&uCheckCRC.acBytes[0],2);	

//		uart1_dma_send(&g_ComBuff[1][0],(u8Length+2));//send whole frame;

//		return TRUE;
//}



///*************************************************
//  Function:       // AnalyseProtocol
//  Description:    // Э�����
//  Calls:          // None
//								 
//  Called By:      // 
//  Table Accessed: // None
//  Table Updated:  // None
//  Input:          // ProtocolFrame�����յ�����
//  Output:         // 
//  Return:         // None
//  Others:         // None
//*************************************************/
//INT8U AnalyseProtocol(ProtocolLayer* ProtocolFrame)
//{
//		INT8U u8CMD;
//		//�����޷�ʶ��
//		ProtocolFrame->ProtocolHandleStatus = PROTOCOL_HANDLE_UNDEFINE;

//		u8CMD = ProtocolFrame->pFrame->FrameCMD;
//	
//		//��������
//		switch(u8CMD)
//		{
//				case ReadRegisters:							//03,���Ĵ���״̬			                      
//					Handle_ReadRegisters(ProtocolFrame);	
//					break;
//				case SetSingleRegisters:				//06�����õ����Ĵ���		                      
//					Handle_SetSingleRegister(ProtocolFrame);	
//					break;
//				default :
//					// �Ƿ�����
//					ProtocolFrame->ProtocolHandleStatus |= PROTOCOL_ILLEGAL_FUNCTION_ERR;
//					break;
//		}   
//		// �����������
//		if(FramePacket(ProtocolFrame))
//		{
//			return TRUE;
//		}
//	    
//		return FALSE;
//			
//}

//INT16U u16Length;
////�Ӷ�����֡���
//INT8 FrameDetectSlave(INT8U Port)
//{	
//		INT8U RXDByte = 0x00;
//		INT16U CheckCRC = 0x00;	
//		INT16U TempCRC = 0x00;	
//		INT8	i8Ret=0x00;
////		INT8U	u8Length=0x00;
////		INT8U timeout = 0x00;
//	
//		u16Length =0x00;
//		Protocol[Port]->CommID=Port;
//		gl_Address = 0x01;	
//#ifdef  	SA
//		gl_Address = GetCommAddress();		
//#endif	
////		gl_Address = GetCommAddress();	
////		Protocol[Port]->CommAddress = gl_Address;	
////		Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;	
//		while(is_fifo8_empty(&fifo_rx_buf) != 1)			//while input fifo is not empty, pop data out continuously
//		{
//				u16Length = get_fifo8_length(&fifo_rx_buf);
//				fifo8_pop(&fifo_rx_buf,(uint8_t*)&RXDByte);						//pop out fifo data			
//				switch(Protocol[Port]->StatckStatus)
//				{	
//						case PROTOCOL_STACK_ADDRESS:							
//								g_ComStat[Port] = RECV_Going;                       //?"????"??
//								Protocol[Port]->StatckType = PROTOCOL_PRIVATE_ADDRES | PROTOCOL_COMMON_ADDRESS;				
//								//ͨ�õ�ַ00
//								if((0x00 == RXDByte)||(gl_Address == RXDByte))
//								{	
//										//ͨ�õ�ַ00
//										if(0x00 == RXDByte)
//										{
//												Protocol[Port]->StatckType &= (~PROTOCOL_PRIVATE_ADDRES);
//										}								
//										//��ַ���
//										else if (gl_Address == RXDByte)
//										{		
//												Protocol[Port]->StatckType &= (~PROTOCOL_COMMON_ADDRESS);													
//										}										
//										g_ComBuff[Port][0] = RXDByte;
//		//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;
//										Protocol[Port]->DataCount = 1;								
//										Protocol[Port]->StatckStatus =PROTOCOL_STACK_CMD;	
//								}								
//								else//��ַ����ȷ
//								{
//										
//										Protocol[Port]->DataCount = 0;	
//										Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;	
//										g_ComStat[Port] = INIT_Apply;                    //?"????"??									
//								}		
//								i8Ret=0;	
//								break;
//						case PROTOCOL_STACK_CMD:		
//								//����������CMD
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;
//								Protocol[Port]->StatckStatus ++;
//								i8Ret=0;	
//								break;	
//						case PROTOCOL_STACK_DATA0:			
//								//����������
//								Protocol[Port]->StatckStatus ++;
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
//								i8Ret=0;					
//								break;			
//						case PROTOCOL_STACK_DATA1:			
//								//����������
//								Protocol[Port]->StatckStatus ++;
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
//								i8Ret=0;					
//								break;	
//						case PROTOCOL_STACK_DATA2:			
//								//����������
//								Protocol[Port]->StatckStatus ++;
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
//								i8Ret=0;					
//								break;	
//						case PROTOCOL_STACK_DATA3:			
//								//����������
//								Protocol[Port]->StatckStatus ++;
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
//								i8Ret=0;					
//								break;			
//						case PROTOCOL_STACK_CRC0:			
//								//����������
//								Protocol[Port]->StatckStatus ++;
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
//								i8Ret=0;					
//								break;							
//						case PROTOCOL_STACK_CRC1:			
//								//����CRCУ��λ
//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;					
//														
//								TempCRC = g_ComBuff[Port][Protocol_Length-2]	<< 8;		
////										TempCRC <<= 8;
//								TempCRC |= g_ComBuff[Port][Protocol_Length-1];											
//								CheckCRC = CRC16(&g_ComBuff[Port][0],Protocol_Length-2);
//								if( CheckCRC == TempCRC)
//								{		
//									//У��������ȷ
//										Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;
//										g_ComStat[Port] = RECV_Over;                    //?"????"??
//										i8Ret = 0x5A;	
//								}
//								else
//								{
//										Protocol[Port]->DataCount = 0;		
//										Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;
//										g_ComStat[Port] = INIT_Apply;               //?"???"??								
//										i8Ret = -1;												
//								}

//								break;	
//						default:
//								Protocol[Port]->DataCount = 0;		
//								Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;
//								g_ComStat[Port] = INIT_Apply;               //?"???"??			
//								i8Ret = 0;							
//					}	
//					if(i8Ret != 0)																//if ret != 0, means frame resolved, either success or failed,function returns.
//							return i8Ret;				
//			}
//			fifo8_reset(&fifo_rx_buf);										//reset fifo after data popped out, prevent unexpected fifo hazard.
//			return i8Ret;		

//}

//////�Ӷ�����֡���
////INT8 FrameDetectSlave(INT8U Port)
////{	
////		INT8U RXDByte = 0x00;
////		INT16U CheckCRC = 0x00;	
////		INT16U TempCRC = 0x00;	
////		INT8	i8Ret=0x00;
//////		INT8U	u8Length=0x00;
//////		INT8U timeout = 0x00;
////	
////		Protocol[Port]->CommID=Port;
////		gl_Address = GetCommAddress();	
//////		Protocol[Port]->CommAddress = gl_Address;	
//////		Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;	
////		while(is_fifo8_empty(&fifo_rx_buf) != 1)			//while input fifo is not empty, pop data out continuously
////		{
////			
////				fifo8_pop(&fifo_rx_buf,(uint8_t*)&RXDByte);						//pop out fifo data			
////				switch(Protocol[Port]->StatckStatus)
////				{	
////						case PROTOCOL_STACK_ADDRESS:							
////								g_ComStat[Port] = RECV_Going;                       //?"????"??
////								Protocol[Port]->StatckType = PROTOCOL_PRIVATE_ADDRES | PROTOCOL_COMMON_ADDRESS;				
////								//ͨ�õ�ַ00
////								if((0x00 == RXDByte)||(gl_Address == RXDByte))
////								{	
////										//ͨ�õ�ַ00
////										if(0x00 == RXDByte)
////										{
////												Protocol[Port]->StatckType &= (~PROTOCOL_PRIVATE_ADDRES);
////										}								
////										//��ַ���
////										else if (gl_Address == RXDByte)
////										{		
////												Protocol[Port]->StatckType &= (~PROTOCOL_COMMON_ADDRESS);													
////										}										
////										g_ComBuff[Port][0] = RXDByte;
////		//								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;
////										Protocol[Port]->DataCount = 1;								
////										Protocol[Port]->StatckStatus =PROTOCOL_STACK_CMD;	
////								}								
////								else//��ַ����ȷ
////								{
////										
////										Protocol[Port]->DataCount = 0;	
////										Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;	
////										g_ComStat[Port] = INIT_Apply;                    //?"????"??									
////								}		
////								i8Ret=0;	
////								break;
////						case PROTOCOL_STACK_CMD:		
////								//����������CMD
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;
////								Protocol[Port]->StatckStatus ++;
////								i8Ret=0;	
////								break;	
////						case PROTOCOL_STACK_DATA0:			
////								//����������
////								Protocol[Port]->StatckStatus ++;
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
////								i8Ret=0;					
////								break;			
////						case PROTOCOL_STACK_DATA1:			
////								//����������
////								Protocol[Port]->StatckStatus ++;
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
////								i8Ret=0;					
////								break;	
////						case PROTOCOL_STACK_DATA2:			
////								//����������
////								Protocol[Port]->StatckStatus ++;
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
////								i8Ret=0;					
////								break;	
////						case PROTOCOL_STACK_DATA3:			
////								//����������
////								Protocol[Port]->StatckStatus ++;
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
////								i8Ret=0;					
////								break;			
////						case PROTOCOL_STACK_CRC0:			
////								//����������
////								Protocol[Port]->StatckStatus ++;
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;	
////								i8Ret=0;					
////								break;							
////						case PROTOCOL_STACK_CRC1:			
////								//����CRCУ��λ
////								g_ComBuff[Port][(Protocol[Port]->DataCount)++] = RXDByte;					
////														
////								TempCRC = g_ComBuff[Port][Protocol_Length-2]	<< 8;		
//////										TempCRC <<= 8;
////								TempCRC |= g_ComBuff[Port][Protocol_Length-1];											
////								CheckCRC = CRC16(&g_ComBuff[Port][0],Protocol_Length-2);
////								if( CheckCRC == TempCRC)
////								{		
////									//У��������ȷ
////										Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;
////										g_ComStat[Port] = RECV_Over;                    //?"????"??
////										i8Ret = 0x5A;	
////								}
////								else
////								{
////										Protocol[Port]->DataCount = 0;		
////										Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;
////										g_ComStat[Port] = INIT_Apply;               //?"???"??								
////										i8Ret = -1;												
////								}

////								break;	
////						default:
////								Protocol[Port]->DataCount = 0;		
////								Protocol[Port]->StatckStatus = PROTOCOL_STACK_ADDRESS;
////								g_ComStat[Port] = INIT_Apply;               //?"???"??			
////								i8Ret = 0;							
////					}	
////					if(i8Ret != 0)																//if ret != 0, means frame resolved, either success or failed,function returns.
////							return i8Ret;				
////			}
////			fifo8_reset(&fifo_rx_buf);										//reset fifo after data popped out, prevent unexpected fifo hazard.
////			return i8Ret;		

////}
