#ifndef __LIB_ANALYSEPROTOCOL_H
#define	__LIB_ANALYSEPROTOCOL_H

#include "macro.h"

	
	#define PROTOCOL_PRIVATE_ADDRES		(0x01)	// ????
	#define PROTOCOL_COMMON_ADDRESS		(0x02)	// ????(?AA)
	
	//???????----------------------------------------------------------------
	typedef enum  
	{
	    RECV_Wait,          //????
	    RECV_Going,         //????
	    RECV_Over,          //????
	    SEND_Wait,          //????
	    SEND_Going,         //????
	    SEND_Over,          //????
	    INIT_Apply          //???
	}PROTOCOL_STATUS;
	// ??DL/T 645-2007 ??????
	enum PROTOCOL_STATCK_STATUS
	{
		PROTOCOL_STACK_ADDRESS=0,
		PROTOCOL_STACK_CMD,
		PROTOCOL_STACK_DATA0,
		PROTOCOL_STACK_DATA1,
		PROTOCOL_STACK_DATA2,
		PROTOCOL_STACK_DATA3,
		PROTOCOL_STACK_CRC0,
		PROTOCOL_STACK_CRC1,
	};
//	enum PROTOCOL_STATCK_STATUS
//	{
//		PROTOCOL_STACK_IDLE=0,		
//		PROTOCOL_STACK_ADDRESS,
//		PROTOCOL_STACK_CMD,
//		PROTOCOL_STACK_DATA,
//		PROTOCOL_STACK_CRC,
//	};
	
	#define PROTOCOL_HANDLE_SUCCEED			(0x8000)	//处理成功
	#define PROTOCOL_HANDLE_UNDEFINE		(0x4000)	//不能失识别
	
	#define PROTOCOL_CMD_RESPONSE_ERR	(0x80)		//返回错误
	//Modbus异常码
	#define PROTOCOL_ILLEGAL_FUNCTION_ERR	(0x01)		//非法功能
	#define PROTOCOL_ILLEGAL_ADDRSEE_ERR	(0x02)		//非法地址
	#define PROTOCOL_ILLEGAL_DATA_ERR			(0x03)		//非法数据
	#define PROTOCOL_EQUIPMENT_ERR				(0x04)		//从站设备故障
	#define PROTOCOL_CONFIRM_ERR					(0x05)		//确认
	#define PROTOCOL_BUSY_ERR							(0x06)		//从属设备忙
	#define PROTOCOL_MEMORY_PARITY_ERR		(0x08)		//存储奇偶性差错
	#define PROTOCOL_UNAVAILABLE_GATEWAY_ERR		(0x0A)		//不可用网关路径
	#define PROTOCOL_RESPONSE_FAILURE_ERR				(0x0B)		//网关目标设备响应失败
	

	#define ReadRegisters					(0x03)	// ??????
	#define SetSingleRegisters		(0x06)	// ??????
	//
	typedef struct
	{
		INT8U Address;
		INT8U FrameCMD;
		INT8U pFrameData[1];
	}ProtocolFrame;

		//数据操作类型(读、写)
	enum OPS_TYPE
	{
		OPS_READ =0,
		OPS_WRITE ,
	};
	
	// ????????
	typedef INT8U (*ProtocolSend) (INT8U,INT8U,INT8U*);
	// ????????
	typedef void (*ProtocolReset) (INT8U);
	// ????????
	typedef void (*ProtocolHandler)(void);
	// ??DL/T 645-2007 ?????????
	typedef struct
	{
		INT8U CommID;		  // ???
		INT8U CommAddress;   //通信地址
		INT8U Baudrate;      // ????????(DL/T 645-2007 ???????,?????????2400bps)
		INT8U StatckStatus;  // ???????????(??->????->)
		INT8U StatckType;    // ????(????[bit0],?AA???AA????[bit1],?99?????[bit2],[bit3...bit6] ???,???0,?????[bit7])
		INT8U DataCount;     // ?????????()
		INT16U ProtocolHandleStatus;		// ??????(?????????,?????????????)
//		ProtocolSend SendProtocol;		// ?????????
//		ProtocolReset ResetProtocol;	// ??????????
//		ProtocolHandler pfnHandlerSuccess;	// ???????????????
		ProtocolFrame* pFrame;			// ?????????(??????)
	  INT8U REGAddress;		//寄存器地址
	  INT8U Length;		// ??????????
	  INT8U OpsType;		// ??????????
	}ProtocolLayer;

	// CheckDI ?????
	typedef struct
	{
		INT16U_UNION uAddress;		// ??
		INT16U_UNION uData;		// ??DI?
//		INT8U Type;			// ????(?/?,??,??)
//		INT8U OpsType;			// ??????(???)
		INT8U MomeryType;		// ??????(EEROM,FRAM,FALSH,VARRAM),??0x80??????
//		ProtocolHandler pfnWriteSuccessed;	// ???????????
//		INT8U Length;			// ????
	}sProtocolDIInfo;

//enum UARTNO
//{
//	USART1_CH,
//	USART2_CH,
////	USART3_CH,
//	USART_NUM
//};
//#define PROTOCOL_REG_NUM  		35	//寄存器个数
//#define PROTOCOL_FRAME_LEN  	PROTOCOL_REG_NUM*2+5	
//#define Protocol_Length  			0x08	

//	typedef struct
//	{
//			INT8U AddressRange[2];				//地址范围
//			INT8U DeviceType[4];					//设备类型
//			INT8U SoftwareVersion[2];			//软件版本
//			INT8U HardwareVersion[2];			//硬件版本
//			INT8U SN[8];									//序列号
//			INT8U ManufactureDate[4];			//出厂日期
//			INT8U CommAddress[2];					//通信地址
//			INT8U BardrateType[2];						//波特率
//	}sParameter;
//#define PARA_LEN  						sizeof(sParameter)	

//#define PARA_CAL_LEN  				PARA_LEN+sizeof(sMeasureCalibrate)	
//#define REG_LEN0  						2*4	
//#define REG_LEN1  						sizeof(sParameter)-REG_LEN0
//#define REG_LEN2  						(40033+3-40030+1)*2

//extern ProtocolLayer ProtocolStatckLayer[USART_NUM];
//extern ProtocolLayer* Protocol[USART_NUM];
	
	
extern INT8 FrameDetectSlave(INT8U Port);
extern INT8U AnalyseProtocol(ProtocolLayer* ProtocolFrame);
extern void Comm_Init(INT8U Port);	
extern INT16U CRC16(INT8U *puchMsg, INT16U usDataLen);
extern void Cal_16CRC(INT8U *pSrc, INT8U u16Length,INT8U *pDest);
	



#endif

