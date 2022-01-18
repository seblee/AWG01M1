#ifndef _MB_CB_H
#define _MB_CB_H

#include "cmsis_os.h"
#include "stdint.h"
#include "macro.h"
#include "mb.h"
#include "mbport.h"
#include "global.h"
#include "Drv_flash.h"
#include "string.h"
#include "SYS_MemoryMap.h"
#include "Lib_Delay.h"
#include "Lib_Memory.h"
#include "Drv_Uart.h"

#define REG_HOLDING_NREGS 120

#define REG_HOLDING_START 0
#define CONFIG_REG_MAP_OFFSET 0
#define STATUS_REG_MAP_OFFSET 110
#define CPAD_REG_HOLDING_WRITE_NREGS (REG_HOLDING_NREGS)

#define MB_SOFTWARE_VER 0x1006  //软件版本
#define MB_HARDWARE_VER 0x1000

#define MB_DEVICE_ADDR 0x01  //默认地址
#define MB_BAUDRATE 19200

// MB通信地址
enum MB_CFG
{
    MB_CFG_Num      = 0,
    MB_CFG_SW       = 1,
    MB_CFG_HW       = 2,
    MB_CFG_WorkMode = 6,
    MB_CFG_CMD      = 7,
    MB_CFG_ADDR     = 8,
    MB_CFG_BAUDRATE = 9,
    MB_CFG_RS       = 10,
    //		MB_CFG_FANSTEP=11,
    //		MB_CFG_FANOUT=12,
    //		MB_GEAR_MIN=13,
    //		MB_GEAR_MAX=14,
    //		MB_TEMPER_MIN=15,
    //		MB_TEMPER_MAX=16,
    //		MB_CFG_TEST=18,
};

#define CMD_MB_SAVE_FLASH 4
#define CMD_MB_SYS_RESET 3
#define CMD_MB_RESET_DEFAULT 2
#define CMD_MB_FACTORY_MODE 1
#define CMD_MB_USER_MODE 0

#define MB_FLASH_WR_FLAG 0x1bdf

// modbus holding regs
typedef struct
{
    uint16_t u8RegStart;  //寄存器起始地址
    uint16_t u16RegBuffer[REG_HOLDING_NREGS];
    uint8_t u8RegProperty[REG_HOLDING_NREGS];  //读写状态
    uint8_t u8RegStatus[REG_HOLDING_NREGS];
    uint16_t u8PgFlag;
} sModbus;

#define PAR_SIZE (sizeof(sModbus))

enum UARTNO
{
    USART0_CH,
    USART1_CH,
    //	USART3_CH,
    USART_NUM
};

enum
{
    BAUD_4800  = 4800,
    BAUD_9600  = 9600,
    BAUD_19200 = 19200,
    BAUD_38400 = 38400,
};

// typedef struct
//{
//		INT16U u16RS;	  		//保留
//		INT16U u16DI_Mask;	//掩码
//		INT16U u16DI_Polarity;	  //极性
//		INT16U u16Rev[2];	  //极性
//		INT16U u16FanStep;	//步长
//		INT16U u16FanOut;	  //输出
//		INT16U u16AO;	//输出
//		INT16U u16DI_Bitmap;		//DI
//		INT16U u16State;		//状态
// }sFan_st;

// typedef struct
//{
//		INT8U AddressRange[2];				//地址范围
//		INT8U DeviceType[4];					//设备类型
//		INT8U SoftwareVersion[2];			//软件版本
//		INT8U HardwareVersion[2];			//硬件版本
//		INT8U SN[8];									//序列号
//		INT8U ManufactureDate[4];			//出厂日期
//		INT8U CommAddress[2];					//通信地址
//		INT8U Bardrate[2];						//波特率
//		INT8U NULLREG[5*2];						//空闲
//		INT8U Status[2];							//状态
//		INT8U ConfigREG[2];							//控制寄存器
// }sParameter;

#define PARA_LEN sizeof(sParameter)

#define PARA_CAL_LEN PARA_LEN + CAL_LEN
#define REG_LEN0 4
#define REG_LEN1 PARA_LEN - REG_LEN0 * 2
#define REG_LEN2 (40033 + 3 - 40030 + 1) * 2

// cpad err code
#define CPAD_ERR_NOERR 0
#define CPAD_ERR_ADDR_OR 1
#define CPAD_ERR_DATA_OR 2
#define CPAD_ERR_PERM_OR 3
#define CPAD_ERR_WR_OR 4
#define CPAD_ERR_CONFLICT_OR 5

#define CPAD_ERR_UNKNOWN 0x1f

extern void SaveSettings(uint8_t u8Type);
extern uint16_t MBGetBaudrate(void);
extern uint8_t MBGetAddrsee(void);

extern INT8U MBRegsiterUpdate(INT8U u8Byte);
extern INT8U MBRegsiterInit(void);
extern void MBResolve(void);
extern void SystemReset(void);

extern ULONG mb_get_baudrate(uint16_t baudrate);
extern uint8_t mb_get_device_addr(void);
extern eMBErrorCode ResetDefaultParameter(void);
#endif
