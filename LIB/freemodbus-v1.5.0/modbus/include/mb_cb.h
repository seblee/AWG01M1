#ifndef _MB_CB_H
#define _MB_CB_H

#include "stdint.h"

#define REG_HOLDING_START 1
#define REG_HOLDING_NUM 35

#define MB_SOFTWARE_VER					0x04
#define MB_SOFTWARE_SUBVER 			0x03
#define MB_HARDWARE_VER					0x02
#define MB_HARDWARE_SUBVER 			0x01
#define MB_DEVICE_TYPE					"S01P"
#define MB_SERIAL_NO						"1234578"
#define MB_MAN_DATE							"20140510"
#define MB_DEVICE_ADDR					1
#define MB_MAN_DATE							"20140510"
#define MB_BAUDRATE							4800

#define CMD_MB_SAVE_FLASH				4
#define CMD_MB_SYS_RESET				3
#define CMD_MB_RESET_DEFAULT		2
#define CMD_MB_FACTORY_MODE			1
#define CMD_MB_USER_MODE				0

#define MB_FLASH_WR_FLAG				0x1bdf

//modbus holding regs
typedef struct
{
		uint8_t u8Address;	
		uint16_t u16Baudrate;		
		uint16_t u8RegStart;	//¼Ä´æÆ÷ÆðÊ¼µØÖ·
		uint16_t u16RegBuffer[REG_HOLDING_NUM];
		uint8_t u8RegProperty[REG_HOLDING_NUM];	
	  uint8_t u8RegStatus[REG_HOLDING_NUM];			
		uint8_t u8UpdateFlag;
}sModbus;

void mb_reg_update(void);
void mb_reg_init(void);
void mb_cmd_resolve(void);
void save_current_settings(void);
uint8_t mb_get_baudrate(void);
uint16_t mb_get_device_addr(void);

#endif

