#ifndef __SYS_MEMORYMAP_H_
#define __SYS_MEMORYMAP_H_
#include "sys_def.h"

// 存储空间RAM,EEROM,FLASH

#define OFFSET(Struct, Member) ((unsigned char *)&((Struct *)0)->Member)
#define SIZEOF(Struct, Member) (sizeof(((Struct *)0)->Member))

enum eMEMORYTYPE
{
    MEMORY_RAM = 0,
    MEMORY_EEROM,
    MEMORY_FLASH,
    MEMORY_MAX
};

/********** FLASH地址空间定义(以FLASH_ADDR_开头) */
#define FLASH_USER_START                                      \
    ((uint32_t)0x08003C00) /* EEPROM emulation start address: \
//                                                  after 14KByte of used Flash memory */
/************????********************************/
#define FLASH_ADDR_PARAMETER_START FLASH_USER_START   
#define FLASH_ADDR_PARAMETER_END (FLASH_ADDR_PARAMETER_START + sizeof(sParameter))
#define FLASH_ADDR_CALIBRATE_START FLASH_ADDR_PARAMETER_END   
#define FLASH_ADDR_CALIBRATE_END (FLASH_ADDR_CALIBRATE_START + sizeof(Calibrate_ATT))

#endif
