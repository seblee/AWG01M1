#include "flash.h"
#include "sys_def.h"

/*********************************************************
  * @name   flash_write
	* @brief  write data messages to designated flash address
	* @calls  device drivers          
  * @called water_quality_gain_calib()
	          hum_current_offset_calib()
  * @param  addr  : flash physical address
	          p_data:data buffer to be write to flash
						count : write counts 	
  * @retval The return value can be:
						@arg 1: flash write success
            @arg 0: flash write fail
*********************************************************/
int16_t flash_write(uint32_t addr, const uint16_t* p_data, uint16_t count)
{
	uint16_t i;
  /* Unlock the Flash to enable the flash control register access *************/ 
  FLASH_Unlock();
  /* Clear pending flags (if any) */  
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 
	
	if(FLASH_ErasePage(FLASH_USER_START_ADDR)!=FLASH_COMPLETE)
	{
		return -1;
	}
	else
	{
		for(i=0;i<count;i++)
		{
			if(FLASH_ProgramHalfWord(addr+2*i+FLASH_USER_START_ADDR,*(p_data+i))!=FLASH_COMPLETE)
				return 0;
		}
	}
	return 1;	
}
