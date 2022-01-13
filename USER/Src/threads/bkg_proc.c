#include "cmsis_os.h"  
#include "sys_def.h"
#include "led.h"

/*********************************************************
  * @name   bkg_proc
	* @brief  system background threads, only toggle led in this program.
	* @calls  led_open()
            led_close()
						osDelay()
  * @called main()
  * @param  None
  * @retval None
*********************************************************/
void bkg_proc(void const *argument)
{
    led_init();
    while(1)
    {
      led_open();
      osDelay(500);
      led_close();
      osDelay(500);
    }
}

