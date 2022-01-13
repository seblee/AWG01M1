/*********************************************************
  Copyright (C), 2021, ALW. Co., Ltd.
  File name:      	App_DI.c
  Author: Alair	    Version: 0.7       Date:  2021-2-15
  Description:    	
  Others:         	n.a
  Function List:  	
	
  Variable List:  	n.a
  Revision History:         
  Date:           Author:          Modification:
	2014-12-05      Alair         file create
*********************************************************/
#include "cmsis_os.h"  
#include "Drv_flash.h"
#include "threads.h" 
#include "global.h"
#include <math.h>
#include <Lib_Delay.h>
#include <string.h>
#include "App_DI.h"
#include "Drv_DIO.h"
#include "DRV_TIME.h"

/*************************************************
  Function:       // DI_Proc
  Description:    // DI任务
  Calls:          // 
								 
  Called By:      // main
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void DI_Proc(void const *argument)
{		
		static uint16_t u16Num = 0;
//		osDelay(DI_OSDELAY);//上电延时
		Drv_DIO_Init();	//DI初始化	
		DIO_reg_init();
		TimersInit_14(3000,ENABLE);//定时3ms
		while(1)
		{		
//				Sync_Di_timeout();//DI采集
				if(++u16Num >= DI_BUF_DEPTH)
				{
					u16Num = 0;
					DI_reg_update();
					DI_sts_update();
				}
				osDelay(SAMPLE_INTERVAL);
		}
}
