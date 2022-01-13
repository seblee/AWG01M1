/*********************************************************
  Copyright (C), 2014, Sunrise Group. Co., Ltd.
  File name:      	com_proc.c
  Author: gongping	Version: 0.7       Date:  2014-12-05
  Description:    	Main entry, system threads initialization
  Others:         	n.a
  Function List:  	n.a
  Variable List:  	n.a
  Revision History:         
  Date:           Author:          Modification:
	2014-12-05      Alair         file create
*********************************************************/


#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "threads.h" 
#include "global.h"
#include "Drv_PWM.h"
#include "Drv_DIO.h"
#include "DRV_TIME.h"
#include "Drv_Led.h"
#include "i2c_bsp.h" 
//static void hw_drivers_init(void);

osThreadId tid_Core;//控制进程
//osThreadId tid_DI;//DI进程
osThreadId tid_Communiction;//通信进程
osThreadId tid_BackGround;//后台进程

osThreadDef(Core_Proc, osPriorityNormal, 1, 0);
//osThreadDef(DI_Proc, osPriorityNormal, 1, 0);
osThreadDef(Communiction_proc, osPriorityNormal, 1, 0);
osThreadDef(BackGround_proc, osPriorityLow, 1, 0);

/*************************************************
  Function:       // main
  Description:    // Main function, global initialization and user thread creation
  Calls:          // 
										 osThreadCreate()
										 osDelay()  										 
  Called By:      // None
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
int main(void)
{   
    tid_Core = osThreadCreate(osThread(Core_Proc), NULL);//控制任务
    tid_Communiction = osThreadCreate(osThread(Communiction_proc), NULL);//通信任务
	  tid_BackGround = osThreadCreate(osThread(BackGround_proc), NULL);//后台任务
	  osDelay(osWaitForever); 	
	  while(1)
	  {;}			
}

//static void hw_drivers_init(void)
//{
//		Drv_DIO_Init();	//DI初始化	
//		drv_i2c_init();
//    led_init();
//		return;
//}
