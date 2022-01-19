/*********************************************************
  Copyright (C), 2021, ALW. Co., Ltd.
  File name:      	App_Core.c
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
#include "global.h"
#include <math.h>
#include <Lib_Delay.h>
#include <string.h>
#include "App_Core.h"
#include "Drv_DIO.h"
#include "DRV_TIME.h"
#include "i2c_bsp.h"
#include "daq.h"
#define osObjectsPublic  // define objects in main module
#include "osObjects.h"   // RTOS object definitions
/*************************************************
  Function:       // Core_Proc
  Description:    // 控制任务
  Calls:          //

  Called By:      // main
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void Core_Proc(void const *argument)
{
    static uint16_t u16Num[2] = {0};

    osDelay(CORE_OSDELAY);  //上电延时
    Drv_DIO_Init();         // DI初始化
    DIO_reg_init();
    TimersInit_14(99, ENABLE);  //定时10ms
    alarm_acl_init();
    printf("Core_Proc\r\n");
    while (1)
    {
        if (++u16Num[0] >= 2)  // 1s
        {
            u16Num[0] = 0;
            //需求执行
            req_execution();
            //输出
            oc_update();
        }
        if (++u16Num[1] >= 3)  // 1.5s
        {
            u16Num[1] = 0;
            alarm_acl_exe();
        }
        DI_reg_update();
        DI_sts_update();
        osDelay(CORE_PROC_DLY);
    }
}
