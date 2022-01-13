/*********************************************************
  Copyright (C), 2014, Sunrise Group. Co., Ltd.
  File name:      	daq_proc.c
  Author: gongping	Version: 0.7       Date:  2014-12-05
  Description:    	Main entry, system threads initialization
  Others:         	n.a
  Function List:  	
	
  Variable List:  	n.a
  Revision History:         
  Date:           Author:          Modification:
	2014-12-05      gongping         file create
*********************************************************/
#include "cmsis_os.h"  
#include "Drv_flash.h"
#include "global.h"
#include "Drv_IIC.h"
#include "App_Power.h"
#include "Hal_Measure.h"
#include <math.h>
#include <Lib_Delay.h>
#include <string.h>

/*************************************************
  Function:       // VariableInit_Power
  Description:    // ��Դ�����ر�����ʼ��
  Calls:          // ReadTempRh
								 
  Called By:      // TemperatureHumidity_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void VariableInit_Power(void)
{

		memset(&g_sVariable.sPower_inst,0x00,sizeof(sPower));
	
}

/*************************************************
  Function:       // MeasureInit
  Description:    // ��Դ������ʼ��
  Calls:          // VariableInit_Power
								 
  Called By:      // Power_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void MeasureInit(void)
{
		Measure_Init();	//��Դ����ʼ��
		VariableInit_Power();	//��Դ�����ر�����ʼ��
}
/*************************************************
  Function:       // Power_Proc
  Description:    // ��Դ��������
  Calls:          // MeasureInit
								 
  Called By:      // main
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void Power_Proc(void const *argument)
{		
		osDelay(600);//�ϵ���ʱ
		MeasureInit();	//��Դ������ʼ��	
		while(1)
		{		
				Measure();//��Դ����
				osDelay(100);
		}
}


