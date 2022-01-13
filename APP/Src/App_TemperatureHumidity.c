/*********************************************************
  Copyright (C), 2014, Sunrise Group. Co., Ltd.
  File name:      	daq_proc.c
  Author: gongping	Version: 0.7       Date:  2014-12-05
  Description:    	Main entry, system threads initialization
  Others:         	n.a
  Function List:  	water_level_sts_get(void)
                    calc_conductivity(void);
										calc_humcurrent(void);
  Variable List:  	n.a
  Revision History:         
  Date:           Author:          Modification:
	2014-12-05      gongping         file create
*********************************************************/
#include "cmsis_os.h"  
#include "sys_def.h"
#include "adc.h"
#include "Drv_flash.h"
#include "global.h"
#include "Drv_IIC.h"
#include "App_TemperatureHumidity.h"
#include <math.h>
#include <string.h>

/*************************************************
  Function:       // VariableInit_TH
  Description:    // 温湿度传感器变量初始化
  Calls:          // ReadTempRh
								 
  Called By:      // TemperatureHumidity_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void VariableInit_TH(void)
{

		memset(&g_sVariable.sTemperatureHumidity_inst,0x00,sizeof(sTemperatureHumidity));
	
}

/*************************************************
  Function:       // TemperatureHumidity
  Description:    // 温湿度采集函数
  Calls:          // ReadTempRh
								 
  Called By:      // TemperatureHumidity_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // 温度、湿度
  Others:         // None
*************************************************/
INT16U TemperatureHumidity(INT8U TH_CMD)
{
		INT16U TempRh;
		TempRh=ReadTempRh(TH_CMD);
		if(TempRh==0x7FF)
		{
			g_sVariable.sTemperatureHumidity_inst.u16State |=HARDERR;
		}
		else
		{
			g_sVariable.sTemperatureHumidity_inst.u16State &=~HARDERR;			
		}

		return TempRh;	
}
/*************************************************
  Function:       // TemperatureHumidityInit
  Description:    // 温湿度采集初始化
  Calls:          // ReadTempRh
								 
  Called By:      // TemperatureHumidity_Proc
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // 温度、湿度
  Others:         // None
*************************************************/
void TemperatureHumidityInit(void)
{
		TransducerInit();	//温湿度传感器初始化
		VariableInit_TH();	//温湿度传感器变量初始化
}
/*************************************************
  Function:       // TemperatureHumidity_Proc
  Description:    // 温湿度采集任务
  Calls:          // HTU21Init
								 
  Called By:      // main
  Table Accessed: // None
  Table Updated:  // None
  Input:          // None
  Output:         // None
  Return:         // None
  Others:         // None
*************************************************/
void TemperatureHumidity_Proc(void const *argument)
{
		
		TemperatureHumidityInit();	//温湿度传感器初始化
		
		while(1)
		{		
				g_sVariable.sTemperatureHumidity_inst.u16Temperature.Value = TemperatureHumidity(T_SCMD);		//温度
				g_sVariable.sTemperatureHumidity_inst.u16Humidity.Value = TemperatureHumidity(RH_SCMD);		//湿度
				g_sVariable.i16Temp =~g_sVariable.sTemperatureHumidity_inst.u16Temperature.Value;//TEST
				g_sVariable.i16Temp +=1;
				osDelay(100);
		}
}
