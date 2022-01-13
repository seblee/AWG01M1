#ifndef _DAQ_H
#define _DAQ_H

#include "sys_conf.h"

#define MAX_PRESS_MEAVAL	3400			//压力传感器最大测量值(34.00bar) 扩大100倍
#define MAX_PRESS_STEP		5			//饱和压力步数 0.5bar


enum
{
	TARGET_MODE_RETURN=0,	//回风
	TARGET_MODE_SUPPLY,		//送风
	TARGET_MODE_REMOTE,		//远程
};
enum
{
		AVERAGE_TEMP_MODE =0 ,//平均温度
		MAX_TEMP_MODE,				//最大温度
};

extern void temp_avg_calc(uint8_t index);
extern void temp_avg_process(void);
extern void press_avg_calc(uint8_t index);
extern void press_avg_process(void);
extern void vapor_temp_avg_calc(uint16_t refrig,uint8_t index);		//压力转温度
extern void vapor_temp_avg_process(void);
extern void super_heat_calc(uint16_t refrig,uint8_t index);   //过热度
extern void super_heat_avg_process(void);
extern void ai_sts_update(void);
extern void daq_gvar_update(void);
#endif
