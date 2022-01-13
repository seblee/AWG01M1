#ifndef _DAQ_H
#define _DAQ_H

#include "sys_conf.h"

#define MAX_PRESS_MEAVAL	3400			//ѹ��������������ֵ(34.00bar) ����100��
#define MAX_PRESS_STEP		5			//����ѹ������ 0.5bar


enum
{
	TARGET_MODE_RETURN=0,	//�ط�
	TARGET_MODE_SUPPLY,		//�ͷ�
	TARGET_MODE_REMOTE,		//Զ��
};
enum
{
		AVERAGE_TEMP_MODE =0 ,//ƽ���¶�
		MAX_TEMP_MODE,				//����¶�
};

extern void temp_avg_calc(uint8_t index);
extern void temp_avg_process(void);
extern void press_avg_calc(uint8_t index);
extern void press_avg_process(void);
extern void vapor_temp_avg_calc(uint16_t refrig,uint8_t index);		//ѹ��ת�¶�
extern void vapor_temp_avg_process(void);
extern void super_heat_calc(uint16_t refrig,uint8_t index);   //���ȶ�
extern void super_heat_avg_process(void);
extern void ai_sts_update(void);
extern void daq_gvar_update(void);
#endif
