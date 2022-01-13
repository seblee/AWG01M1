#ifndef __THREADS_H
#define	__THREADS_H

#define		COM_OSDELAY         200
#define		CORE_OSDELAY				100	
#define		DI_OSDELAY					300
#define		BKG_OSDELAY					500	

#define CORE_PROC_DLY		500u
#define COM_PROC_DLY    10u
//#define BKG_PROC_DLY		500u
#define BKG_PROC_DLY		100u

void Communiction_proc(void const *argument);
void BackGround_proc(void const *argument);
void TemperatureHumidity_Proc(void const *argument);
void Power_Proc(void const *argument);
void Core_Proc(void const *argument);
void DI_Proc(void const *argument);
#endif/*__THREADS_H*/
