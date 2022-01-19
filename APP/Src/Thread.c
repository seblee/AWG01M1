
#define osObjectsPublic  // define objects in main module
#include "osObjects.h"   // RTOS object definitions

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/

osThreadId tid_Core;          //控制进程
osThreadId tid_Communiction;  //通信进程

osThreadDef(Core_Proc, osPriorityNormal, 1, 0);
osThreadDef(Communiction_proc, osPriorityNormal, 1, 0);

int Init_Thread(void)
{
    tid_Core = osThreadCreate(osThread(Core_Proc), NULL);
    if (!tid_Core)
        return (-1);
    tid_Communiction = osThreadCreate(osThread(Communiction_proc), NULL);
    if (!tid_Communiction)
        return (-1);

    return (0);
}
