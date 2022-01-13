#include "systick.h"
#include "cmsis_os.h"  

//static __IO uint32_t TimingDelay;

//void Delay(__IO uint32_t nTime)//us
//{ 
////  TimingDelay = nTime;

////  while(TimingDelay != 0);
////		osDelay(nTime);
//}

///**
//  * @brief  Decrements the TimingDelay variable.
//  * @param  None
//  * @retval None
//  */
//void TimingDelay_Decrement(void)
//{
//  if (TimingDelay != 0x00)
//  { 
//    TimingDelay--;
//  }
//}

//void Systick_Init(void)
//{
////  if (SysTick_Config(SystemCoreClock / 1000))//1∫¡√Î
//  if (SysTick_Config(SystemCoreClock ))//1us
//  { 
//    /* Capture error */ 
//    while (1);
//  }
//}
