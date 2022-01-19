/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //检查相关函数
  Version:         //V1.0
  Function List:   //StringCompare
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/30    1.0     build this moudle
***********************************************************/
#include "macro.h"
#include <stdio.h>
#include <Lib_Check.h>

//===========================================================================================
// Function:static INT8 StringCompare(U8 *newval,U8 *oldval,U8 len)
// Description:????????:????,????
// Input:newval:??1,oldval:??2,len:????
// Output:NULL
// Return:
// 	      1:newval>oldval
//        0:newval=oldval
//   	 -1:newval<oldval
// Others:NULL
//===========================================================================================
INT8 StringCompare(INT8U *newval, INT8U *oldval, INT8U Length)
{
    U8 i;
    for (i = Length - 1; i != 0xff; i--)
    {
        if (newval[i] != oldval[i])
        {
            if (newval[i] > oldval[i])
            {
                return GREATER;  //??
            }
            else
            {
                return LESS;  //??
            }
        }
    }
    return EQUAL;  //????
}
