#ifndef __MACRO_H
#define __MACRO_H

#include "stdint.h"

/* 定义系统所使用的基本数据类型 */
//// 无符号char,1字节,(+0)-(+255)
// typedef unsigned char U8;
//// 有符号char,1字节,(-127)-(+127)
// typedef signed char S8;
//// 无符号short int,2字节,(+0)-(+65535)
// typedef unsigned short int U16;
//// 有符号short int,2字节,(-32767)-(+32767)
// typedef signed short int S16;
//// 无符号long,4字节,(+0)-(+(2^32-1))
// typedef unsigned long U32;
//// 有符号long,4字节,(-(2^31-1))-(+(2^31-1))
// typedef signed long S32;

// typedef unsigned char           INT8U;
// typedef unsigned short          INT16U;
// typedef unsigned long           INT32U;

// typedef signed char             INT8S;
// typedef signed short            INT16S;
// typedef signed long             INT32S;

typedef unsigned char INT8U;
typedef unsigned short int INT16U;
typedef unsigned int INT32U;

typedef signed char INT8;
typedef signed short int INT16;
typedef signed int INT32;

#define U8 INT8U
#define U16 INT16U
#define U32 INT32U

#define u8 INT8U
#define u16 INT16U
#define u32 INT32U

//(TRUE/FALSE)
typedef unsigned char BOOL;
#define TRUE 1
#define FALSE 0

// 16位联合体
typedef union
{
    INT16U Value;
    INT8U acBytes[2];
} INT16U_UNION;
// 32位联合体
typedef union
{
    INT32U lValue;
    INT8U acBytes[4];
} INT32U_UNION;

//带反码校验字节
typedef struct
{
    INT8U cValue;
    INT8U CheckValue;
} INT8U_Check;

#endif /*__MACRO_H*/
