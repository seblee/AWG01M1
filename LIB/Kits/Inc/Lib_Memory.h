#ifndef _LIB_MEMORY_H_
#define _LIB_MEMORY_H_
#include "macro.h"
extern void MemoryReverse(U8 *pData, U8 nLen);
// ??????
extern void MemoryCopy(U8 *pDis, const U8 *pSource, U8 Length);
// ??????
extern void MemoryReverseCopy(U8 *pDis, const U8 *pSource, U8 Length);

extern void ReverseCopy(unsigned char *pDis, unsigned char *pSource, unsigned char Length);
extern BOOL Judge_ArrayZero(U8 *pData, U8 nLen);
extern U32 BCDToULong(U8 *pBCD);
extern U16 BCDToU16(U8 *pBCD);
extern void ULongToBCD(U32 lHEX, unsigned char *pbBCD);

#endif
