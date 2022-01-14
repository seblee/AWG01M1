
#include "Lib_Memory.h"


// ==================================================================
// Function name: 	ArraySwap
// Description:		?????????????????	
// Parameter:			pData = ??????;nLen = ????,????100??
// Return:           	NULL
// Other:            	NULL
// ==================================================================
void MemoryReverse(U8 *pData,U8 nLen)
{
    unsigned char j;
    unsigned char i;
    unsigned char nSwap;
    
    if (nLen == 0 || nLen == 1)
    {
        return;
    }
    
    i = 0;
    j = nLen - 1;
    while (i < j)
    {
        nSwap = *(pData + i);
        *(pData + i) = *(pData + j);
        *(pData + j) = nSwap;
        i++;
        j--;
    }
    return;
}

void MemoryCopy(U8 *pDis,const U8 *pSource,U8 Length)
{
	if(!Length)
	{
		return;
	}
	while(Length--)
	{
		*(pDis++) = *(pSource++);
	}
	return;
}
void MemoryReverseCopy(U8 *pDis,const U8 *pSource,U8 Length)
{
	if(!Length)
	{
		return;
	}
	pSource += (Length-1);
	while(Length--)
	{
		*(pDis++) = *(pSource--);
	}
	return;
}
//??????0
BOOL Judge_ArrayZero(U8 *pData,U8 nLen)
{
	U8 i;
	for(i=0;i<nLen;i++)
	{
		if(pData[i]!=0x00)
		{
			return FALSE;
		}
	}
	return TRUE;
}

U8 BCDToHEX(U8 nValue)
{
	U8 nOut;
	nOut = (((nValue >> 4) * 10) + (nValue & 0x0F));
	return nOut;
}
U8 HEXToBCD(U8 nValue)
{
	U8 nOut;
	
	nOut = (((nValue / 10) << 4) + (nValue % 10));
	return nOut;
}
U16  BCDToU16(U8 *pBCD)
{
    U16 n;
	
    n = BCDToHEX(pBCD[1]);
    n*= 100;
    n += BCDToHEX(pBCD[0]);
    return n;
}

U32  BCDToULong(U8 *pBCD)
{
    U32 n;
    n = BCDToHEX(pBCD[3]);
    n*= 100;
    n += BCDToHEX(pBCD[2]);
    n*= 100;
    n += BCDToHEX(pBCD[1]);
    n*= 100;
    n += BCDToHEX(pBCD[0]);
    return n;
}
#define TEN_MILLIONS 10000000
#define ONE_MILLION 1000000
#define HUNDRED_THOUSANDS 100000
#define TEN_THOUSANDS 10000
#define ONE_THOUSAND 1000
#define ONE_HUNDRED 100
void ULongToBCD(U32 lHEX, unsigned char *pbBCD)
{   
    pbBCD[0] = 0;
    pbBCD[1] = 0;
    pbBCD[2] = 0;
    pbBCD[3] = 0;
    while (lHEX > (TEN_MILLIONS-1))
    {
        pbBCD[3] += 0x10;
        lHEX -= TEN_MILLIONS;
    }
    while (lHEX >(ONE_MILLION - 1))
    {
        pbBCD[3] += 0x01;
        lHEX -= ONE_MILLION;
    }
    while (lHEX > (HUNDRED_THOUSANDS - 1))
    {
        pbBCD[2] += 0x10;
        lHEX -= HUNDRED_THOUSANDS;
    }
    while (lHEX > (TEN_THOUSANDS - 1))
    {
        pbBCD[2] += 0x01;
        lHEX -= TEN_THOUSANDS;
    }
    while (lHEX > (ONE_THOUSAND - 1))
    {
        pbBCD[1] += 0x10;
        lHEX -= ONE_THOUSAND;
    }
    while (lHEX > (ONE_HUNDRED - 1))
    {
        pbBCD[1] += 0x01;
        lHEX -= ONE_HUNDRED;
    }
    pbBCD[0] = HEXToBCD((unsigned char)lHEX);
}

void U16ToBCD(U16 lHEX, unsigned char *pbBCD)
{   
    pbBCD[0] = 0;
    pbBCD[1] = 0;
    while (lHEX > (ONE_THOUSAND - 1))
    {
        pbBCD[1] += 0x10;
        lHEX -= ONE_THOUSAND;
    }
    while (lHEX > (ONE_HUNDRED - 1))
    {
        pbBCD[1] += 0x01;
        lHEX -= ONE_HUNDRED;
    }
    pbBCD[0] = HEXToBCD((unsigned char)lHEX);
}


