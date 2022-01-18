#include "Lib_Delay.h"
#include "macro.h"

void Delay(U16 DelayValue)  // us
{
    DelayValue *= 4;
    while (DelayValue--)
        ;
    return;
}

void Delay_us(unsigned long u32us)
{
    unsigned char i;

    while (u32us--)
    {
        for (i = 0; i < 2; i++)
            ;
    }
}
