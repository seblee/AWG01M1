/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: portserial.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

#include "port.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "Drv_Uart.h"
#include "fifo.h"
#include "global.h"

/* ----------------------- static functions ---------------------------------*/

/* ----------------------- Start implementation -----------------------------*/
// void
// vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
//{
//     /* If xRXEnable enable serial receive interrupts. If xTxENable enable
//      * transmitter empty interrupts.
//      */
// }

void vMBPortSerialEnable(BOOL xRxEnable, BOOL xTxEnable)
{
    if (xRxEnable)
    {
        /* 485通信时，等待串口移位寄存器中的数据发送完成后，再去使能485的接收、失能485的发送*/
        while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))
        {
        }
        RE485;
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    }
    else
    {
        TE485;
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_RXNE);
    }
    if (xTxEnable)
    {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    }
}

BOOL xMBPortSerialInit(UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity)
{
    uart1_init(ulBaudRate);
    return TRUE;
}

BOOL xMBPortSerialPutByte(CHAR ucByte)
{
    if (HAL_UART_Transmit(&huart1, (uint8_t*)&ucByte, 1, 1) != HAL_OK)
    {
        return FALSE;
    }
    return TRUE;
}

BOOL xMBPortSerialGetByte(CHAR* pucByte)
{
    *pucByte = uart1RXBuff;
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call
 * xMBPortSerialPutByte( ) to send the character.
 */
void prvvUARTTxReadyISR(void)
{
    pxMBFrameCBTransmitterEmpty();
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
void prvvUARTRxISR(void)
{
    pxMBFrameCBByteReceived();
}
