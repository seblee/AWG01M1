/*********************************************************
  Copyright (C), 2014, Sunrise Group. Co., Ltd.
  File name:      	com_proc.c
  Author: gongping	Version: 0.7       Date:  2014-12-05
  Description:    	Communication handling thread,
                                        dealing with RS48 bus activities
                                        and command resolving
  Others:         	n.a
  Function List:  	frame_detect(void);
                      cmd_resolve(void);
                                        framing(uint8_t cmd_type, const uint8_t *tx_buf, uint8_t length);
  Variable List:  	rx_framebuf[32]
                      tx_framebuf[32]
                                        tx_buf[16]
  Revision History:
  Date:           Author:          Modification:
    2014-12-05      gongping         file create
*********************************************************/

#include "cmsis_os.h"
#include "stm32g0xx_hal.h"
#include "uart.h"
#include "flash.h"
#include "fifo.h"
#include "sys_def.h"
#include "global_var.h"
#include "string.h"
#include "Lib_Memory.h"
#include "Lib_AnalyseProtocol.h"

// extern volatile fifo8_cb_td fifo_rx_buf;

// static uint8_t rx_framebuf[32];	//uart recieve frame buffer
// static uint8_t tx_framebuf[32];	//uart transmitt frame buffer
// static uint8_t tx_buf[16];			//uart transmitt data buffer

/*********************************************************
  * @name   Communiction_proc
    * @brief  communication thread, deal with network protocal and command resolving.
    * @calls  uart1_init()
            frame_detect()
            cmd_resolve()
                        framing()
                        osDelay()
  * @called main()
  * @param  *argument : versatile pointer, not used
  * @retval None
*********************************************************/
void Communiction_proc(void const *argument)
{
    INT8 i8Ret;

    i8Ret = 0;

    //	uart1_init();								//uart initialization
    fifo8_init(&fifo_rx_buf, 1, 64);  // fifo initialization

    Comm_Init(COM_PortUart0);

    while (1)
    {
        //		i16Ret = FrameDetect();				//check if there is any 485 bus activity which might addressing this
        //device, if any resovle
        i8Ret = FrameDetectSlave(COM_PortUart0);
        if (i8Ret > 0)  // pass intergrity check, go into command resolve;
        {
            //			i8Ret = cmd_resolve();
            i8Ret = AnalyseProtocol(&ProtocolStatckLayer[COM_PortUart0]);
        }
        else if (i8Ret == -1)  // if checksum fail, nak
        {
            //			i8Ret = framing(CMD_NAK,NULL,0);
            //			cs_fail_cnt ++;
        }
        else
        {
            ;
        }
        osDelay(10);
    }
}
