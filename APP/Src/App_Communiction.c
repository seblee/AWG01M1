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
#include "threads.h"
#include "stm32g0xx_hal.h"
#include "Drv_Uart.h"
#include "Drv_flash.h"
#include "fifo.h"
#include "sys_def.h"
#include "global.h"
#include "string.h"
#include "Lib_Memory.h"
#include "DRV_FLASH_EEPROM.h"
#include "SYS_MemoryMap.h"
#include "App_Communiction.h"
#include "mb.h"
#include "i2c_bsp.h"

typedef struct
{
    uint16_t baudrate;
    uint16_t com_addr;
} communication_change_st;

communication_change_st com_change_inst;

static void change_surv_baudrate(void)
{
    uint8_t u8MB = 0;

    if ((com_change_inst.baudrate != g_sVariable.gPara.Bardrate) ||
        (g_sVariable.gPara.CommAddress != com_change_inst.com_addr))
    {
        u8MB = 1;
    }
    //ͨѶ�쳣
    g_sVariable.status.Com_error++;
    if (g_sVariable.status.Com_error >= COMERR_3S)
    {
        u8MB                         = 1;
        g_sVariable.status.Com_error = 0;
    }
    //���³�ʼ������
    if (u8MB)
    {
        com_change_inst.baudrate = g_sVariable.gPara.Bardrate;
        com_change_inst.com_addr = g_sVariable.gPara.CommAddress;
        // eMBDisable();
        eMBInit(MB_RTU, mb_get_device_addr(), 0, mb_get_baudrate(com_change_inst.baudrate), MB_PAR_NONE);
        eMBEnable();
    }
    return;
}

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
    static uint8_t num[2] = {0};
    osDelay(COM_OSDELAY);  //�ϵ���ʱ
#ifdef E2PROM
    drv_i2c_init();
#else

#endif
    MBRegsiterInit();
    eMBInit(MB_RTU, MBGetAddrsee(), USART0_CH, MBGetBaudrate(), MB_PAR_NONE);
    eMBEnable();

    while (1)
    {
        eMBPoll();  // MB״̬����
#ifdef E2PROM
        change_surv_baudrate();
#else
        MBResolve();          // MB����
        MBRegsiterUpdate(1);  // MB���ݸ���
#endif
        if (++num[0] >= 50)
        {
            num[0] = 0;
            //�ص�������дEEPROM
            CallBack_map_write();
            // TDS�������ݴ���
            Comm_Service();
        }
        if (++num[1] >= 100)
        {
            num[1] = 0;
            TDS_Send();
        }

        osDelay(COM_PROC_DLY);
    }
}
