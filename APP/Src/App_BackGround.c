#include "cmsis_os.h"
#include "sys_def.h"
#include "Drv_Led.h"
#include "global.h"
#include "sys_status.h"
#include "daq.h"
#define osObjectsPublic  // define objects in main module
#include "osObjects.h"   // RTOS object definitions

extern IWDG_HandleTypeDef hiwdg;

static void sys_comp_cooldown(void)
{
    extern local_reg_st l_sys;
    uint16_t i;

    for (i = 0; i < DO_MAX_CNT; i++)
    {
        if (l_sys.comp_timeout[i] > 0)
        {
            l_sys.comp_timeout[i]--;
        }
        else
        {
            l_sys.comp_timeout[i] = 0;
        }
    }
    //出水延时
    if (l_sys.Water.OutWater_Delay)
    {
        l_sys.Water.OutWater_Delay--;
    }
    else
    {
        l_sys.Water.OutWater_Delay = 0;
    }

    //出水结束延时
    if (l_sys.Water.u8CloseDelay)
    {
        l_sys.Water.u8CloseDelay--;
    }
    else
    {
        l_sys.Water.u8CloseDelay = 0;
    }

    if (l_sys.Water.TH_Check_Interval > 0)
    {
        l_sys.Water.TH_Check_Interval--;
    }
    else
    {
        l_sys.Water.TH_Check_Interval = 0;
    }

    return;
}
//运行时间计算
static void time_calc(uint16_t *sec, uint16_t *h)
{
    uint16_t second;
    uint16_t hour;

    second = *sec;
    hour   = *h;
    if ((second & 0x0fff) >= 3600)  // 1小时
    {
        second = 0;
        if (second == 0)
        {
            hour++;
            *h = hour;
        }
        *sec = second;
    }
}
//运行时间
static void run_time_process(void)
{
    extern local_reg_st l_sys;
    //		time_t now;
    uint16_t i;
    static uint16_t u16Sec = 0;

    for (i = 0; i < DO_FILLTER_BPOS; i++)
    {
        if (i >= 16)  //大于16
        {
            if ((g_sVariable.status.u16DO_Bitmap[1] & (0x0001 << (i - 16))) != 0)
            {
                g_sVariable.status.u16Runtime[0][i]++;
                time_calc(&g_sVariable.status.u16Runtime[0][i], &g_sVariable.status.u16Runtime[1][i]);
            }
        }
        else
        {
            if ((g_sVariable.status.u16DO_Bitmap[0] & (0x0001 << i)) != 0)
            {
                g_sVariable.status.u16Runtime[0][i]++;
                time_calc(&g_sVariable.status.u16Runtime[0][i], &g_sVariable.status.u16Runtime[1][i]);
            }
        }
    }
    //过滤网运行时间累计
    if ((g_sVariable.status.u16DO_Bitmap[0] & (0x0001 << DO_FAN_BPOS)) != 0)
    {
        g_sVariable.status.u16Runtime[0][DO_FILLTER_BPOS]++;
        time_calc(&g_sVariable.status.u16Runtime[0][DO_FILLTER_BPOS],
                  &g_sVariable.status.u16Runtime[1][DO_FILLTER_BPOS]);
    }
    //滤芯运行时间累计
    if (!g_sVariable.gPara.Water.u16FILTER_ELEMENT_Type)  //时间计算
    {
        if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, OUTWATER_STS_BPOS) == TRUE))  // Water out
        {
            for (i = DO_FILLTER_ELEMENT_BPOS_0; i <= DO_FILLTER_ELEMENT_MAX; i++)
            {
                g_sVariable.status.u16Runtime[0][i]++;
                time_calc(&g_sVariable.status.u16Runtime[0][i], &g_sVariable.status.u16Runtime[1][i]);
            }
        }
        if (l_sys.Water.OutWater_OK == WATER_OUT)
        {
            l_sys.Water.OutWater_OK = WATER_IDLE;
            //						//总流量
            //						g_sVariable.status.Water.u16Cumulative_Water[2] += g_sVariable.status.u16Last_Water;
            //						Flow_calc(&g_sVariable.status.Water.u16Cumulative_Water[2],
            //&g_sVariable.status.Water.u16Cumulative_Water[0]);
            //						Flow_calc(&g_sVariable.status.Water.u16Cumulative_Water[0],
            //&g_sVariable.status.Water.u16Cumulative_Water[1]);
        }
    }
    else  //流量计算
    {
    }

    u16Sec++;
    // rt_kprintf("adc_value=%d\n", u16Sec);
    if ((u16Sec % FIXED_SAVETIME) == 0)  //每15分钟保存一次
    {
        u16Sec = 0;
        I2C_EE_BufWrite((uint8_t *)&g_sVariable.status.u16Runtime, STS_REG_EE1_ADDR,
                        sizeof(g_sVariable.status.u16Runtime));  // when, fan is working update eeprom every minite
    }
    return;
}

//清零运行时间
uint8_t reset_runtime(uint16_t param)
{
    uint8_t i, req = 0;

    if (param == 0xff)
    {
        for (i = 0; i < DO_MAX_CNT; i++)
        {
            g_sVariable.status.u16Runtime[0][i] = 0;
            g_sVariable.status.u16Runtime[1][i] = 0;
        }
        req = 1;
    }
    else
    {
        if ((param > 0) && (param <= DO_MAX_CNT))
        {
            g_sVariable.status.u16Runtime[0][param - 1] = 0;
            g_sVariable.status.u16Runtime[1][param - 1] = 0;
            req                                         = 1;
        }
    }
    if (req == 1)
    {
        I2C_EE_BufWrite((uint8_t *)&g_sVariable.status.u16Runtime, STS_REG_EE1_ADDR,
                        sizeof(g_sVariable.status.u16Runtime));
    }

    return (req);
}

/*********************************************************
  * @name   BackGround_proc
    * @brief  system background threads, only toggle led in this program.
    * @calls  led_open()
            led_close()
                        osDelay()
  * @called main()
  * @param  None
  * @retval None
*********************************************************/
void BackGround_proc(void)
{
    static uint8_t num[5] = {0};

    osDelay(BKG_OSDELAY);  //上电延时
//    led_init();
//    AM_Init();  // TH
//    drv_adc_dma_init();
//    TDS_Usart_Init();  // TDS
//    IWDG_Configuration();
    while (1)
    {
        LEDCtrlStatus();  // LED运行状态
        if (++num[0] >= 11)
        {
            num[0] = 0;
            sys_running_mode_update();
            daq_gvar_update();
        }
        if (++num[1] >= 13)
        {
            num[1] = 0;
            AM_Sensor_update();  //温湿度更新
        }
        //			if(++num[2] >= 19)
        //			{
        //				num[2]=0;
        //				TDS_Send();
        //			}
        //			if(++num[3] >= 5)
        //			{
        //				num[3]=0;
        //				Comm_Service();
        //			}
        if (++num[4] >= 10)
        {
            num[4] = 0;
            sys_comp_cooldown();
            run_time_process();
        }
        /* 从新导入IWDG计数器 */
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(BKG_PROC_DLY);
    }
}
