#include "cmsis_os.h"  // ARM::CMSIS:RTOS:Keil RTX
#include "threads.h"
#include "global.h"
#include "calc.h"
#include "sys_conf.h"
#include "req_execution.h"
#include "Drv_DIO.h"
#include "sys_status.h"
#include <stdlib.h>
#include <math.h>
#include "Drv_Led.h"

enum
{
    RUNING_STATUS_COOLING_BPOS = 0,
    RUNING_STATUS_HEATING_BPOS,
    RUNING_STATUS_HUMIDIFYING_BPOS,
    RUNING_STATUS_DEHUMING_BPOS,
};

enum
{
    COMPRESSOR_SIG_HOLD = 0,
    COMPRESSOR_SIG_ON,
    COMPRESSOR_SIG_OFF,
    COMPRESSOR_SIG_ERR,
};

enum
{
    FAN_SIG_IDLE = 0,
    FAN_SIG_START,
    FAN_SIG_STOP
};

enum
{
    FAN_TPYE_AC = 0,
    FAN_TPYE_EC
};

typedef struct
{
    uint16_t time_out;
    uint16_t flush_delay_timer;
    uint16_t hum_fill_cnt;
    uint32_t hum_timer;
    uint32_t check_timer;
    uint8_t check_fill_flag;
    uint8_t check_drain_flag;
    uint8_t check_flag;
    uint16_t warm_time;
} hum_timer;

// static hum_timer hum_delay_timer;

//需求比特位操作函数
void req_bitmap_op(uint8_t component_bpos, uint8_t action)
{
    extern local_reg_st l_sys;
    uint8_t byte_offset, bit_offset;

    byte_offset = component_bpos >> 4;
    bit_offset  = component_bpos & 0x0f;

    if (action == 0)
    {
        l_sys.bitmap[byte_offset][BITMAP_REQ] &= ~(0x0001 << bit_offset);
    }
    else
    {
        l_sys.bitmap[byte_offset][BITMAP_REQ] |= (0x0001 << bit_offset);
    }

    //			if(action == 0)
    //			{
    //					l_sys.bitmap[BITMAP_REQ] &= ~(0x0001<<component_bpos);
    //			}
    //			else
    //			{
    //					l_sys.bitmap[BITMAP_REQ] |= (0x0001<<component_bpos);
    //			}
}

void Close_DIS_PWR(void)
{
    //    req_bitmap_op(DO_PWR_CTRL_BPOS, 1); //关闭显示电源
}

//风机档位输出
static void Fan_Fsm_Out(uint8_t Fan_Gear)
{
    switch (Fan_Gear)
    {
        case FAN_GEAR_START:
            req_bitmap_op(DO_FAN_LOW_BPOS, 1);
            break;
        case FAN_GEAR_NO:
            req_bitmap_op(DO_FAN_LOW_BPOS, 0);
            break;
        default:
            req_bitmap_op(DO_FAN_LOW_BPOS, 0);
            break;
    }
}

/**
 * @brief 	fan output control state FSM execution
 * @param  none
 * @retval none
 */
//风机状态机执行函数
static void fan_fsm_exe(uint8_t fan_signal)
{
    extern local_reg_st l_sys;
    uint16_t u16BUFF = 0;

    switch (l_sys.l_fsm_state[FAN_FSM_STATE])
    {
        case (FSM_FAN_IDLE): {
            if (fan_signal == FAN_SIG_START)
            {
                u16BUFF |= 0x01;
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_INIT;
                l_sys.comp_timeout[DO_FAN_BPOS] =
                    g_sVariable.gPara.fan.startup_delay;  // assign startup delay to timeout counter
            }
            else
            {
                u16BUFF |= 0x02;
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // remains timeout counter
            }
            l_sys.Fan.Fan_Gear = FAN_GEAR_NO;  //无输出						//disable fan output
            break;
        }
        case (FSM_FAN_INIT): {
            if (fan_signal != FAN_SIG_START)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
                l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;                      //无输出
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
            }
            else
            {
                if (l_sys.comp_timeout[DO_FAN_BPOS] == 0)
                {
                    u16BUFF |= 0x04;
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_START_UP;
                    l_sys.comp_timeout[DO_FAN_BPOS]  = 0;
                    l_sys.Fan.Fan_Gear               = FAN_GEAR_START;  //
                }
                else  // wait until startup delay elapses
                {
                    u16BUFF |= 0x08;
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_INIT;
                    l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;  //无输出
                }
            }
            break;
        }
        case (FSM_FAN_START_UP): {
            if (l_sys.comp_timeout[DO_FAN_BPOS] == 0)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
            }
            else
            {
                u16BUFF |= 0x10;
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_START_UP;
            }
            l_sys.Fan.Fan_Gear              = FAN_GEAR_START;                   //
            l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];  // remain timeout counter
            break;
        }
        case (FSM_FAN_NORM): {
            if ((fan_signal == FAN_SIG_STOP) && (l_sys.comp_timeout[DO_FAN_BPOS] == 0))
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_SHUT;
                l_sys.comp_timeout[DO_FAN_BPOS] =
                    g_sVariable.gPara.fan.stop_delay;  // assign startup delay to timeout counter
            }
            else
            {
                u16BUFF |= 0x20;
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
            }
            l_sys.Fan.Fan_Gear = FAN_GEAR_START;  //

            break;
        }
        case (FSM_FAN_SHUT): {
            if (fan_signal == FAN_SIG_START)
            {
                l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_NORM;
                l_sys.Fan.Fan_Gear               = FAN_GEAR_START;                   //
                l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
            }
            else
            {
                if (l_sys.comp_timeout[DO_FAN_BPOS] == 0)
                {
                    u16BUFF |= 0x100;
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
                    l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;                     //																			//enable
                                                                                        //fan output
                    l_sys.comp_timeout[DO_FAN_BPOS] = l_sys.comp_timeout[DO_FAN_BPOS];  // reset timeout counter
                }
                else  // wait until startup delay elapses
                {
                    u16BUFF |= 0x200;
                    l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_SHUT;
                    l_sys.comp_timeout[DO_FAN_BPOS]  = l_sys.comp_timeout[DO_FAN_BPOS];  // remain timeout counter
                }
            }
            break;
        }
        default: {
            l_sys.l_fsm_state[FAN_FSM_STATE] = FSM_FAN_IDLE;
            l_sys.Fan.Fan_Gear               = FAN_GEAR_NO;  //
            l_sys.comp_timeout[DO_FAN_BPOS]  = 0;            // reset timeout counter
            break;
        }
    }

    if ((sys_get_pwr_sts() != 0) && (l_sys.Fan.Fan_Gear != FAN_GEAR_NO))
    {
        u16BUFF |= 0x800;
        l_sys.Fan.Fan_Gear = FAN_GEAR_LOW;  //风机低挡
    }
    g_sVariable.status.REQ_TEST[1] = u16BUFF;
    Fan_Fsm_Out(l_sys.Fan.Fan_Gear);  //风机DO输出
    return;
}

/**************************************/
//风机状态机信号产生电路
static uint8_t fan_signal_gen(void)
{
    extern local_reg_st l_sys;
    uint8_t fan_signal;

    fan_signal = 0;
    if ((sys_get_pwr_sts() == TRUE) && (l_sys.Fan.Fan_Close == 0x00))
    {
        fan_signal = FAN_SIG_START;
    }
    else if ((sys_get_pwr_sts() == FALSE) || (l_sys.Fan.Fan_Close != 0x00))
    {
        if ((g_sVariable.status.u16DO_Bitmap[0] & (0x0001 << DO_COMP1_BPOS)) == 0)
        {
            fan_signal = FAN_SIG_STOP;
        }
        else
        {
            fan_signal = FAN_SIG_IDLE;
        }
    }
    else
    {
        fan_signal = FAN_SIG_IDLE;
    }
    return fan_signal;
}

//制水
void fan_req_exe(void)
{
    uint8_t fan_signal;
    fan_signal = fan_signal_gen();  //风机状态机信号产生
    fan_fsm_exe(fan_signal);        //风机状态机执行
    return;
}

static uint16_t compressor_signal_gen(uint8_t *comp1_sig)
{
    extern local_reg_st l_sys;

    if (sys_get_do_sts(DO_FAN_BPOS) == 0)  // fan disabled, emergency shutdown
    {
        *comp1_sig                                = COMPRESSOR_SIG_ERR;
        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        return 0;
    }

    if ((sys_get_pwr_sts() == FALSE) || ((l_sys.Comp.Comp_Close != 0x00)))
    {
        *comp1_sig                                = COMPRESSOR_SIG_OFF;
        l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        if (l_sys.Comp.Comp_Close & FC_ALARMSTOP)
        {
            l_sys.comp_timeout[DO_COMP1_BPOS] = 0;  //立即停机
        }

        return 0;
    }

    if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, FAN_STS_BPOS) != 0))  //风机启动
    {
        {
            //            if (get_alarm_bitmap(ACL_E1) || get_alarm_bitmap(ACL_E2)) //告警
            //            {
            //                *comp1_sig = COMPRESSOR_SIG_OFF;
            //            }
            //            else
            //            {
            *comp1_sig = COMPRESSOR_SIG_ON;
            //            }
            l_sys.l_fsm_state[COMPRESS_SIG_FSM_STATE] = 0;
        }
        return 1;
    }
    return 0;
}

//压缩机状态机函数
static void compressor_fsm(uint8_t compressor_id, uint8_t signal)
{
    extern local_reg_st l_sys;

    uint16_t compress_fsm_state;
    uint8_t l_fsm_state_id;
    uint8_t do_bpos;
    uint16_t u16BUFF;

    l_fsm_state_id = COMPRESS1_FSM_STATE;
    do_bpos        = DO_COMP1_BPOS;

    compress_fsm_state = l_sys.l_fsm_state[l_fsm_state_id];

    //		rt_kprintf("compressor_id=%d,signal=%d,,compress_fsm_state=%d,l_sys.Comp_Close=%d,l_sys.comp_startup_interval=%d\n",compressor_id,signal,compress_fsm_state,l_sys.Comp_Close,l_sys.comp_startup_interval);

    switch (compress_fsm_state)
    {
        case (COMPRESSOR_FSM_STATE_IDLE): {
            if ((signal == COMPRESSOR_SIG_ON) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
                l_sys.comp_timeout[do_bpos]       = g_sVariable.gPara.CP.u16startup_delay;

                req_bitmap_op(do_bpos, 0);
                u16BUFF |= 0x01;
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 0);
                u16BUFF |= 0x02;
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_INIT): {
            if (signal != COMPRESSOR_SIG_ON)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
                l_sys.comp_timeout[do_bpos]       = 0;
                req_bitmap_op(do_bpos, 0);
            }
            else if ((signal == COMPRESSOR_SIG_ON) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                //            if (l_sys.comp_startup_interval == 0)
                {
                    l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STARTUP;
                    l_sys.comp_timeout[do_bpos]       = g_sVariable.gPara.CP.startup_lowpress_shield;
                    req_bitmap_op(do_bpos, 1);
                    //                l_sys.comp_startup_interval = g_sys.config.ComPara.u16Comp_Interval;
                }
                //            else
                //            {
                //                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
                //                l_sys.comp_timeout[do_bpos] = l_sys.comp_timeout[do_bpos];
                //                req_bitmap_op(do_bpos, 0);
                //            }
                u16BUFF |= 0x04;
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_INIT;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 0);
                u16BUFF |= 0x08;
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_STARTUP): {
            if (signal == COMPRESSOR_SIG_ERR)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if (l_sys.comp_timeout[do_bpos] == 0)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
                l_sys.comp_timeout[do_bpos] =
                    (g_sVariable.gPara.CP.u16min_runtime > g_sVariable.gPara.CP.startup_lowpress_shield)
                        ? (g_sVariable.gPara.CP.u16min_runtime - g_sVariable.gPara.CP.startup_lowpress_shield)
                        : 0;
                req_bitmap_op(do_bpos, 1);
                u16BUFF |= 0x10;
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STARTUP;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 1);
                u16BUFF |= 0x20;
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_NORMAL): {
            if (signal == COMPRESSOR_SIG_ERR)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if ((signal == COMPRESSOR_SIG_OFF) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_SHUTING;
                l_sys.comp_timeout[do_bpos]       = g_sVariable.gPara.CP.u16stop_delay;
                req_bitmap_op(do_bpos, 1);
                u16BUFF |= 0x40;
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 1);
                u16BUFF |= 0x80;
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_SHUTING): {
            if (signal == COMPRESSOR_SIG_ERR)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
            }
            else if ((signal == COMPRESSOR_SIG_OFF) && (l_sys.comp_timeout[do_bpos] == 0))
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_STOP;
                u16BUFF |= 0x100;
            }
            else if (signal == COMPRESSOR_SIG_ON)
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_NORMAL;
                l_sys.comp_timeout[do_bpos]       = 0;
                req_bitmap_op(do_bpos, 1);
                u16BUFF |= 0x200;
            }
            else
            {
                l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_SHUTING;
                l_sys.comp_timeout[do_bpos]       = l_sys.comp_timeout[do_bpos];
                req_bitmap_op(do_bpos, 1);
                u16BUFF |= 0x400;
            }
            break;
        }
        case (COMPRESSOR_FSM_STATE_STOP): {
            l_sys.l_fsm_state[l_fsm_state_id] = COMPRESSOR_FSM_STATE_IDLE;
            l_sys.comp_timeout[do_bpos]       = g_sVariable.gPara.CP.u16min_stoptime;
            req_bitmap_op(do_bpos, 0);
            u16BUFF |= 0x800;
            break;
        }
    }
    g_sVariable.status.REQ_TEST[2] = u16BUFF;
    return;
}

// compressor requirement execution
static void compressor_req_exe(void)
{
    uint8_t comp1_sig;
    compressor_signal_gen(&comp1_sig);  // FSM signal generation

    compressor_fsm(0, comp1_sig);  // compressor 1 FSM execution

    return;
}

//除霜
void Defrost_req_exe(void)
{
    extern local_reg_st l_sys;
    int16_t i16NTC1;
    uint16_t u16BUFF;
    //    static uint16_t u16StartDefrost[2]={0};
    //    static uint16_t u16StopDefrost[2]={0};

    i16NTC1 = (int16_t)g_sVariable.status.u16AI[AI_NTC_Defrost];

    // set Deforost status
    if (i16NTC1 < (int16_t)g_sVariable.gPara.CP.Defrost.u16Start_Defrost_Temp)
    {
        u16BUFF |= 0x01;
        sys_set_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS, 1);
        l_sys.Comp.u8Defrost = TRUE;
    }
    else if (i16NTC1 > (int16_t)g_sVariable.gPara.CP.Defrost.u16Stop_Defrost_Temp)
    {
        u16BUFF |= 0x02;
        sys_set_remap_status(WORK_MODE_STS_REG_NO, DEFROST1_STS_BPOS, 0);
        l_sys.Comp.u8Defrost = FALSE;
    }
    g_sVariable.status.REQ_TEST[3] = u16BUFF;
    return;
}

#define EXITWATER_TIME 90  //
//外接水桶
static void ExitWater_exe(void)
{
    extern local_reg_st l_sys;
    uint16_t u16WL_S;

    if ((Exit_Water() == WATER_FILL))  //外接水源
    {
        u16WL_S = Get_Water_level(SW_TANK);
        if (l_sys.comp_timeout[DO_JSB_BPOS] <= 1)
        {
            if (u16WL_S < S_M)  //源水箱中水位
            {
                req_bitmap_op(DO_JSB_BPOS, 0);  //外接水泵
                g_sVariable.gPara.Water.u16ExitWater_Mode = 0;
            }
        }
        else
        {
            if (u16WL_S >= S_M)  //源水箱中水位
            {
                req_bitmap_op(DO_JSB_BPOS, 0);  //外接水泵
                g_sVariable.gPara.Water.u16ExitWater_Mode = 0;
            }
            else
            {
                req_bitmap_op(DO_JSB_BPOS, 1);  //外接水泵
            }
        }
    }
    else
    {
        l_sys.comp_timeout[DO_JSB_BPOS] = EXITWATER_TIME;
    }
    return;
}

//收集
void Collect_exe(void)
{
    extern local_reg_st l_sys;
    uint16_t u16WL_S, u16WL_D;
    static uint8_t u8PWP_S = FALSE;
    uint16_t u16BUff1      = 0;

    u16WL_S = Get_Water_level(SW_TANK);
    u16WL_D = Get_Water_level(DW_TANK);
    //源水箱高水位
    if ((u16WL_S >= S_M))
    {
        u16BUff1 |= 0x01;
        if ((u16WL_D < D_U))  //饮水箱未满
        {
            u16BUff1 |= 0x02;
            l_sys.Water.Pwp_Open = TRUE;
        }
    }
    else
    {
        u16BUff1 |= 0x04;
        l_sys.Water.Pwp_Open = FALSE;
    }
    //开收集泵、进水阀
    if ((l_sys.Water.Pwp_Open == TRUE) || (u8PWP_S == TRUE))
    {
        u16BUff1 |= 0x08;
        req_bitmap_op(DO_ZYB_BPOS, 1);
        u8PWP_S = TRUE;
        if (u16WL_S < S_L)  //到低水位退出
        {
            u16BUff1 |= 0x10;
            l_sys.Water.Pwp_Open = FALSE;
            req_bitmap_op(DO_ZYB_BPOS, 0);
            u8PWP_S = FALSE;
        }
    }
    else
    {
        u16BUff1 |= 0x20;
        req_bitmap_op(DO_ZYB_BPOS, 0);
    }
    g_sVariable.status.REQ_TEST[4] = u16BUff1;
    return;
}

//循环杀菌
void Circle_exe(void)
{
    extern local_reg_st l_sys;
    uint16_t u16WL_D;
    static uint32_t u32Sterilize_Interval;
    static uint16_t u16Sterilize_Time;
    uint16_t u16BUFF = 0;

    //饮水箱水位
    u16WL_D = Get_Water_level(DW_TANK);

    if (u16WL_D < D_L)  //饮水箱低水位
    {
        u16BUFF |= 0x01;
        req_bitmap_op(DO_LSB_BPOS, 0);  //循环泵
        return;
    }

    u32Sterilize_Interval++;
    if ((sys_get_remap_status(WORK_MODE_STS_REG_NO, OUTWATER_STS_BPOS) == TRUE))  // Water out
    {
        return;
    }
    if (u32Sterilize_Interval >= (g_sVariable.gPara.Water.u16Sterilize_Interval * 60 * SEC_FACTOR))
    {
        u16BUFF |= 0x02;
        u16Sterilize_Time++;
        req_bitmap_op(DO_LSB_BPOS, 1);  //出水泵
    }
    u16BUFF |= 0x04;
    if (u16Sterilize_Time >= g_sVariable.gPara.Water.u16Sterilize_Time * 60 * SEC_FACTOR)
    {
        u16BUFF |= 0x08;
        u32Sterilize_Interval = 0;
        u16Sterilize_Time     = 0;
        req_bitmap_op(DO_LSB_BPOS, 0);  //出水泵
    }
    g_sVariable.status.REQ_TEST[5] = u16BUFF;
    return;
}
#define POWERTIME 5400
#define THINTERVAL 180
#define CF_DELAY 60

//流量计算
uint16_t PluseCalc_Water(uint16_t PluseCnt)
{
    extern local_reg_st l_sys;
    uint16_t u16Water_Flow;
    float fWflow;

    //		if(PluseCnt<L200)//小于0.15L
    //		{
    //			fWflow=	(float)PluseCnt*L200_FACTOR;
    //		}
    //		else
    if (PluseCnt < L300)  //小于0.3L
    {
        fWflow = (float)PluseCnt * L300_FACTOR;
    }
    else if (PluseCnt < L500)
    {
        fWflow = (float)PluseCnt * L500_FACTOR;
    }
    else if (PluseCnt < L1000)
    {
        fWflow = (float)PluseCnt * L1000_FACTOR;
    }
    else if (PluseCnt < L1500)
    {
        fWflow = (float)PluseCnt * L1500_FACTOR;
    }
    else if (PluseCnt < L2000)
    {
        fWflow = (float)PluseCnt * L2000_FACTOR;
    }
    else
    {
        fWflow = (float)PluseCnt * L2000_FACTOR;
    }

    u16Water_Flow = (uint16_t)fWflow;

    return u16Water_Flow;
}
//按键出水检测
void WaterOut_Key(void)
{
    extern local_reg_st l_sys;
    //	led_toggle();
    //冷水 1
    if (sys_get_di_sts(DI_Cold_1_BPOS))
    //		if(	GPIO_ReadInputDataBit(GPIOA,GPIO_PIN_13))
    {
        l_sys.Water.OutWater_Key |= WATER_NORMAL_ICE;
        l_sys.Water.OutWater_Delay = WATER_MAXTIME;
    }
    else
    {
        l_sys.Water.OutWater_Key &= ~WATER_NORMAL_ICE;
        l_sys.Water.OutWater_Delay = 0;
    }

    //    //冷水 2
    //    if ((sys_get_di_sts(DI_Cold_2_BPOS) == 1) && (g_sys.config.ComPara.u16Water_Ctrl & TWO_COLD)) //双路出冷水
    //    {
    //        l_sys.OutWater_Key |= WATER_ICE;
    //        l_sys.OutWater_Delay[2] = WATER_MAXTIME;
    //    }
    //    else
    //    {
    //        l_sys.OutWater_Key &= ~WATER_ICE;
    //        l_sys.OutWater_Delay[2] = 0;
    //    }

    //    //		//童锁
    //    //		if((sys_get_di_sts(DI_K3_BPOS)==1))
    //    //		{
    //    //				l_sys.ChildLock_Cnt[0]++;
    //    //		}
    //    //		else
    //    //		{
    //    //				l_sys.ChildLock_Cnt[0]=0;
    //    //		}

    //    if (l_sys.ChildLock_Cnt[0] >= ChildKey_Cnt)
    //    {
    //        l_sys.ChildLock_Cnt[0] = 0;
    //        l_sys.ChildLock_Key = 1;
    //        l_sys.ChildLock_Cnt[1] = ChildKey_Lose;
    //    }
    //    //童锁使能
    //    if (l_sys.ChildLock_Key)
    //    {
    //        //热水
    //        if ((sys_get_di_sts(DI_Heat_BPOS) == 1))
    //        {
    //            l_sys.OutWater_Key |= WATER_HEAT;
    //            l_sys.OutWater_Delay[1] = WATER_MAXTIME;
    //        }
    //        else
    //        {
    //            l_sys.OutWater_Key &= ~WATER_HEAT;
    //            l_sys.OutWater_Delay[1] = 0;
    //        }
    //    }
    //    else
    //    {
    //        if ((sys_get_di_sts(DI_Heat_BPOS) == 1)) //无效
    //        {
    //        }
    //        else
    //        {
    //            l_sys.OutWater_Key &= ~WATER_HEAT;
    //            l_sys.OutWater_Delay[1] = 0;
    //        }
    //    }

    //    //童锁指示
    //    if (l_sys.ChildLock_Cnt[1])
    //    {
    //        req_bitmap_op(DO_LED_LOCK_BPOS, 1); //LED指示,反向
    //    }
    //    else
    //    {
    //        req_bitmap_op(DO_LED_LOCK_BPOS, 0); //LED指示
    //        l_sys.ChildLock_Key = 0;
    //    }

    return;
}
//关闭出水
uint8_t WaterOut_Close(uint8_t u8Type, uint8_t u8Water)
{
    extern local_reg_st l_sys;
    //    static uint8_t u8CloseNum;

    if (u8Type == 0)
    {
        //        u8CloseNum = 0;
    }
    else if (u8Type == 1)
    {
        l_sys.comp_timeout[DO_WV_WaterOut_BPOS] = 0;         //出水计时
        l_sys.Water.OutWater_Flag               = WATER_NO;  //关闭出水

        if (l_sys.Water.OutWater_OK)
        {
            l_sys.Water.OutWater_OK               = WATER_OUT;
            g_sVariable.gPara.Water.u16Water_Mode = 0;
            g_sVariable.gPara.Water.u16Water_Flow = 0;
            l_sys.Water.u8StartDelay              = g_sVariable.gPara.Water.u16StartDelay;
            if (u8Water == WATER_NORMAL_ICE)
            {
                //常温水相关

                if (g_sVariable.gPara.Water.u16CloseDelay)
                {
                    l_sys.Water.u8CloseDelay = g_sVariable.gPara.Water.u16CloseDelay;

                    req_bitmap_op(DO_LSB_BPOS, 0);  //出水泵
                }
                else
                {
                    req_bitmap_op(DO_LSB_BPOS, 0);          //出水泵
                    req_bitmap_op(DO_WV_WaterOut_BPOS, 0);  //出水阀
                    req_bitmap_op(DO_LED_WaterOut_BPOS, 0);
                }
            }
        }
        else
        {
            if ((g_sVariable.gPara.Water.u16CloseDelay) && (l_sys.Water.u8CloseDelay == 0))
            {
                req_bitmap_op(DO_WV_WaterOut_BPOS, 0);  //出水阀
                req_bitmap_op(DO_LED_WaterOut_BPOS, 0);
            }
        }
    }

    return FALSE;
}

//出水
void WaterOut_exe(void)
{
    extern local_reg_st l_sys;
    uint16_t u16WL_D;

    g_sVariable.status.REQ_TEST[6] = 0;

    //饮水箱水位
    u16WL_D = Get_Water_level(DW_TANK);

    if (g_sVariable.gPara.Water.u16Water_Ctrl & HMI_KEY)  //按键出水
    {
        g_sVariable.status.REQ_TEST[6] |= 0x01;
        WaterOut_Key();
        if ((!(l_sys.Water.OutWater_Key)) || (u16WL_D < D_L))  //饮水箱低水位
        {
            g_sVariable.status.REQ_TEST[6] |= 0x02;
            g_sVariable.gPara.Water.u16Water_Mode = 0;
            g_sVariable.gPara.Water.u16Water_Flow = 0;
            WaterOut_Close(1, WATER_NORMAL_ICE);
            //            WaterOut_Close(1, WATER_HEAT);
            //            WaterOut_Close(1, WATER_ICE);
            return;
        }
    }
    else  // HMI出水
    {
        if ((!(g_sVariable.gPara.Water.u16Water_Mode) && !(g_sVariable.gPara.Water.u16Water_Flow)) ||
            (u16WL_D < D_L))  //饮水箱低水位,不允许出水
        {
            g_sVariable.status.REQ_TEST[6] |= 0x02;
            if (u16WL_D < D_L)  //饮水箱低水位
            {
                g_sVariable.gPara.Water.u16Water_Mode = 0;
                g_sVariable.gPara.Water.u16Water_Flow = 0;
            }

            WaterOut_Close(1, WATER_NORMAL_ICE);
            //            WaterOut_Close(1, WATER_HEAT);
            //            WaterOut_Close(1, WATER_ICE);
            return;
        }
    }

    if (((g_sVariable.gPara.Water.u16Water_Mode == WATER_HEAT) && (g_sVariable.gPara.Water.u16Water_Flow)) ||
        (l_sys.Water.OutWater_Key & WATER_HEAT))  //热水
    {
        g_sVariable.status.REQ_TEST[6] |= 0x04;
    }
    else if ((((g_sVariable.gPara.Water.u16Water_Mode == WATER_NORMAL_ICE) ||
               (g_sVariable.gPara.Water.u16Water_Mode == WATER_ICE)) &&
              (g_sVariable.gPara.Water.u16Water_Flow)) ||
             (l_sys.Water.OutWater_Key & WATER_NORMAL_ICE) || (l_sys.Water.OutWater_Key & WATER_ICE))  //常温水/冰水
    {
        {
            g_sVariable.status.REQ_TEST[6] |= 0x08;
            //出水1
            req_bitmap_op(DO_LSB_BPOS, 1);  //出水阀
            //出水开始延时
            if (l_sys.Water.u8StartDelay)
            {
                l_sys.Water.u8StartDelay--;
            }
            else
            {
                l_sys.Water.u8StartDelay = 0;
                req_bitmap_op(DO_WV_WaterOut_BPOS, 1);  //出水阀
                req_bitmap_op(DO_LED_WaterOut_BPOS, 1);
            }
            l_sys.Water.OutWater_OK   = WATER_READ;
            l_sys.Water.OutWater_Flag = WATER_NORMAL_ICE;  //出水中
        }
    }
    else  //关闭出水
    {
        g_sVariable.status.REQ_TEST[6] |= 0x10;
        WaterOut_Close(1, WATER_NORMAL_ICE);
        //        WaterOut_Close(1, WATER_HEAT);
        //        WaterOut_Close(1, WATER_ICE);
    }
    //最大出水限制

    return;
}

//制冰水
void Cold_Water_exe(void)
{
    return;
}
//排水
void WaterDrain_exe(uint8_t u8Type)
{
    return;
}
//贮存
void Storage_exe(uint8_t u8Type)
{
    return;
}

// UV需求
void UV_exe(void)
{
    if (g_sVariable.gPara.Water.u16CloseUV)
    {
        req_bitmap_op(DO_LED_UV1_BPOS, 0);
        //				req_bitmap_op(DO_LED_UV2_BPOS, 0);
        req_bitmap_op(DO_LED_UV3_BPOS, 0);
        req_bitmap_op(DO_UV1_BPOS, 0);
        req_bitmap_op(DO_UV1_BPOS, 0);
        req_bitmap_op(DO_UV1_BPOS, 0);
    }
    else
    {
        req_bitmap_op(DO_LED_UV1_BPOS, 1);  //默认常开紫外灯
                                            //				req_bitmap_op(DO_LED_UV2_BPOS, 1);//默认常开紫外灯
        req_bitmap_op(DO_LED_UV3_BPOS, 1);  //默认常开紫外灯
        req_bitmap_op(DO_UV1_BPOS, 1);      //默认常开紫外灯
        req_bitmap_op(DO_UV2_BPOS, 1);      //默认常开紫外灯
        req_bitmap_op(DO_UV3_BPOS, 1);      //默认常开紫外灯
    }
    return;
}

////LED输出指示
// void LED_exe(void)
//{
//     return;
// }

//水路需求
void Water_req_exe(void)
{
    //外接水
    ExitWater_exe();
    //收集
    Collect_exe();
    //    //冰水
    //    Cold_Water_exe();
    //循环杀菌
    Circle_exe();
    //出水
    WaterOut_exe();

    //杀菌
    UV_exe();

    return;
}

//外接水源
uint8_t Exit_Water(void)
{
    uint8_t u8ExitWaterMode;
    if (g_sVariable.gPara.Water.u16ExitWater_Mode < WATER_MAXTYPE)
    {
        u8ExitWaterMode = g_sVariable.gPara.Water.u16ExitWater_Mode;
    }
    else
    {
        u8ExitWaterMode = WATER_AIR;
    }
    return u8ExitWaterMode;
}

//模式计算
static uint8_t local_Mode_req_calc(void)
{
    extern local_reg_st l_sys;

    static uint8_t u8Start_Water = FALSE;
    static uint8_t u8Offset_CNT  = 0;
    static uint8_t u8FCP_Start   = FALSE;
    static uint8_t u8FCP_Def     = FALSE;
    float fTH                    = 0;
    float fyTH                   = 0;
    int16_t i16Temp              = 0;
    int16_t i16Hum               = 0;
    int16_t i16Dew               = 0;
    uint16_t u16Buff             = 0;

    //水位保护
    /************************************************************/
    if ((Get_Water_level(SW_TANK) >= S_U))  //水满,停机
    {
        u16Buff |= 0x01;
        l_sys.Fan.Fan_Close |= FC_WL;
        l_sys.Comp.Comp_Close |= FC_WL;
        u8Start_Water = TRUE;
    }
    else
    {
        if (u8Start_Water == FALSE)  //第一次上电制水
        {
            u16Buff |= 0x02;
            if ((Get_Water_level(SW_TANK) < S_U))  //
            {
                l_sys.Fan.Fan_Close &= ~FC_WL;
                l_sys.Comp.Comp_Close &= ~FC_WL;
            }
        }
        else
        {
            u16Buff |= 0x04;
            if ((Get_Water_level(SW_TANK) < S_M))  //
            {
                l_sys.Fan.Fan_Close &= ~FC_WL;
                l_sys.Comp.Comp_Close &= ~FC_WL;
                u8Start_Water = FALSE;
            }
        }
    }

    /************************************************************/
    //温湿度限制
    /************************************************************/
    /****************露点温度计算*****************/
    /*
    Td=by(T,RH)/(a-y(T,RH))
    y(T,RH)=aT/b+T +In(RH/100)
    a=17.27，b=237.7
    */
    //		g_sys.status.ComSta.u16TH[0].Temp=90;
    //		g_sys.status.ComSta.u16TH[0].Hum=670;
    i16Temp                            = g_sVariable.status.u16TH[0].Temp;
    i16Hum                             = g_sVariable.status.u16TH[0].Hum;
    fTH                                = (float)i16Hum / 10 / 100;
    fyTH                               = D_A * (float)i16Temp / 10 / (D_B + (float)i16Temp / 10) + log(fTH);
    fTH                                = D_B * fyTH / (D_A - fyTH);
    i16Dew                             = (int16_t)(fTH * 10);  //放大10倍
    g_sVariable.status.u16TH[0].u16Dew = i16Dew;               //放大10倍

    u8Offset_CNT++;
    if (u8Offset_CNT >= CF_DELAY)  //上电延时判断温湿度
    {
        u8Offset_CNT = CF_DELAY;
    }
    if ((l_sys.Water.TH_Check_Interval == 0) && (u8Offset_CNT >= CF_DELAY))
    {
        l_sys.Water.TH_Check_Interval = g_sVariable.gPara.TH.u16TH_Interal;  //温湿度判断间隔
        if (u8FCP_Start == FALSE)                                            //冷启动
        {
            if (i16Dew > (int16_t)g_sVariable.gPara.TH.u16Start_Temp)  //温湿度满足条件
            {
                u16Buff |= 0x100;
                l_sys.Fan.Fan_Close &= ~FC_TH;
                l_sys.Comp.Comp_Close &= ~FC_TH;
                u8FCP_Start = TRUE;  //启动
            }
            else
            {
                u16Buff |= 0x200;
                l_sys.Fan.Fan_Close |= FC_TH;
                l_sys.Comp.Comp_Close |= FC_TH;
            }
        }
        else
        {
            if (i16Dew < (int16_t)g_sVariable.gPara.TH.u16Stop_Temp)  //温湿度不满足条件
            {
                u16Buff |= 0x400;
                l_sys.Fan.Fan_Close |= FC_TH;
                l_sys.Comp.Comp_Close |= FC_TH;
                u8FCP_Start = FALSE;
            }
        }
    }
    ///************************************************************/
    ////外接水源
    ///************************************************************/
    //    if ((Exit_Water() != WATER_AIR)) //外接水源
    //    {
    //        u16Buff |= 0x800;
    //        l_sys.Fan.Fan_Close |= FC_WS;
    //        l_sys.Comp.Comp_Close |= FC_WS;
    //    }
    //    else
    //    {
    //        u16Buff |= 0x1000;
    //        l_sys.Fan.Fan_Close &= ~FC_WS;
    //        l_sys.Comp.Comp_Close &= ~FC_WS;
    //    }
    /************************************************************/
    //告警停机

    if (Get_alarm_arbiration())
    {
        u16Buff |= 0x1000;
        l_sys.Fan.Fan_Close |= FC_ALARMSTOP;
        l_sys.Comp.Comp_Close |= FC_ALARMSTOP;
    }
    else
    {
        l_sys.Fan.Fan_Close &= ~FC_ALARMSTOP;
        l_sys.Comp.Comp_Close &= ~FC_ALARMSTOP;
    }
    /************************************************************/
    // CP高温
    if ((g_sVariable.status.u16AI[AI_NTC_Warm] >= 1250) && (g_sVariable.status.u16AI[AI_NTC_Warm] != ABNORMAL_VALUE))
    {
        u16Buff |= 0x2000;
        l_sys.Comp.Comp_Close |= FC_NT;
    }
    else
    {
        l_sys.Comp.Comp_Close &= ~FC_NT;
    }

    /************************************************************/
    //除霜
    /************************************************************/
    if (l_sys.Comp.u8Defrost == TRUE)
    {
        u16Buff |= 0x4000;
        l_sys.Comp.Comp_Close |= FC_DF;
        u8FCP_Def = TRUE;
    }
    else
    {
        l_sys.Comp.Comp_Close &= ~FC_DF;
        if (u8FCP_Def == TRUE)
        {
            l_sys.Water.TH_Check_Interval = 0;  //除霜后，需要检测温湿度
            u8FCP_Start                   = FALSE;
            u8FCP_Def                     = FALSE;
            u16Buff |= 0x8000;
        }
    }
    g_sVariable.status.Water.u16FC_ON = l_sys.Fan.Fan_Close;
    g_sVariable.status.REQ_TEST[0]    = u16Buff;

    //		rt_kprintf("u8Mode_state=%d,u16Mode_req=%d,u32CompRunTime=%d,Fan_Close=%x\n", u8Mode_state,
    //l_sys.Comp.u16Mode_req,l_sys.Comp.u32CompRunTime, l_sys.Fan.Fan_Close);
    return TRUE;
}

//开关机处理
void Power_OnOff_exe()
{
    extern local_reg_st l_sys;
    //开机
    if (sys_get_pwr_sts() == TRUE)
    {
        //				g_sys.config.ComPara.u16Manual_Test_En=MANUAL_TEST_UNABLE;
        //				g_sys.config.ComPara.u16Test_Mode_Type=TEST_UNABLE;
    }
    else
    {
        l_sys.Comp.u8Last_Status = LAST_ONOFF;
    }
    return;
}

//制水需求
void Product_req_exe(void)
{
    //开关机处理
    Power_OnOff_exe();
    //模式判断
    local_Mode_req_calc();
    //风机控制
    fan_req_exe();
    //压缩机控制
    compressor_req_exe();
    //除霜
    Defrost_req_exe();
    return;
}

//总体需求执行逻辑
void req_execution(void)
{
    //制水
    Product_req_exe();
    //水路
    Water_req_exe();
    return;
}
