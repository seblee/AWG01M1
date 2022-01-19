#include "cmsis_os.h"  // ARM::CMSIS:RTOS:Keil RTX
#include "global.h"
#include "calc.h"
#include "sys_conf.h"
#include "req_execution.h"
#include "Drv_DIO.h"
#include "sys_status.h"
#include "math.h"
#include "stdlib.h"

void sys_set_remap_status(uint8_t reg_no, uint8_t sbit_pos, uint8_t bit_action)
{
    if (bit_action == 1)
    {
        g_sVariable.status.u16Status_remap[reg_no] |= (0x0001 << sbit_pos);
    }
    else
    {
        g_sVariable.status.u16Status_remap[reg_no] &= ~(0x0001 << sbit_pos);
    }
}

uint16_t sys_get_remap_status(uint8_t reg_no, uint8_t rbit_pos)
{
    //		return ((g_sys.status.ComSta.u16Status_remap[reg_no] >> rbit_pos) & 0x0001);
    return ((g_sVariable.status.u16Status_remap[reg_no] >> rbit_pos) & 0x0001);
}

uint8_t sys_get_di_sts(uint8_t din_channel)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = din_channel >> 4;
    bit_offset  = din_channel & 0x0f;
    //		if((g_sys.status.ComSta.u16Din_bitmap[byte_offset]>>bit_offset) & 0X0001)
    if ((g_sVariable.status.u16DI_Bitmap[byte_offset] >> bit_offset) & 0X0001)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//数字输入状态
void sys_option_di_sts(uint8_t din_channel, uint8_t option)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = din_channel >> 4;
    bit_offset  = din_channel & 0x0f;

    if (option)
    {
        g_sVariable.status.u16DI_Bitmap[byte_offset] |= (0x0001 << bit_offset);
    }
    else
    {
        g_sVariable.status.u16DI_Bitmap[byte_offset] &= (~(0x0001 << bit_offset));
    }
}
uint8_t sys_get_do_sts(uint8_t dout_channel)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = dout_channel >> 4;
    bit_offset  = dout_channel & 0x0f;
    if ((g_sVariable.status.u16DO_Bitmap[byte_offset] >> bit_offset) & 0X0001)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t sys_get_pwr_signal(void)
{
    uint8_t ret;
    //		if((sys_get_di_sts(DI_ONOFF_BPOS) == 1)&&
    //			(g_sys.config.ComPara.u16Power_Mode == 1))
    if (g_sVariable.gPara.u16Power_Mode == 1)
    {
        ret = TRUE;
    }
    else
    {
        ret = FALSE;
    }
    return ret;
}

void sys_running_mode_update(void)
{
    extern local_reg_st l_sys;
    // FAN STATUS UPDATE

    if (sys_get_pwr_signal() == TRUE)  //开关机信号
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, PWR_STS_BPOS, 1);
    }
    else
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, PWR_STS_BPOS, 0);
    }

    if (sys_get_do_sts(DO_FAN_BPOS) == 1)
    //    if (l_sys.Fan.u8FanOK!=FSM_FAN_IDLE)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, FAN_STS_BPOS, 1);
    }
    else
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, FAN_STS_BPOS, 0);
    }

    // set cooling status
    if (sys_get_do_sts(DO_COMP1_BPOS) == 1)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, COOLING_STS_BPOS, 1);
    }
    else
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, COOLING_STS_BPOS, 0);
    }

    // set Prowater status
    if ((sys_get_do_sts(DO_COMP1_BPOS) == 1) && (sys_get_do_sts(DO_FAN_BPOS) == 1))
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, PROWATER_STS_BPOS, 1);
    }
    else
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, PROWATER_STS_BPOS, 0);
    }

    if (l_sys.Water.OutWater_Flag)
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, OUTWATER_STS_BPOS, 1);
    }
    else
    {
        sys_set_remap_status(WORK_MODE_STS_REG_NO, OUTWATER_STS_BPOS, 0);
    }
    return;
}

uint16_t sys_get_pwr_sts(void)
{
    //		if((g_sys.status.ComSta.u16Status_remap[WORK_MODE_STS_REG_NO]>>PWR_STS_BPOS) & 0X0001)
    if ((g_sVariable.status.u16Status_remap[WORK_MODE_STS_REG_NO] >> PWR_STS_BPOS) & 0X0001)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

////浮球水位
// uint16_t Get_Water_level(void)
//{
//     extern sys_reg_st g_sys;
//     uint16_t u16Water_level = 0;

//    //极性
//    //S_L
//    if (sys_get_di_sts(DI_SOURCE_DOWN_BPOS) == 1)
//    {
//        u16Water_level |= S_L;
//    }
//    else
//    {
//        u16Water_level &= ~S_L;
//    }

//    //S_U
//    if (sys_get_di_sts(DI_SOURCE_UP_BPOS) == 1)
//    {
//        u16Water_level |= S_U;
//    }
//    else
//    {
//        u16Water_level &= ~S_U;
//    }

//    g_sys.status.ComSta.Water.u16S_WL = u16Water_level;
//    return u16Water_level;
//}

//源饮水箱低水位判断
uint16_t Get_Water_level(uint8_t u8Type)
{
    uint16_t u16WL      = 0;
    uint16_t u16AI_TANK = 0;

    if (u8Type == SW_TANK)
    {
        u16AI_TANK = g_sVariable.status.u16AI[AI_NTC_STANK];
        // S_L
        if (abs(u16AI_TANK - WL_L) <= WL_OFFSET)
        {
            u16WL = S_L;
        }
        else if (abs(u16AI_TANK - WL_M) <= WL_OFFSET)
        {
            u16WL = S_M;
        }
        else if (abs(u16AI_TANK - WL_U) <= WL_OFFSET)
        {
            u16WL = S_U;
        }
        else
        {
            u16WL = S_NO;
        }
        g_sVariable.status.Water.u16WL &= 0xFF00;
        g_sVariable.status.Water.u16WL |= u16WL;
        return u16WL;
    }
    else if (u8Type == DW_TANK)  //饮水箱水位
    {
        u16AI_TANK = g_sVariable.status.u16AI[AI_NTC_DTANK];
        // D_L
        if (abs(u16AI_TANK - WL_L) <= WL_OFFSET)
        {
            u16WL = D_L;
        }
        else if (abs(u16AI_TANK - WL_M) <= WL_OFFSET)
        {
            u16WL = D_M;
        }
        else if (abs(u16AI_TANK - WL_U) <= WL_OFFSET)
        {
            u16WL = D_U;
        }
        else
        {
            u16WL = D_NO;
        }
        g_sVariable.status.Water.u16WL &= 0x00FF;
        g_sVariable.status.Water.u16WL |= (u16WL << 8);
        return u16WL;
    }
    return 0;
}

// UV状态
uint16_t Get_UV_Status(void)
{
    uint16_t u16UVStatus = 0;
    // LED_UV1
    if (sys_get_do_sts(DO_LED_UV1_BPOS) == 1)
    {
        if ((g_sVariable.gPara.dev_mask.ain) & (0x01 << AI_LEDUV1_V))
        {
            if (g_sVariable.status.u16AI[AI_LEDUV1_V] > UV_OFFSET)
            {
                u16UVStatus &= ~UV1_ON;
            }
            else
            {
                u16UVStatus |= UV1_ON;
                u16UVStatus |= UV_ERR;
            }
        }
    }
    else
    {
        u16UVStatus &= ~UV1_ON;
    }
    //		//LED_UV2
    //    if (sys_get_do_sts(DO_LED_UV2_BPOS) == 1)
    //    {
    //			if ((g_sVariable.gPara.dev_mask.din) & (0x01 << DI_LED_UV2_BPOS))
    //			{
    //        if(sys_get_di_sts(DI_LED_UV2_BPOS))
    //				{
    //					u16UVStatus&= ~UV2_ON;
    //				}
    //				else
    //				{
    //					u16UVStatus|=UV2_ON;
    //					u16UVStatus|=UV_ERR;
    //				}
    //			}
    //    }
    //    else
    //    {
    //					u16UVStatus&= ~UV2_ON;
    //    }
    // LED_UV3
    if (sys_get_do_sts(DO_LED_UV3_BPOS) == 1)
    {
        if ((g_sVariable.gPara.dev_mask.ain) & (0x01 << AI_LEDUV3_V))
        {
            if (g_sVariable.status.u16AI[AI_LEDUV3_V] > UV_OFFSET)
            {
                u16UVStatus &= ~UV3_ON;
            }
            else
            {
                u16UVStatus |= UV3_ON;
                u16UVStatus |= UV_ERR;
            }
        }
    }
    else
    {
        u16UVStatus &= ~UV3_ON;
    }
    g_sVariable.status.Water.u16UVStatus = u16UVStatus;
    return u16UVStatus;
}

// Filiter状态
uint16_t Get_Filiter_Status(uint8_t u8Type)
{
    uint16_t u16Filiter_OF    = 0;
    uint16_t u16RemianTime    = 0;
    uint16_t u16FiliterStatus = 0;
    // LED_UV1
    switch (u8Type)
    {
        case ACL_FILLTER: {
            u16Filiter_OF = g_sVariable.gPara.alarm[ACL_FILLTER].alarm_param / 4;
            u16RemianTime =
                g_sVariable.gPara.alarm[ACL_FILLTER].alarm_param - g_sVariable.status.u16Runtime[1][DO_FILLTER_BPOS];
            u16FiliterStatus                   = (u16RemianTime + u16Filiter_OF - 1) / u16Filiter_OF;
            g_sVariable.status.Water.u16Fliter = u16FiliterStatus;
        }
        break;
        case ACL_FILLTER_ELEMENT_0: {
            u16Filiter_OF = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_0].alarm_param / 4;
            u16RemianTime = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_0].alarm_param -
                            g_sVariable.status.u16Runtime[1][DO_FILLTER_ELEMENT_BPOS_0];
            u16FiliterStatus                             = (u16RemianTime + u16Filiter_OF - 1) / u16Filiter_OF;
            g_sVariable.status.Water.u16FliterElement[0] = u16FiliterStatus;
        }
        break;
        case ACL_FILLTER_ELEMENT_1: {
            u16Filiter_OF = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_1].alarm_param / 4;
            u16RemianTime = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_1].alarm_param -
                            g_sVariable.status.u16Runtime[1][DO_FILLTER_ELEMENT_BPOS_1];
            u16FiliterStatus                             = (u16RemianTime + u16Filiter_OF - 1) / u16Filiter_OF;
            g_sVariable.status.Water.u16FliterElement[1] = u16FiliterStatus;
        }
        break;
        case ACL_FILLTER_ELEMENT_2: {
            u16Filiter_OF = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_2].alarm_param / 4;
            u16RemianTime = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_2].alarm_param -
                            g_sVariable.status.u16Runtime[1][DO_FILLTER_ELEMENT_BPOS_2];
            u16FiliterStatus                             = (u16RemianTime + u16Filiter_OF - 1) / u16Filiter_OF;
            g_sVariable.status.Water.u16FliterElement[2] = u16FiliterStatus;
        }
        break;
        case ACL_FILLTER_ELEMENT_3: {
            u16Filiter_OF = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_3].alarm_param / 4;
            u16RemianTime = g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_3].alarm_param -
                            g_sVariable.status.u16Runtime[1][DO_FILLTER_ELEMENT_BPOS_3];
            u16FiliterStatus                             = (u16RemianTime + u16Filiter_OF - 1) / u16Filiter_OF;
            g_sVariable.status.Water.u16FliterElement[3] = u16FiliterStatus;
        }
        break;
        default:

            break;
    }

    return u16FiliterStatus;
}
