/*********************************************************
  Copyright (C), 2014, Sunrise Group. Co., Ltd.
  File name:      	com_proc.c
  Author: gongping	Version: 0.7       Date:  2014-12-05
  Description:    	Global variables instantation
  Others:         	n.a
  Function List:  	void g_var_init(void);		//global variables initialization
  Variable List:  	g_var_st g_var_st_inst;		//global variables structure
  Revision History:
  Date:           Author:          Modification:
    2014-12-05      gongping         file create
*********************************************************/

#include "global.h"
#include "string.h"
#include "fifo.h"
#include "Lib_CRC.h"
#include "Drv_DIO.h"

sVariable g_sVariable;
local_reg_st l_sys;  // local status declairation
volatile fifo8_cb_td fifo_rx_buf;

/* ----------------------- Defines ------------------------------------------*/

uint16_t conf_reg[CONF_REG_MAP_NUM];
uint16_t test_reg[CONF_REG_MAP_NUM];
typedef enum
{
    INIT_LOAD_USR = 0,
    INIT_LOAD_FACT,
    INIT_LOAD_DEBUT,
    INIT_LOAD_DEFAULT,
} init_state_em;

#define EE_FLAG_LOAD_USR 0xdf
#define EE_FLAG_LOAD_FACT 0x1b
#define EE_FLAG_LOAD_DFT 0x9b
#define EE_FLAG_LOAD_DEBUT 0xff

const conf_reg_map_st conf_reg_map_inst[REG_MAP_NUM] = {
    // id			 						mapped registers									                min			max					default
    // permission
    // r/w
    // chk_ptr
    {0, &g_sVariable.gPara.Num, 0, 0xFFFF, REG_HOLDING_NREGS, 0, NULL},
    {1, &g_sVariable.gPara.Software, 0, 0xFFFF, MB_SOFTWARE_VER, 0, NULL},
    {2, &g_sVariable.gPara.Hardware, 0, 0xFFFF, MB_HARDWARE_VER, 0, NULL},
    {3, NULL, 0, 0xFFFF, 0, 0, NULL},
    {4, NULL, 0, 0xFFFF, 0, 0, NULL},
    {5, &g_sVariable.gPara.u16Power_Mode, 0, 1, 1, 1, NULL},
    {MB_CFG_WorkMode, &g_sVariable.gPara.WorkMode, 0, 1, 0, 1, NULL},
    {FACTORY_RESET, &g_sVariable.gPara.CMD, 0, 0xFFFF, 0, 1, NULL},
    {MB_CFG_ADDR, &g_sVariable.gPara.CommAddress, 0, 247, 1, 1, NULL},
    {MB_CFG_BAUDRATE, &g_sVariable.gPara.Bardrate, 0, 19200, 19200, 1, NULL},
    {10, &g_sVariable.gPara.u16RS, 0, 0xFFFF, 0, 1, NULL},
    {11, &g_sVariable.gPara.dev_mask.din_bitmap_polarity, 0, 0xFFFF, 0x0F, 1, NULL},
    {12, &g_sVariable.gPara.dev_mask.din, 0, 0xFFFF, 0x0F, 1, NULL},
    {13, &g_sVariable.gPara.dev_mask.dout[0], 0, 0xFFFF, 0xFFFF, 1, NULL},
    {14, &g_sVariable.gPara.dev_mask.dout[1], 0, 0xFFFF, 0x007F, 1, NULL},
    {15, &g_sVariable.gPara.dev_mask.ain, 0, 0xFFFF, 0x01B6, 1, NULL},
    {16, NULL, 0, 0xFFFF, 0, 0, NULL},
    {MANUAL_TSET, &g_sVariable.gPara.u16Manual_Test_En, 0, 3, 0, 1, NULL},
    {18, &g_sVariable.gPara.diagnose_mode_en, 0, 1, 0, 1, NULL},
    {19, &l_sys.bitmap[0][BITMAP_MANUAL], 0, 0xffff, 0, 1, NULL},
    {20, &l_sys.bitmap[1][BITMAP_MANUAL], 0, 0xffff, 0, 1, NULL},
    {21, NULL, 0, 0xFFFF, 0, 1, NULL},
    {CLEAR_RT, &g_sVariable.gPara.u16Clear_RT, 0, 0xFFFF, 0, 1, NULL},
    {CLEAR_ALARM, &g_sVariable.gPara.u16Clear_ALARM, 0, 0xFFFF, 0, 1, NULL},
    {24, NULL, 0, 0xFFFF, 0, 0, NULL},
    {25, &g_sVariable.gPara.TH.u16TH_Interal, 0, 0xFFFF, 180, 1, NULL},
    {26, &g_sVariable.gPara.TH.u16Start_Temp, 0, 800, 20, 1, NULL},
    {27, &g_sVariable.gPara.TH.u16Stop_Temp, 0, 800, 5, 1, NULL},
    {28, NULL, 0, 0xFFFF, 0, 0, NULL},
    {29, NULL, 0, 0xFFFF, 0, 0, NULL},
    {30, &g_sVariable.gPara.TH.u16TH_Cali[0], 0, 0xFFFF, 0, 1, NULL},
    {31, &g_sVariable.gPara.TH.u16TH_Cali[1], 0, 0xFFFF, 0, 1, NULL},
    {32, &g_sVariable.gPara.TH.u16NTC_Cali[0], 0, 0xFFFF, 0, 1, NULL},
    {33, &g_sVariable.gPara.TH.u16NTC_Cali[1], 0, 0xFFFF, 0, 1, NULL},
    {34, &g_sVariable.gPara.TH.u16NTC_Cali[2], 0, 0xFFFF, 0, 1, NULL},
    {35, &g_sVariable.gPara.TH.u16NTC_Cali[3], 0, 0xFFFF, 0, 1, NULL},
    {36, NULL, 0, 0xFFFF, 0, 0, NULL},
    {37, NULL, 0, 0xFFFF, 0, 0, NULL},
    {38, &g_sVariable.gPara.fan.startup_delay, 0, 600, 5, 1, NULL},
    {39, &g_sVariable.gPara.fan.stop_delay, 0, 600, 30, 1, NULL},
    {40, &g_sVariable.gPara.CP.u16startup_delay, 0, 600, 0, 1, NULL},
    {41, &g_sVariable.gPara.CP.u16stop_delay, 0, 600, 5, 1, NULL},
    {42, &g_sVariable.gPara.CP.u16min_runtime, 0, 600, 180, 1, NULL},
    {43, &g_sVariable.gPara.CP.u16min_stoptime, 0, 600, 300, 1, NULL},
    {44, &g_sVariable.gPara.CP.startup_lowpress_shield, 0, 600, 120, 1, NULL},
    {45, NULL, 0, 0xFFFF, 0, 0, NULL},
    {46, &g_sVariable.gPara.CP.Defrost.u16Start_Defrost_Temp, 0, 0xFFFF, (uint16_t)-30, 1, NULL},
    {47, &g_sVariable.gPara.CP.Defrost.u16Stop_Defrost_Temp, 0, 0xFFFF, 30, 1, NULL},
    {48, NULL, 0, 0xFFFF, 0, 0, NULL},
    {49, NULL, 0, 0xFFFF, 0, 0, NULL},
    {50, &g_sVariable.gPara.Water.u16Water_Ctrl, 0, 0xFF, 2, 1, NULL},
    {51, &g_sVariable.gPara.Water.u16Water_Mode, 0, 3, 0, 1, NULL},
    {52, &g_sVariable.gPara.Water.u16Water_Flow, 0, 65530, 1000, 1, NULL},
    {53, &g_sVariable.gPara.Water.u16ExitWater_Mode, 0, 4, 0, 1, NULL},
    {54, &g_sVariable.gPara.Water.u16CloseUV, 0, 1, 0, 1, NULL},
    {55, &g_sVariable.gPara.Water.u16Sterilize_Interval, 0, 10000, 60, 1, NULL},
    {56, &g_sVariable.gPara.Water.u16Sterilize_Time, 0, 10000, 3, 1, NULL},
    {57, &g_sVariable.gPara.Water.u16HotWater_Temp, 0, 10000, 900, 1, NULL},
    {58, NULL, 0, 0xFFFF, 0, 1, NULL},
    {59, NULL, 0, 0xFFFF, 0, 1, NULL},
    {60, NULL, 0, 0xFFFF, 0, 1, NULL},
    {61, NULL, 0, 0xFFFF, 0, 1, NULL},
    {62, NULL, 0, 0xFFFF, 0, 1, NULL},
    {63, NULL, 0, 0xFFFF, 0, 1, NULL},
    {64, &g_sVariable.gPara.Water.u16StartDelay, 0, 10, 0, 1, NULL},
    {65, &g_sVariable.gPara.Water.u16CloseDelay, 0, 10, 0, 1, NULL},
    {66, &g_sVariable.gPara.Water.u16FILTER_ELEMENT_Type, 0, 1, 0, 1, NULL},
    {67, NULL, 0, 0xFFFF, 0, 0, NULL},
    {68, &g_sVariable.gPara.Water.u16TDS_Cail, 0, 0xFFFF, 0, 1, NULL},
    //		{	69,			 					&g_sVariable.gPara.Water.u16TDS_Cail[1],					0,
    //0xFFFF,
    // 0,
    // 1,      NULL},
    {69, NULL, 0, 0xFFFF, 0, 0, NULL},
    {70, &g_sVariable.gPara.alarm[ACL_FILLTER].alarm_param, 1, 65535, 4320, 1, NULL},
    {71, &g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_0].alarm_param, 1, 65535, 4320, 1, NULL},
    {72, &g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_1].alarm_param, 1, 65535, 4320, 1, NULL},
    {73, &g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_2].alarm_param, 1, 65535, 8640, 1, NULL},
    {74, &g_sVariable.gPara.alarm[ACL_FILLTER_ELEMENT_3].alarm_param, 1, 65535, 4320, 1, NULL},
    {75, NULL, 0, 0xFFFF, 0, 0, NULL},
    {76, NULL, 0, 0xFFFF, 0, 0, NULL},
    {77, NULL, 0, 0xFFFF, 0, 0, NULL},
    {78, NULL, 0, 0xFFFF, 0, 0, NULL},
    {79, NULL, 0, 0xFFFF, 0, 0, NULL},
    {80, &g_sVariable.status.u16Status_remap[0], 0, 0xFFFF, 0, 0, NULL},
    {81, &g_sVariable.status.u16Status_remap[1], 0, 0xFFFF, 0, 0, NULL},
    {82, &g_sVariable.status.u16Status_remap[2], 0, 0xFFFF, 0, 0, NULL},
    {83, &g_sVariable.status.alarm_bitmap[0], 0, 0xFFFF, 0, 0, NULL},
    {84, &g_sVariable.status.Water.u16FC_ON, 0, 0xFFFF, 0, 0, NULL},
    {85, &g_sVariable.status.u16AI[0], 0, 0xFFFF, 0, 0, NULL},
    {86, &g_sVariable.status.u16AI[1], 0, 0xFFFF, 0, 0, NULL},
    {87, &g_sVariable.status.u16AI[2], 0, 0xFFFF, 0, 0, NULL},
    {88, &g_sVariable.status.u16AI[3], 0, 0xFFFF, 0, 0, NULL},
    {89, &g_sVariable.status.u16AI[4], 0, 0xFFFF, 0, 0, NULL},
    {90, &g_sVariable.status.u16AI[5], 0, 0xFFFF, 0, 0, NULL},
    {91, &g_sVariable.status.u16AI[6], 0, 0xFFFF, 0, 0, NULL},
    {92, &g_sVariable.status.u16AI[7], 0, 0xFFFF, 0, 0, NULL},
    {93, &g_sVariable.status.u16AI[8], 0, 0xFFFF, 0, 0, NULL},
    {94, &g_sVariable.status.u16AI[9], 0, 0xFFFF, 0, 0, NULL},
    {95, &g_sVariable.status.u16DI_Bitmap[0], 0, 0xFFFF, 0, 0, NULL},
    {96, &g_sVariable.status.u16DO_Bitmap[0], 0, 0xFFFF, 0, 0, NULL},
    {97, &g_sVariable.status.u16DO_Bitmap[1], 0, 0xFFFF, 0, 0, NULL},
    {98, &g_sVariable.status.u16TH[0].Temp, 0, 0xFFFF, 0, 0, NULL},
    {99, &g_sVariable.status.u16TH[0].Hum, 0, 0xFFFF, 0, 0, NULL},
    {100, &g_sVariable.status.u16TH[0].u16Dew, 0, 0xFFFF, 0, 0, NULL},
    {101, &g_sVariable.status.u16TH[1].Temp, 0, 0xFFFF, 0, 0, NULL},
    {102, &g_sVariable.status.u16TH[1].Hum, 0, 0xFFFF, 0, 0, NULL},
    {103, &g_sVariable.status.Water.u16WL, 0, 0xFFFF, 0, 0, NULL},
    {104, &g_sVariable.status.Water.u16UVStatus, 0, 0xFFFF, 0, 0, NULL},
    {105, &g_sVariable.status.Water.u16Fliter, 0, 0xFFFF, 0, 0, NULL},
    {106, &g_sVariable.status.Water.u16FliterElement[0], 0, 0xFFFF, 0, 0, NULL},
    {107, &g_sVariable.status.Water.u16FliterElement[1], 0, 0xFFFF, 0, 0, NULL},
    {108, &g_sVariable.status.Water.u16FliterElement[2], 0, 0xFFFF, 0, 0, NULL},
    {109, &g_sVariable.status.Water.u16FliterElement[3], 0, 0xFFFF, 0, 0, NULL},
    {110, &g_sVariable.status.Water.u16TDS[0], 0, 0xFFFF, 0, 0, NULL},
    {111, &g_sVariable.status.Water.u16TDS[1], 0, 0xFFFF, 0, 0, NULL},
    {112, &g_sVariable.status.REQ_TEST[0], 0, 0xFFFF, 0, 0, NULL},
    {113, &g_sVariable.status.REQ_TEST[1], 0, 0xFFFF, 0, 0, NULL},
    {114, &g_sVariable.status.REQ_TEST[2], 0, 0xFFFF, 0, 0, NULL},
    {115, &g_sVariable.status.REQ_TEST[3], 0, 0xFFFF, 0, 0, NULL},
    {116, &g_sVariable.status.REQ_TEST[4], 0, 0xFFFF, 0, 0, NULL},
    {117, &g_sVariable.status.REQ_TEST[5], 0, 0xFFFF, 0, 0, NULL},
    {118, &g_sVariable.status.REQ_TEST[6], 0, 0xFFFF, 0, 0, NULL},
    {119, &g_sVariable.status.REQ_TEST[7], 0, 0xFFFF, 0, 0, NULL},
};
/* ----------------------- Static variables ---------------------------------*/

/**
  * @brief  get eeprom program status
  * @param  None
  * @retval
        `EE_FLAG_OK:		configuration data valid in eeprom
        `EE_FLAG_EMPTY:	eeprom empty
  */

static init_state_em get_ee_status(void)
{
    init_state_em em_init_state;
    uint8_t ee_pflag;
    // wait for eeprom power on
    I2C_EE_BufRead(&ee_pflag, STS_EE_ADDR, 1);  //启动区
#ifdef RSTORE
    if (GetSEL())  // Original parameter
    {
        ee_pflag = EE_FLAG_LOAD_DEBUT;
    }
#endif
    //		//TEST
    //		ee_pflag =INIT_LOAD_DEBUT;
    switch (ee_pflag)
    {
        case (EE_FLAG_LOAD_USR): {
            em_init_state = INIT_LOAD_USR;
            break;
        }

        case (EE_FLAG_LOAD_FACT): {
            em_init_state = INIT_LOAD_FACT;
            break;
        }
        case (EE_FLAG_LOAD_DFT): {
            em_init_state = INIT_LOAD_DEFAULT;
            break;
        }
        default: {
            em_init_state = INIT_LOAD_DEBUT;
            break;
        }
    }
    return em_init_state;
}

/**
  * @brief 	save system configurable variables initialization
    * @param  0:load usr1 eeprom
                        1:load usr2 eeprom
                        2:load facotry eeprom
                        3:load default eeprom
    * @retval err_cnt: mismatch read/write data count
  */

uint16_t set_load_flag(uint8_t ee_load_flag)
{
    uint8_t ee_flag;
    switch (ee_load_flag)
    {
        case (0): {
            ee_flag = EE_FLAG_LOAD_USR;
            break;
        }
        case (1): {
            ee_flag = EE_FLAG_LOAD_FACT;
            break;
        }
        case (2): {
            ee_flag = EE_FLAG_LOAD_DEBUT;
            break;
        }
        default: {
            ee_flag = EE_FLAG_LOAD_DFT;
            break;
        }
    }
    I2C_EE_BufWrite(&ee_flag, STS_EE_ADDR, 1);
    return 1;
}

/**
  * @brief 	save system configurable variables initialization
    * @param  addr_sel:
                        `0: save current configuration to usr1 eeprom address
                        `1:	save current configuration to usr2 eeprom address
                        `2:	save current configuration to facotry eeprom address
    * @retval err_cnt: mismatch read/write data count
  */
uint16_t save_conf_reg(uint8_t addr_sel)
{
    // uint16_t conf_reg[CONF_REG_MAP_NUM];
    //		uint16_t test_reg[CONF_REG_MAP_NUM];
    uint16_t i, j, err_cnt, chk_res;
    uint16_t ee_save_addr;
    uint8_t ee_flag, req;

    ee_save_addr = 0;
    err_cnt      = 0;

    switch (addr_sel)
    {
        case (0): {
            ee_flag = EE_FLAG_LOAD_USR;
            break;
        }
        case (1): {
            ee_save_addr = CONF_REG_FACT_ADDR;
            ee_flag      = EE_FLAG_LOAD_FACT;
            break;
        }
        default: {
            return 0xff;
        }
    }

    for (i = 0; i < CONF_REG_MAP_NUM; i++)  // set configration reg with default value
    {
        conf_reg[i] = *(conf_reg_map_inst[i].reg_ptr);
    }
    if (ee_flag == EE_FLAG_LOAD_USR)
    {
        req = 0;
        for (j = 0; j < 3; j++)
        {
            if (j == 0)
            {
                ee_save_addr = CONF_REG_EE1_ADDR;
            }
            else if (j == 1)
            {
                ee_save_addr = CONF_REG_EE2_ADDR;
            }
            else
            {
                ee_save_addr = CONF_REG_EE3_ADDR;
            }

            I2C_EE_BufWrite((uint8_t *)conf_reg, ee_save_addr,
                            (CONF_REG_MAP_NUM)*2);  // save configuration data to eeprom
            for (i = 0; i < 10; i++)
                ;
            I2C_EE_BufRead((uint8_t *)test_reg, ee_save_addr, (CONF_REG_MAP_NUM)*2);
            for (i = 0; i < CONF_REG_MAP_NUM; i++)
            {
                if (conf_reg[i] != test_reg[i])
                {
                    err_cnt++;
                }
            }
            if (err_cnt == 0)
            {
                chk_res = checksum_u16(conf_reg, CONF_REG_MAP_NUM);  // set parameter checksum
                I2C_EE_BufWrite((uint8_t *)&chk_res, ee_save_addr + (CONF_REG_MAP_NUM * 2), 2);

                I2C_EE_BufWrite(&ee_flag, STS_EE_ADDR, 1);  // set eeprom program flag
            }
            else
            {
                req++;
            }
        }
        if (req < 2)
        {
            err_cnt = 0;
        }
        else
        {
            err_cnt = req;
        }
    }
    else
    {
        I2C_EE_BufWrite((uint8_t *)conf_reg, ee_save_addr, (CONF_REG_MAP_NUM)*2);  // save configuration data to eeprom
        for (i = 0; i < 10; i++)
            ;
        I2C_EE_BufRead((uint8_t *)test_reg, ee_save_addr, (CONF_REG_MAP_NUM)*2);
        for (i = 0; i < CONF_REG_MAP_NUM; i++)
        {
            if (conf_reg[i] != test_reg[i])
            {
                err_cnt++;
            }
        }
        if (err_cnt == 0)
        {
            chk_res = checksum_u16(conf_reg, CONF_REG_MAP_NUM);  // set parameter checksum
            I2C_EE_BufWrite((uint8_t *)&chk_res, ee_save_addr + (CONF_REG_MAP_NUM * 2), 2);
            I2C_EE_BufWrite(&ee_flag, STS_EE_ADDR, 1);  // set eeprom program flag
        }
    }

    return err_cnt;
}
uint16_t init_load_default(void)
{
    uint16_t i, ret;
    ret = 1;
    for (i = 0; i < CONF_REG_MAP_NUM; i++)  // initialize global variable with default values
    {
        if (conf_reg_map_inst[i].reg_ptr != NULL)
        {
            *(conf_reg_map_inst[i].reg_ptr) = conf_reg_map_inst[i].dft;
        }
    }
    ret = 1;

    return ret;
}

/**
 * @brief  load system configuration data from eeprom
 * @param  None
 * @retval None
 */

static uint16_t conf_reg_read_ee(uint16_t addr)
{
    uint16_t reg;
    //	    uint16_t err_num = 0;
    uint16_t ee_err, ret;
    reg    = 0;
    ee_err = 0;
    ret    = 0;
    ee_err = eeprom_compare_read(addr, &reg);
    if ((conf_reg_map_inst[addr].reg_ptr != NULL) &&
        ((reg < conf_reg_map_inst[addr].min) || (reg > conf_reg_map_inst[addr].max)))
    {
        *(conf_reg_map_inst[addr].reg_ptr) = (conf_reg_map_inst[addr].dft);
        ret                                = 0;
    }
    else
    {
        *(conf_reg_map_inst[addr].reg_ptr) = reg;
        ret                                = 1;
    }

    if ((ee_err != 0) || (ret == 0))
    {
        ret = 1;
    }
    else
    {
        ret = 0;
    }
    return ret;
}

static uint16_t init_load_user_conf(void)
{
    uint16_t i, sum, sum_reg;
    sum = 0;

    for (i = 0; i < CONF_REG_MAP_NUM; i++)
    {
        sum_reg = sum;
        sum += conf_reg_read_ee(i);

        if (sum != sum_reg)
        {
            // break;
        }
    }

    if (sum == 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

static uint16_t init_load_factory_conf(void)
{
    uint16_t buf_reg[CONF_REG_MAP_NUM + 1];
    uint16_t i;
    uint16_t chk_res;
    uint16_t ee_load_addr;
    ee_load_addr = CONF_REG_FACT_ADDR;

    I2C_EE_BufRead((uint8_t *)buf_reg, ee_load_addr,
                   (CONF_REG_MAP_NUM + 1) * 2);  // read eeprom data & checksum to data buffer
                                                 // //wait for i2c opeartion comletion
    chk_res = checksum_u16(buf_reg, (CONF_REG_MAP_NUM + 1));
    if (chk_res != 0)  // eeprom configuration data checksum fail
    {
        for (i = 0; i < CONF_REG_MAP_NUM; i++)  // initialize global variable with default values
        {
            if (conf_reg_map_inst[i].reg_ptr != NULL)
            {
                *(conf_reg_map_inst[i].reg_ptr) = conf_reg_map_inst[i].dft;
            }
        }
        return 0;
    }
    else
    {
        for (i = 0; i < CONF_REG_MAP_NUM; i++)
        {
            *(conf_reg_map_inst[i].reg_ptr) = buf_reg[i];
        }

        return 1;
    }
}

/**
 * @brief  initialize system status reg data
 * @param  None
 * @retval None
 */
static void init_load_status(void)
{
    //		uint8_t i;
    //		for(i=0;i<STATUS_REG_MAP_NUM;i++)
    //		{
    //				if(status_reg_map_inst[i].reg_ptr != NULL)
    //				{
    //						*(status_reg_map_inst[i].reg_ptr) = status_reg_map_inst[i].dft;
    //				}
    //		}
    return;
}

/**
  * @brief  system configurable variables initialization
  * @param  None
  * @retval
            1:	load default data
            2:	load eeprom data
  */
uint16_t sys_global_var_init(void)
{
    uint16_t ret;

    init_state_em em_init_state;
    osDelay(100);
    init_load_status();
    // I2C_EE_BufWrite(&test,STS_EE_ADDR,1);
    em_init_state = get_ee_status();  // get eeprom init status
    switch (em_init_state)
    {
        case (INIT_LOAD_USR):  // load usr1 data
        {
            ret = init_load_user_conf();
            if (ret == 1)
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] |= 0x01;  //初始化成功
            }
            else
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] &= 0xFFFE;  //初始化失败
            }
            break;
        }
        case (INIT_LOAD_FACT):  // load factory data
        {
            ret = init_load_factory_conf();
            if (ret == 1)
            {
                save_conf_reg(0);
                set_load_flag(0);
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] |= 0x02;
            }
            else
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] &= 0xFFFE;
            }
            break;
        }
        case (INIT_LOAD_DEBUT):  // resotre default configuration data, include reset password to default values
        {
            ret = init_load_default();
            if (ret == 1)
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] |= 0x04;
                save_conf_reg(0);
                save_conf_reg(1);
                set_load_flag(0);
            }
            else
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] &= 0xFFFE;
            }
            break;
        }
        default:  // resotre default configuration data, include reset password to default values
        {
            ret = init_load_default();
            if (ret == 1)
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] |= 0x08;
            }
            else
            {
                g_sVariable.status.u16Status_remap[GEN_STS_REG_NO] &= 0xFFFE;
            }
            break;
        }
    }
    //测试模式和诊断模式复位。
    g_sVariable.gPara.u16Manual_Test_En = 0;
    g_sVariable.gPara.diagnose_mode_en  = 0;
    //外接水
    g_sVariable.gPara.Water.u16ExitWater_Mode = 0;

    return ret;
}

static int16_t eeprom_singel_write(uint16_t base_addr, uint16_t reg_offset_addr, uint16_t wr_data, uint16_t rd_data)
{
    int16_t err_no;
    uint16_t wr_data_buf;
    uint16_t cs_data, ee_rd_cheksum;
    wr_data_buf = wr_data;

    err_no = I2C_EE_BufRead((uint8_t *)&cs_data, base_addr + (CONF_REG_MAP_NUM << 1), 2);
    //计算check_sum
    ee_rd_cheksum = cs_data ^ rd_data ^ wr_data;
    // 写寄存器
    err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf, base_addr + (reg_offset_addr << 1), 2);
    // 写校验
    err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum, base_addr + (CONF_REG_MAP_NUM << 1), 2);

    return err_no;
}

int16_t eeprom_compare_read(uint16_t reg_offset_addr, uint16_t *rd_data)
{
    uint16_t rd_buf0;
    uint16_t rd_buf1;
    uint16_t rd_buf2;
    int16_t ret;
    uint16_t Tempbuf[3] = {0};
    Tempbuf[0]          = Tempbuf[1];
    Tempbuf[0] =
        I2C_EE_BufRead((uint8_t *)&rd_buf0, CONF_REG_EE1_ADDR + (reg_offset_addr << 1), 2);  //从用户区的三处读数据
    Tempbuf[1] = I2C_EE_BufRead((uint8_t *)&rd_buf1, CONF_REG_EE2_ADDR + (reg_offset_addr << 1), 2);
    Tempbuf[2] = I2C_EE_BufRead((uint8_t *)&rd_buf2, CONF_REG_EE3_ADDR + (reg_offset_addr << 1), 2);
    //		I2C_EE_BufRead((uint8_t *)&rd_buf0,CONF_REG_EE1_ADDR+(reg_offset_addr<<1),2); //从用户区的三处读数据
    //		I2C_EE_BufRead((uint8_t *)&rd_buf1,CONF_REG_EE2_ADDR+(reg_offset_addr<<1),2);
    //		I2C_EE_BufRead((uint8_t *)&rd_buf2,CONF_REG_EE3_ADDR+(reg_offset_addr<<1),2);
    // normal situation
    if ((rd_buf0 == rd_buf1) && (rd_buf2 == rd_buf1))
    {
        if (rd_buf0)
            *rd_data = rd_buf0;
        ret = 0;
    }
    else if ((rd_buf0 == rd_buf1) || (rd_buf0 == rd_buf2) || (rd_buf1 == rd_buf2))
    {
        *rd_data = rd_buf0;
        if (rd_buf0 == rd_buf1)  // buf2!= buf1
        {
            *rd_data = rd_buf0;
            eeprom_singel_write(CONF_REG_EE3_ADDR, reg_offset_addr, rd_buf0, rd_buf2);
        }
        else if (rd_buf0 == rd_buf2)  // buf2 = buf0, buf1错
        {
            *rd_data = rd_buf2;
            eeprom_singel_write(CONF_REG_EE2_ADDR, reg_offset_addr, rd_buf2, rd_buf1);
        }
        else  //(rd_buf1 == rd_buf2)
        {
            *rd_data = rd_buf1;
            eeprom_singel_write(CONF_REG_EE1_ADDR, reg_offset_addr, rd_buf1, rd_buf0);
        }
        ret = 0;
    }
    else  //三个都错误
    {
        *rd_data = ABNORMAL_VALUE;
        ret      = 1;
    }
    return (ret);
}

int16_t eeprom_tripple_write(uint16_t reg_offset_addr, uint16_t wr_data, uint16_t rd_data)
{
    int16_t err_no;
    uint16_t wr_data_buf;
    uint16_t cs_data, ee_rd_cheksum;
    wr_data_buf = wr_data;

    err_no = eeprom_compare_read(CONF_REG_MAP_NUM, &cs_data);
    if (wr_data == rd_data)  //相等，避免重复写入
    {
        return 1;
    }
    else
    {
        if (err_no == 0)
        {
            ee_rd_cheksum = cs_data ^ rd_data ^ wr_data;
        }
        else
        {
            return -1;
        }
    }

    err_no = 0;

    // write data to eeprom
    err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf, CONF_REG_EE1_ADDR + (reg_offset_addr << 1), 2);
    err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf, CONF_REG_EE2_ADDR + (reg_offset_addr << 1), 2);
    err_no += I2C_EE_BufWrite((uint8_t *)&wr_data_buf, CONF_REG_EE3_ADDR + (reg_offset_addr << 1), 2);

    // write checksum data to eeprom
    err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum, CONF_REG_EE1_ADDR + (CONF_REG_MAP_NUM * 2), 2);
    err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum, CONF_REG_EE2_ADDR + (CONF_REG_MAP_NUM * 2), 2);
    err_no += I2C_EE_BufWrite((uint8_t *)&ee_rd_cheksum, CONF_REG_EE3_ADDR + (CONF_REG_MAP_NUM * 2), 2);
    return err_no;
}

/**
 * @brief  write register map with constraints.
 * @param  reg_addr: reg map address.
 * @param  wr_data: write data.
 * @param  permission_flag:
 *   This parameter can be one of the following values:
 *     @arg PERM_PRIVILEGED: write opertion can be performed dispite permission level
 *     @arg PERM_INSPECTION: write operation could only be performed when pass permission check
 * @retval
 *   This parameter can be one of the following values:
 *     @arg 1: write operation success
 *     @arg 0: write operation fail
 */
uint16_t reg_map_write(uint16_t reg_addr, uint16_t *wr_data, uint8_t wr_cnt)
{
    uint16_t i;
    uint16_t err_code = CPAD_ERR_NOERR;
    //		uint16_t ee_wr_data,ee_rd_data;
    if ((reg_addr + wr_cnt) > CONF_REG_MAP_NUM)  // address range check
    {
        err_code = CPAD_ERR_ADDR_OR;
        return err_code;
    }

    for (i = 0; i < wr_cnt; i++)  // writablility check
    {
        //			if(conf_reg_map_inst[reg_addr+i].rw != 1)
        if ((conf_reg_map_inst[reg_addr + i].rw != 1) && (conf_reg_map_inst[reg_addr + i].rw != 2))
        {
            err_code = CPAD_ERR_WR_OR;
            return err_code;
        }
    }

    for (i = 0; i < wr_cnt; i++)  // min_max limit check
    {
        if ((*(wr_data + i) > conf_reg_map_inst[reg_addr + i].max) ||
            (*(wr_data + i) < conf_reg_map_inst[reg_addr + i].min))  // min_max limit check
        {
            err_code = CPAD_ERR_DATA_OR;
            return err_code;
        }

        if (conf_reg_map_inst[reg_addr + i].chk_ptr != NULL)
        {
            if (conf_reg_map_inst[reg_addr + i].chk_ptr(*(wr_data + i)) == 0)
            {
                err_code = CPAD_ERR_CONFLICT_OR;
                return err_code;
            }
        }
    }

    for (i = 0; i < wr_cnt; i++)  // data write
    {
        //				ee_rd_data = *(conf_reg_map_inst[reg_addr+i].reg_ptr);				//buffer legacy reg data
        //				ee_wr_data = *(wr_data+i);															//buffer
        //current write data

        *(conf_reg_map_inst[reg_addr + i].reg_ptr) = *(wr_data + i);  // write data to designated register
        if (conf_reg_map_inst[reg_addr + i].rw == 1)                  //写EEPROM
        {
            g_sVariable.u8ReturnCall[reg_addr] =
                TRUE;  //回调标志
                       //			    eeprom_tripple_write(reg_addr+i,ee_wr_data,ee_rd_data);
        }
    }
    return err_code;
}

int16_t CallBack_map_write(void)
{
    uint16_t i;
    uint16_t ee_wr_data = 0;
    uint16_t ee_rd_data = 0xFFFF;
    uint8_t u8Ret       = 0;

    for (i = 0; i < CONF_REG_MAP_NUM; i++)  // data write
    {
        if (g_sVariable.u8ReturnCall[i] == TRUE)  //
        {
            g_sVariable.u8ReturnCall[i] = FALSE;
            ee_wr_data                  = *(conf_reg_map_inst[i].reg_ptr);
            u8Ret                       = eeprom_tripple_write(i, ee_wr_data, ee_rd_data);
            return u8Ret;
        }
    }
    return u8Ret;
}
