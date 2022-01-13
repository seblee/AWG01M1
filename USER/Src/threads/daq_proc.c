/*********************************************************
  Copyright (C), 2014, Sunrise Group. Co., Ltd.
  File name:      	daq_proc.c
  Author: gongping	Version: 0.7       Date:  2014-12-05
  Description:    	Main entry, system threads initialization
  Others:         	n.a
  Function List:  	water_level_sts_get(void)
                    calc_conductivity(void);
                                        calc_humcurrent(void);
  Variable List:  	n.a
  Revision History:
  Date:           Author:          Modification:
    2014-12-05      gongping         file create
*********************************************************/
#include "cmsis_os.h"
#include "sys_def.h"
#include "adc.h"
#include "flash.h"
#include "global_var.h"
#include <math.h>

static uint8_t water_level_sts_get(void);  // water level get function
static uint16_t calc_conductivity(void);   // water conductivity calculation
static uint16_t calc_humcurrent(void);     // humidifier current calculation

/*********************************************************
  * @name   daq_proc
    * @brief  Sample water quality, humidifier current and water level signals
    * @calls  adc_init()
            calc_conductivity()
            calc_humcurrent()
                        water_level_sts_get()
                        osDelay()
  * @called main()
  * @param  *argument : versatile pointer, not used
  * @retval None
*********************************************************/
void daq_proc(void const *argument)
{
    adc_init();
    while (1)
    {
        g_var_st_inst.status_st_inst.water_quality = calc_conductivity();     // water quality equavalent resistance
        g_var_st_inst.status_st_inst.hum_current   = calc_humcurrent();       // humidification current
        g_var_st_inst.status_st_inst.water_level   = !water_level_sts_get();  // water level
        osDelay(100);
    }
}

// water conductivity calculation
/*********************************************************
 * @name   calc_conductivity
 * @brief  caculate water conductivity
 * @calls  None
 * @called daq_proc()
 * @param  None
 * @retval water conductivty equavalent resistance
 *********************************************************/
static uint16_t calc_conductivity(void)
{
    int32_t res;
    static uint16_t cond_vi_reg[8];
    static uint8_t i = 0;
    uint8_t j;
    res = 0;
    if (g_var_st_inst.status_st_inst.cond_vi == 0)  // if cond_vi ==0, means calibration needed
    {
        res = 0xffff;
    }
    else
    {
        res = RES_P * g_var_st_inst.status_st_inst.cond_vi / (g_var_st_inst.driver_st_inst.adc_conv_data[0]) -
              (2 * RES_P + CAP_C);  // water relative resistance calculation
    }

    cond_vi_reg[i % 8] = (uint16_t)res;
    i++;

    res = 0;
    for (j = 0; j < 8; j++)
    {
        res += cond_vi_reg[j];
    }
    res = res >> 3;
    if ((res < 0) || (res > 2048))  // data out of range ,return invalid value;
        return 0xffff;
    else
        return (uint16_t)res;
}

/*********************************************************
 * @name   calc_humcurrent
 * @brief  caculate humidifier current
 * @calls  None
 * @called daq_proc()
 * @param  None
 * @retval humidifier current
 *********************************************************/
static uint16_t calc_humcurrent(void)
{
    static uint16_t hum_c_reg[8];
    static uint8_t i = 0;
    uint8_t j;
    int32_t res;
    res              = 0;
    res              = g_var_st_inst.driver_st_inst.adc_conv_data[1] - g_var_st_inst.status_st_inst.hc_voff;
    hum_c_reg[i % 8] = (uint16_t)res;
    i++;
    res = 0;
    for (j = 0; j < 8; j++)
    {
        res += hum_c_reg[j];
    }
    res = res >> 3;
    if ((res < 0) || (res > 2048))  // data out of range ,return invalid value;
        return 0xffff;
    else
        return (uint16_t)res;
}

/*********************************************************
  * @name   water_level_sts_get
    * @brief  water level detection GPIO configuration
    * @calls  GPIO_ReadInputDataBit
  * @called None
  * @param  None
  * @retval The return value can be:
                        @arg 1: water level high
            @arg 0: water level low
*********************************************************/
static uint8_t water_level_sts_get(void)
{
    return GPIO_ReadInputDataBit(GPIOA, GPIO_PIN_4);
}
