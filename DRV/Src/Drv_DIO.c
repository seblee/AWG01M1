/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //DIO相关驱动函数
  Version:         //V1.0
  Function List:   //DIO_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/15    1.0     build this moudle
***********************************************************/

#include "stm32g0xx_hal.h"
#include "macro.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "Drv_DIO.h"
#include "sys_conf.h"
// local variable definition
static dio_dev_st dio_dev_inst;

#define Pin_Map_In DI_MAX_CNT
//数字输入Pin_Map
const pin_map_st in_pin_map_inst[Pin_Map_In] = {
    {GPIO_PIN_6, GPIOF},   // DI1 PB2
    {GPIO_PIN_13, GPIOA},  // DI2 PB10
    {GPIO_PIN_11, GPIOA},  // DI3 PC3
    {GPIO_PIN_2, GPIOB},   // DI4 PC12
};

#define Pin_Map_Out 24
//数字输出Pin_Map
const pin_map_st out_pin_map_inst[Pin_Map_Out] = {
    {GPIO_PIN_12, GPIOA},  // DO1  PA12
    {GPIO_PIN_9, GPIOD},   // DO2  PD9
    {GPIO_PIN_8, GPIOD},   // DO3  PD8
    {GPIO_PIN_9, GPIOA},   // DO4  PA9
    {GPIO_PIN_8, GPIOA},   // DO5  PA8
    {GPIO_PIN_12, GPIOB},  // DO6  PB12
    {GPIO_PIN_11, GPIOB},  // DO7  PB11
    {GPIO_PIN_9, GPIOB},   // DO8  PB9
    {GPIO_PIN_5, GPIOB},   // DO9  PB5
    {GPIO_PIN_4, GPIOB},   // DO10  PB4
    {GPIO_PIN_3, GPIOB},   // DO11  PB3
    {GPIO_PIN_4, GPIOD},   // DO12  PD4
    {GPIO_PIN_3, GPIOD},   // DO13  PD3
    {GPIO_PIN_2, GPIOD},   // DO14  PD2
    {GPIO_PIN_1, GPIOD},   // DO15  PD1
    {GPIO_PIN_0, GPIOD},   // DO16  PD0
    {GPIO_PIN_15, GPIOA},  // DO17  PA15
    {GPIO_PIN_6, GPIOC},   // DO18  PC6
    {GPIO_PIN_7, GPIOC},   // DO19  PC7
    {GPIO_PIN_8, GPIOC},   // DO20  PC8
    {GPIO_PIN_9, GPIOC},   // DO21  PC9
    {GPIO_PIN_2, GPIOC},   // DO22  PC2
    {GPIO_PIN_1, GPIOC},   // DO23  PC1
    {GPIO_PIN_0, GPIOC},   // DO24  PC0
};

/**
 * @brief  digital IOs GPIO initialization
 * @param  none
 * @retval none
 */
//数字输入输出初始化函数
void Drv_DIO_Init(void)
{
    uint16_t i;

    /* GPIOA clock enable */

    //数字输入PIN初始化

    //数字输出PIN初始化

    //复位
    for (i = 1; i <= Pin_Map_Out; i++)
    {
        do_set(i, GPIO_PIN_RESET);
    }

#ifdef RSTORE
/**
 * PC13 init input
 */
#endif
}

static uint16_t Di_read(void)
{
    uint16_t read_bitmap;
    uint16_t i;
    read_bitmap = 0;
    for (i = 0; i < DI_MAX_CNT; i++)
    {
        read_bitmap |=
            HAL_GPIO_ReadPin(in_pin_map_inst[DI_MAX_CNT - 1 - i].pin_base, in_pin_map_inst[DI_MAX_CNT - 1 - i].pin_id);
        if (i < (DI_MAX_CNT - 1))
        {
            read_bitmap = read_bitmap << 1;
        }
    }
    return read_bitmap;
}

/**
 * @brief  digital input sample interval timeout callback function, calls di_read() each time to update di buffer queue
 * @param  none
 * @retval none
 */
//数字输入定时器回调函数，对数字输入电平进行采样后放入缓冲队列；
void Sync_Di_timeout(void)
{
    static uint16_t u16Cnt;
    uint16_t pBuf;

    if (u16Cnt >= DI_BUF_DEPTH)
    {
        u16Cnt = u16Cnt % DI_BUF_DEPTH;
    }
    pBuf                               = Di_read();  //数字输入状态
    dio_dev_inst.din.reg_array[u16Cnt] = pBuf;
    u16Cnt++;
    return;
}

/**
 * @brief  digital io stucture initialization
 * @param  none
 * @retval none
 */
//数字输入初始化函数?
void DIO_reg_init(void)
{
    uint16_t i;

    dio_dev_inst.din.bitmap = 0;
    for (i = 0; i < DI_BUF_DEPTH; i++)
    {
        dio_dev_inst.din.reg_array[i] = 0;
    }
}

/**
 * @brief  digital input result caculation
 * @param  none
 * @retval none
 */
//数字输入OOK解调
void DI_reg_update(void)
{
    uint16_t di_data, i, j;
    uint16_t di_reg[DI_MAX_CNT];

    di_data = 0;

    for (i = 0; i < DI_MAX_CNT; i++)
    {
        di_reg[i] = 0;
    }

    for (i = 0; i < DI_MAX_CNT; i++)  // outer loop caculate each channels di data
    {
        for (j = 0; j < DI_BUF_DEPTH; j++)  // inner loop caculate sum of one channel di data
        {
            di_reg[i] += (dio_dev_inst.din.reg_array[j] >> i) & (0x0001);
        }
    }
    for (i = 0; i < DI_MAX_CNT; i++)
    {
        {
            if (di_reg[i] > (DI_BUF_DEPTH - (DI_BUF_DEPTH >> 3)))
            {
                di_data &= ~(0x0001 << i);
            }
            else
            {
                di_data |= (0x0001 << i);
            }
        }
    }
    // rt_kprintf("di_reg [0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d\n", di_reg[0], di_reg[1], di_reg[2], di_reg[3], di_reg[4]);
    dio_dev_inst.din.bitmap = di_data;
    return;
}

/**
 * @brief  update global variable g_din_inst and g_ain_inst according to di and ai inputs
 * @param  none
 * @retval none
 **/
void DI_sts_update(void)
{
    uint16_t din_mask_bitmap;
    uint16_t din_bitmap_polarity;
    uint16_t u16Din_bitmap;

    // mask报警掩码
    din_mask_bitmap     = g_sVariable.gPara.dev_mask.din;
    din_bitmap_polarity = g_sVariable.gPara.dev_mask.din_bitmap_polarity;
    //		din_mask_bitmap=DI_MASK;
    //		din_bitmap_polarity=DI_POLARITY;

    dio_dev_inst.din.bitmap = (~(dio_dev_inst.din.bitmap ^ din_bitmap_polarity));

    // 数字输入掩码
    u16Din_bitmap = din_mask_bitmap & dio_dev_inst.din.bitmap;

    g_sVariable.status.u16DI_Bitmap[0] = u16Din_bitmap;

    //		rt_kprintf("u16DI0 = %X,gds_sys_ptr->status.din_bitmap[0] = %X,u16DI0_P = %X,,din_bitmap_polarity =
    //%X\n",u16DI0,gds_sys_ptr->status.din_bitmap[0],u16DI0_P,din_bitmap_polarity[0]);
    return;
}

//恢复原始参数
uint8_t GetSEL(void)
{
    uint8_t u8Ret;

    u8Ret = 0x00;

    if (!RST_READ)
    {
        u8Ret |= 0x02;
    }
    else
    {
        u8Ret &= ~0x02;
    }
    // g_sVariable.u16Test = u8Ret;
    // if (!SLE1_READ)
    // {
    //     u8Ret |= 0x01;
    // }
    // else
    // {
    //     u8Ret &= ~0x01;
    // }
    return u8Ret;
}

//数字输出控制函数；
uint16_t do_set(int16_t pin_id, GPIO_PinState value)
{
    if ((pin_id <= (Pin_Map_Out)) && (pin_id > 0))
    {
        HAL_GPIO_WritePin(out_pin_map_inst[pin_id - 1].pin_base, out_pin_map_inst[pin_id - 1].pin_id, value);
        return 1;
    }
    else
    {
        return 0;
    }
}

// //置位所有数字输出
// static void do_set_all(void)
// {
//     uint16_t i;
//     for (i = 1; i < Pin_Map_Out - 1; i++)
//     {
//         do_set(i, Bit_SET);
//     }
// }

// //复位所有数字输出
// static void do_reset_all(void)
// {
//     uint16_t i;
//     for (i = 1; i < Pin_Map_Out - 1; i++)
//     {
//         do_set(i, Bit_RESET);
//     }
// }

/********************************************************************/
/**
 * @brief 	system output arbitration
 * @param  none
 * @retval final output bitmap
 */
//数字输出仲裁函数
static uint32_t oc_arbitration(void)
{
    extern local_reg_st l_sys;
    uint16_t cat_bitmap;
    uint16_t sys_bitmap;
    uint16_t target_bitmap;
    uint16_t final_bitmap;
    uint8_t u8i;
    uint32_t u32Bit_Final;

    for (u8i = 0; u8i <= 1; u8i++)
    {
        cat_bitmap = l_sys.bitmap[u8i][BITMAP_REQ];

        if ((g_sVariable.gPara.diagnose_mode_en == 0) &&
            (g_sVariable.gPara.u16Manual_Test_En ==
             0))  // if diagnose enable, output manual, else out put concatenated bitmap
        {
            target_bitmap                    = cat_bitmap;
            l_sys.bitmap[u8i][BITMAP_MANUAL] = l_sys.bitmap[u8i][BITMAP_FINAL];
        }
        else
        {
            target_bitmap = l_sys.bitmap[u8i][BITMAP_MANUAL];
        }

        sys_bitmap   = target_bitmap;  // sys_out_bitmap output
        final_bitmap = (g_sVariable.gPara.u16Manual_Test_En == 0)
                           ? sys_bitmap
                           : l_sys.bitmap[u8i][BITMAP_MANUAL];  // final bitmap selection, if test mode enable, output
                                                                // manual, otherwise sys_bitmap

        l_sys.bitmap[u8i][BITMAP_FINAL] = final_bitmap & g_sVariable.gPara.dev_mask.dout[u8i];
        //		rt_kprintf("target_bitmap = %X,bitmap_mask_reg = %X,sys_bitmap = %X,final_bitmap = %X,l_sys =
        //%X\n",target_bitmap,bitmap_mask_reg,sys_bitmap,final_bitmap,l_sys.bitmap[BITMAP_FINAL]); 				return
        // l_sys.bitmap[0][BITMAP_FINAL];
    }

    u32Bit_Final = l_sys.bitmap[0][BITMAP_FINAL] | ((uint32_t)l_sys.bitmap[1][BITMAP_FINAL] << 16);  //输出
    return u32Bit_Final;
}

void dio_set_do(uint16_t channel_id, GPIO_PinState data)
{
    do_set(channel_id, data);
}

/**
 * @brief 	output control module dout and system status update
 * @param  none
 * @retval none
 */
//数字输出执行函数
static void oc_do_update(void)
{
    extern local_reg_st l_sys;

    uint32_t xor_bitmap, new_bitmap;
    uint16_t i;
    uint16_t u16new_bitmap_H;
    uint16_t u16new_bitmap_L;

    u16new_bitmap_L = l_sys.bitmap[0][BITMAP_FINAL];
    u16new_bitmap_H = l_sys.bitmap[1][BITMAP_FINAL];
    // rt_kprintf("new_bitmap=%x,old_bitmap=%x,u32DO[0]=%x,u32DO[1]=%x\n", new_bitmap, old_bitmap, u32DO[0], u32DO[1]);
    xor_bitmap = u16new_bitmap_L ^ g_sVariable.status.u16DO_Bitmap[0] |
                 ((u16new_bitmap_H ^ g_sVariable.status.u16DO_Bitmap[1]) << 16);

    g_sVariable.status.REQ_TEST[7] = u16new_bitmap_L;
    //		g_sVariable.status.REQ_TEST[8]=u16new_bitmap_H;
    new_bitmap = u16new_bitmap_L | (u16new_bitmap_H << 16);
    if (xor_bitmap != 0)  // if output bitmap changed
    {
        for (i = 0; i < 32; i++)
        {
            if (((xor_bitmap >> i) & 1) != 0)  // do status change
            {
                if (((new_bitmap >> i) & 1) != 0)
                {
                    dio_set_do(i + 1, GPIO_PIN_SET);
                }
                else
                {
                    dio_set_do(i + 1, GPIO_PIN_RESET);
                }
            }
            else  // do status no change, continue for loop
            {
                continue;
            }
        }
        g_sVariable.status.u16DO_Bitmap[0] = u16new_bitmap_L;  // update system dout bitmap
        g_sVariable.status.u16DO_Bitmap[1] = u16new_bitmap_H;  // update system dout bitmap
    }
}

/**
 * @brief 	update system output reffering to local bitmaps
 * @param  none
 * @retval none
 */
void oc_update(void)
{
    //数字输出仲裁判决
    oc_arbitration();
    //数字输出执行
    oc_do_update();
    //    //模拟输出仲裁
    //		oc_ao_arbitration();
    //    //模拟输出执行
    //		oc_ao_update();
}
