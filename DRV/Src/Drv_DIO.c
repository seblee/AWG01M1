/************************************************************
  Copyright (C), 1988-1999, Sunrise Tech. Co., Ltd.
  FileName: Drv_IIC.c
  Author:        Version :          Date:
  Description:     //DIO�����������
  Version:         //V1.0
  Function List:   //DIO_Init
    1. -------
  History:         //
      <author>  <time>   <version >   <desc>
      xdp       14/12/15    1.0     build this moudle
***********************************************************/

#include "stm32g0xx_hal.h"
#include "stm32f0xx_misc.h"
#include "macro.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "Drv_DIO.h"
#include "Drv_PWM.h"
#include "sys_conf.h"
// local variable definition
static dio_dev_st dio_dev_inst;

#define Pin_Map_In DI_MAX_CNT
const pin_map_st in_pin_map_inst[Pin_Map_In] =  //��������Pin_Map
    {
        {GPIO_PIN_6, GPIOF},   // DI1
        {GPIO_PIN_13, GPIOA},  // DI2
        {GPIO_PIN_11, GPIOA},  // DI3
        {GPIO_PIN_2, GPIOB},   // DI4
};

#define Pin_Map_Out 7
const pin_map_st out_pin_map_inst[Pin_Map_Out] =  //�������Pin_Map
    {
        {GPIO_PIN_10, GPIOA},  // DO1
        {GPIO_PIN_9, GPIOA},   // DO2
        {GPIO_PIN_8, GPIOA},   // DO3
        {GPIO_PIN_15, GPIOB},  // DO4
        {GPIO_PIN_14, GPIOB},  // DO5
        {GPIO_PIN_13, GPIOB},  // DO6
        {GPIO_PIN_12, GPIOB},  // DO7
};

// DO��ӳ���74HC595D���˳�򽻻�
const Bit_remap_st DO_remap_table[] = {
    {0, 4},    //
    {1, 3},    //
    {2, 2},    //
    {3, 1},    //
    {4, 0},    //
    {5, 15},   //
    {6, 14},   //
    {7, 13},   //
    {8, 12},   //
    {9, 11},   //
    {10, 7},   //
    {11, 6},   //
    {12, 5},   //
    {13, 10},  //
    {14, 9},   //
    {15, 8},   //
};

//��ӳ��˿ڣ�����M1��
uint16_t Sts_Remap(uint16_t u16IN_Bit, uint8_t Rep_Type, uint8_t Rep_Dir)
{
    uint16_t u16Rep_bit;
    uint8_t i, u8Length;

    u16Rep_bit = 0x00;
    switch (Rep_Type)
    {
            //			case Rep_DI:
            //					u8Length=sizeof(DI_remap_table)/2;
            //					for(i=0;i<u8Length;i++)
            //					{
            //						if(Rep_Dir)
            //						{
            //							if((u16IN_Bit >> DI_remap_table[i].u8M1_Bit) & 0x0001)
            //							{
            //								u16Rep_bit |= (0x0001<<DI_remap_table[i].u8Mx_Bit);
            //							}
            //						}
            //						else
            //						{
            //							if((u16IN_Bit >> DI_remap_table[i].u8Mx_Bit) & 0x0001)
            //							{
            //								u16Rep_bit |= (0x0001<<DI_remap_table[i].u8M1_Bit);
            //							}
            //						}
            //					}
            ////			rt_kprintf("u16IN_Bit = %X,u8Length = %X,u16Rep_bit = %X\n",u16IN_Bit,u8Length,u16Rep_bit);
            //					break;
        case Rep_DO:
            u8Length = sizeof(DO_remap_table) / 2;
            for (i = 0; i < u8Length; i++)
            {
                if (Rep_Dir)
                {
                    if ((u16IN_Bit >> DO_remap_table[i].u8M1_Bit) & 0x0001)
                    {
                        u16Rep_bit |= (0x0001 << DO_remap_table[i].u8Mx_Bit);
                    }
                }
                else
                {
                    if ((u16IN_Bit >> DO_remap_table[i].u8Mx_Bit) & 0x0001)
                    {
                        u16Rep_bit |= (0x0001 << DO_remap_table[i].u8M1_Bit);
                    }
                }
            }
            //			rt_kprintf("u16IN_Bit = %X,u8Length = %X,u16Rep_bit = %X\n",u16IN_Bit,u8Length,u16Rep_bit);
            break;
            //			case Rep_AI:
            //					u8Length=sizeof(AI_remap_table)/2;
            //					for(i=0;i<u8Length;i++)
            //					{
            //						if(Rep_Dir)
            //						{
            //							if(u16IN_Bit == AI_remap_table[i].u8M1_Bit)
            //							{
            //								u16Rep_bit = AI_remap_table[i].u8Mx_Bit;
            //							}
            //						}
            //						else
            //						{
            //							if(u16IN_Bit == AI_remap_table[i].u8Mx_Bit)
            //							{
            //								u16Rep_bit = AI_remap_table[i].u8M1_Bit;
            //							}
            //						}
            //					}
            //					break;
            //			case Rep_AO:
            //					u8Length=sizeof(AO_remap_table)/2;
            //					for(i=0;i<u8Length;i++)
            //					{
            //						if(Rep_Dir)
            //						{
            //							if(u16IN_Bit == AO_remap_table[i].u8M1_Bit)
            //							{
            //								u16Rep_bit = AO_remap_table[i].u8Mx_Bit;
            //							}
            //						}
            //						else
            //						{
            //							if(u16IN_Bit == AO_remap_table[i].u8Mx_Bit)
            //							{
            //								u16Rep_bit = AO_remap_table[i].u8M1_Bit;
            //							}
            //						}
            //					}
            //					break;
        default:
            break;
    }
    return u16Rep_bit;
}

/**
 * @brief  digital IOs GPIO initialization
 * @param  none
 * @retval none
 */
//�������������ʼ������
void Drv_DIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint16_t i;

    /* GPIOA clock enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);

    //��������PIN��ʼ��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;

    for (i = 0; i < Pin_Map_In; i++)
    {
        GPIO_InitStructure.GPIO_Pin = in_pin_map_inst[i].pin_id;
        GPIO_Init(in_pin_map_inst[i].pin_base, &GPIO_InitStructure);
    }

    //�������PIN��ʼ��
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    for (i = 0; i < Pin_Map_Out; i++)
    {
        GPIO_InitStructure.GPIO_Pin = out_pin_map_inst[i].pin_id;
        GPIO_Init(out_pin_map_inst[i].pin_base, &GPIO_InitStructure);
    }
    //��λ
    for (i = 1; i <= Pin_Map_Out; i++)
    {
        do_set(i, Bit_RESET);
    }

    HC595_GPIO_Config();
#ifdef RSTORE
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = RST_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;  // 10M
    GPIO_Init(RST_GPIO, &GPIO_InitStructure);
#endif

    return;
}

static uint16_t Di_read(void)
{
    uint16_t read_bitmap;
    uint16_t i;
    read_bitmap = 0;
    for (i = 0; i < DI_MAX_CNT; i++)
    {
        read_bitmap |= GPIO_ReadInputDataBit(in_pin_map_inst[DI_MAX_CNT - 1 - i].pin_base,
                                             in_pin_map_inst[DI_MAX_CNT - 1 - i].pin_id);
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
//�������붨ʱ���ص������������������ƽ���в�������뻺����У�
void Sync_Di_timeout(void)
{
    static uint16_t u16Cnt;
    uint16_t pBuf;

    if (u16Cnt >= DI_BUF_DEPTH)
    {
        u16Cnt = u16Cnt % DI_BUF_DEPTH;
    }
    pBuf                               = Di_read();  //��������״̬
    dio_dev_inst.din.reg_array[u16Cnt] = pBuf;
    u16Cnt++;
    return;
}

/**
 * @brief  digital io stucture initialization
 * @param  none
 * @retval none
 */
//���������ʼ������?
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
//��������OOK���
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
    //				rt_kprintf("di_reg[0] = %d,di_reg[1] = %d,di_reg[2] = %d,di_reg[3] = %d,di_reg[4] =
    //%d\n",di_reg[0],di_reg[1],di_reg[2],di_reg[3],di_reg[4]);
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

    // mask��������
    din_mask_bitmap     = g_sVariable.gPara.dev_mask.din;
    din_bitmap_polarity = g_sVariable.gPara.dev_mask.din_bitmap_polarity;
    //		din_mask_bitmap=DI_MASK;
    //		din_bitmap_polarity=DI_POLARITY;

    dio_dev_inst.din.bitmap = (~(dio_dev_inst.din.bitmap ^ din_bitmap_polarity));

    // ������������
    u16Din_bitmap = din_mask_bitmap & dio_dev_inst.din.bitmap;

    g_sVariable.status.u16DI_Bitmap[0] = u16Din_bitmap;

    //		rt_kprintf("u16DI0 = %X,gds_sys_ptr->status.din_bitmap[0] = %X,u16DI0_P = %X,,din_bitmap_polarity =
    //%X\n",u16DI0,gds_sys_ptr->status.din_bitmap[0],u16DI0_P,din_bitmap_polarity[0]);
    return;
}

//�ָ�ԭʼ����
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
    //		g_sVariable.u16Test=u8Ret;
    //    if (!SLE1_READ)
    //    {
    //        u8Ret |= 0x01;
    //    }
    //    else
    //    {
    //        u8Ret &= ~0x01;
    //    }
    return u8Ret;
}

//����������ƺ�����
uint16_t do_set(int16_t pin_id, BitAction value)
{
    if ((pin_id <= (Pin_Map_Out)) && (pin_id > 0))
    {
        GPIO_WriteBit(out_pin_map_inst[pin_id - 1].pin_base, out_pin_map_inst[pin_id - 1].pin_id, value);
        return 1;
    }
    else
    {
        return 0;
    }
}

////��λ�����������
// static void do_set_all(void)
//{
//		uint16_t i;
//		for(i=1;i<Pin_Map_Out-1;i++)
//		{
//				do_set(i,Bit_SET);
//		}
// }

////��λ�����������
// static void do_reset_all(void)
//{
//		uint16_t i;
//		for(i=1;i<Pin_Map_Out-1;i++)
//		{
//				do_set(i,Bit_RESET);
//		}
// }

/********  74HC595 GPIO ���� *************************/
void HC595_GPIO_Config(void)
{
    u8 u8Data[2] = {0};

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(OE_GPIO_CLK | SHCP_GPIO_CLK | STCP_GPIO_CLK | DS_GPIO_CLK, ENABLE);

    RCC_LSEConfig(RCC_LSE_OFF); /* �ر��ⲿ����ʱ��,PC14+PC15����������ͨIO*/

    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    //    GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL ;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;

    GPIO_InitStructure.GPIO_Pin = OE_GPIO_PIN;
    GPIO_Init(OE_GPIO_PORT, &GPIO_InitStructure);  // ��ʼ�� SHCP ����

    GPIO_InitStructure.GPIO_Pin = SHCP_GPIO_PIN;
    GPIO_Init(SHCP_GPIO_PORT, &GPIO_InitStructure);  // ��ʼ�� SHCP ����

    GPIO_InitStructure.GPIO_Pin = STCP_GPIO_PIN;
    GPIO_Init(STCP_GPIO_PORT, &GPIO_InitStructure);  // ��ʼ�� STCP ����

    GPIO_InitStructure.GPIO_Pin = DS_GPIO_PIN;
    GPIO_Init(DS_GPIO_PORT, &GPIO_InitStructure);  // ��ʼ�� DS   ����

    GPIO_ResetBits(OE_GPIO_PORT, OE_GPIO_PIN);      //�͵�ƽ��ʹ��HC595
    GPIO_ResetBits(SHCP_GPIO_PORT, SHCP_GPIO_PIN);  // ���ų�ʼ״̬Ϊ�ߣ����ڲ���������
    GPIO_ResetBits(STCP_GPIO_PORT, STCP_GPIO_PIN);
    GPIO_ResetBits(DS_GPIO_PORT, DS_GPIO_PIN);

    HC595_Send_Multi_Byte(&u8Data[0], 2);
    return;
}
/***
 *74HC595 ����һ���ֽ�
 *����74HC595��DS���ŷ���һ���ֽ�
 */
void HC595_Send_Byte(u8 byte)
{
    u8 i;

    for (i = 0; i < 8; i++)  //һ���ֽ�8λ������8�Σ�һ��һλ��ѭ��8�Σ��պ�����8λ
    {
        /****  ����1�������ݴ���DS����    ****/
        if (byte & 0x80)        //�ȴ����λ��ͨ���������жϵڰ��Ƿ�Ϊ1
            HC595_Data_High();  //����ڰ�λ��1������ 595 DS���ӵ���������ߵ�ƽ
        else                    //��������͵�ƽ
            HC595_Data_Low();

        /*** ����2��SHCPÿ����һ�������أ���ǰ��bit�ͱ�������λ�Ĵ��� ***/
        HC595_SHCP_Low();   // SHCP����
        Delay_us(10);       // �ʵ���ʱ
        HC595_SHCP_High();  // SHCP���ߣ� SHCP����������
        Delay_us(10);

        byte <<= 1;  // ����һλ������λ����λ�ƣ�ͨ��	if (byte & 0x80)�жϵ�λ�Ƿ�Ϊ1
    }
}

/**
 *74HC595������� ʹ��
 **/
void HC595_CS(void)
{
    /**  ����3��STCP����һ�������أ���λ�Ĵ�������������洢�Ĵ���  **/
    HC595_STCP_Low();   // ��STCP����
    Delay_us(10);       // �ʵ���ʱ
    HC595_STCP_High();  // �ٽ�STCP���ߣ�STCP���ɲ���һ��������
    Delay_us(10);
}

/**
 *���Ͷ���ֽ�
 *���ڼ���ʱ���ݵķ���
 *����N��������Ҫ����N���ֽڿ���HC595
 ***/
void HC595_Send_Multi_Byte(u8 *data, u8 len)
{
    u8 i;
    for (i = 0; i < len; i++)  // len ���ֽ�
    {
        HC595_Send_Byte(data[i]);
    }

    HC595_CS();  //�Ȱ������ֽڷ����꣬��ʹ�����
}
/********************************************************************/
/**
 * @brief 	system output arbitration
 * @param  none
 * @retval final output bitmap
 */
//��������ٲú���
static uint32_t oc_arbitration(void)
{
    extern local_reg_st l_sys;
    uint16_t cat_bitmap;
    uint16_t sys_bitmap;
    uint16_t alarm_bitmap, bitmap_mask, bitmap_mask_reg, bitmap_mask_reset;
    uint16_t target_bitmap;
    uint16_t final_bitmap;
    uint8_t u8i;
    uint32_t u32Bit_Final;

    for (u8i = 0; u8i <= 1; u8i++)
    {
        bitmap_mask_reset = 0;
        alarm_bitmap      = l_sys.bitmap[u8i][BITMAP_ALARM];
        bitmap_mask       = l_sys.bitmap[u8i][BITMAP_MASK];

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

        //        bitmap_mask_reg = (g_sys.config.general.alarm_bypass_en == 0) ? bitmap_mask : bitmap_mask_reset;
        //        //bitmap mask selection, if alarm_bypass_en set, output reset bitmap
        //        //		rt_kprintf("alarm_bitmap = %X,bitmap_mask = %X,bitmap_mask_reg =
        //        %X\n",alarm_bitmap,bitmap_mask,bitmap_mask_reg); sys_bitmap = (target_bitmap & ~bitmap_mask_reg) |
        //        (alarm_bitmap & bitmap_mask_reg); //sys_out_bitmap output
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

    u32Bit_Final = l_sys.bitmap[0][BITMAP_FINAL] | ((uint32_t)l_sys.bitmap[1][BITMAP_FINAL] << 16);  //���
    return u32Bit_Final;
}

void dio_set_do(uint16_t channel_id, BitAction data)
{
    do_set(channel_id, data);
}

/**
 * @brief 	output control module dout and system status update
 * @param  none
 * @retval none
 */
//�������ִ�к���
static void oc_do_update(void)
{
    extern local_reg_st l_sys;

    uint16_t xor_bitmap[2];
    uint16_t i;
    uint16_t u16new_bitmap_H;
    uint16_t u16new_bitmap_L;
    U16_8Data u16Lnew_bitmap;
    //		//TEST
    //		{
    //			static uint16_t j=0;
    //			l_sys.bitmap[1][BITMAP_FINAL]=0;
    //			l_sys.bitmap[0][BITMAP_FINAL]=0;

    //			if(j<16)
    //			{
    //				j++;
    //			}
    //			else
    //			{
    //				j=0;
    //			}
    //			l_sys.bitmap[0][BITMAP_FINAL] |= (0x0001<<j);
    //			l_sys.bitmap[1][BITMAP_FINAL] |= (0x0001<<j);
    //		}

    u16new_bitmap_L = l_sys.bitmap[0][BITMAP_FINAL];
    u16new_bitmap_H = l_sys.bitmap[1][BITMAP_FINAL];
    // rt_kprintf("new_bitmap= %x,old_bitmap= %x,u32DO[0]=%x,u32DO[1]=%x\n", new_bitmap, old_bitmap, u32DO[0],
    // u32DO[1]);
    xor_bitmap[0] = u16new_bitmap_L ^ g_sVariable.status.u16DO_Bitmap[0];
    xor_bitmap[1] = u16new_bitmap_H ^ g_sVariable.status.u16DO_Bitmap[1];

    u16Lnew_bitmap.u16Data = Sts_Remap(u16new_bitmap_L, Rep_DO, 0);

    g_sVariable.status.REQ_TEST[7] = u16new_bitmap_L;
    //		g_sVariable.status.REQ_TEST[8]=u16new_bitmap_H;
    //��λ
    if (xor_bitmap[0] != 0)  // if output bitmap changed
    {
        //				u16Lnew_bitmap.u16Data = ~u16Lnew_bitmap.u16Data;
        HC595_Send_Multi_Byte(&u16Lnew_bitmap.u8Data[0], 2);
        g_sVariable.status.u16DO_Bitmap[0] = u16new_bitmap_L;  // update system dout bitmap
    }
    //��λ
    if (xor_bitmap[1] != 0)  // if output bitmap changed
    {
        for (i = 0; i < Pin_Map_Out; i++)
        {
            if (((xor_bitmap[1] >> i) & 0x0001) != 0)  // do status change
            {
                if (((u16new_bitmap_H >> i) & 0x0001) != 0)
                {
                    dio_set_do(i + 1, Bit_SET);
                }
                else
                {
                    dio_set_do(i + 1, Bit_RESET);
                }
            }
            else  // do status no change, continue for loop
            {
                continue;
            }
        }
        g_sVariable.status.u16DO_Bitmap[1] = u16new_bitmap_H;  // update system dout bitmap
    }
    else  // output bitmap unchange
    {
        ;
    }
}

/**
 * @brief 	update system output reffering to local bitmaps
 * @param  none
 * @retval none
 */
void oc_update(void)
{
    //��������ٲ��о�
    oc_arbitration();
    //�������ִ��
    oc_do_update();
    //    //ģ������ٲ�
    //		oc_ao_arbitration();
    //    //ģ�����ִ��
    //		oc_ao_update();
}
