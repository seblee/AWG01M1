/*
 * FreeModbus Libary: BARE Demo Application
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id: demo.c,v 1.1 2006/08/22 21:35:13 wolti Exp $
 */

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb_cb.h"
#include "i2c_bsp.h"
#include "global.h"
/* ----------------------- Defines ------------------------------------------*/
static uint16_t mbs_read_reg(uint16_t read_addr);
/* ----------------------- Static variables ---------------------------------*/
// static sModbus lgsModbus_inst;
extern conf_reg_map_st conf_reg_map_inst[];

// mb_reg_st lgsModbus_inst;

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    return MB_ENOREG;
}

uint16_t Reg_Map_Write(uint16_t reg_addr, uint16_t *wr_data)
{
    uint16_t err_code;
    err_code = CPAD_ERR_NOERR;

    if ((conf_reg_map_inst[reg_addr].rw != 1) && (conf_reg_map_inst[reg_addr].rw != 2))
    {
        err_code = CPAD_ERR_WR_OR;
        //				rt_kprintf("CPAD_ERR_WR_OR02 failed\n");
        return err_code;
    }

    if ((*(wr_data) > conf_reg_map_inst[reg_addr].max) ||
        (*(wr_data) < conf_reg_map_inst[reg_addr].min))  // min_max limit check
    {
        err_code = CPAD_ERR_DATA_OR;
        //				rt_kprintf("CPAD_ERR_WR_OR03 failed\n");
        return err_code;
    }
    return err_code;
}

/*********************************************************
  * @name   eMBRegHoldingCB
    * @brief  modbus holding register operation call back function
    * @calls  None
  * @called modbus protocal stack
  * @param  None
  * @retval The return value can be:
                        @arg MB_ENOERR:
            @arg MB_ENORES:
*********************************************************/
// eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
//{
////    extern g_var_st g_var_st_inst;
//		eMBErrorCode    eStatus = MB_ENOERR;
//    int             iRegIndex;
//	  uint16_t u16RegAddr = 0;
//	  uint16_t usValue = 0;
//	  uint16_t u16Ret = 0;

//    if( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NUM )
//    {
//        iRegIndex = ( int )( usAddress - REG_HOLDING_START );
//        switch ( eMode )
//        {
//        // Pass current register values to the protocol stack.
//        case MB_REG_READ:
//            while( usNRegs > 0 )
//            {
//                *pucRegBuffer++ = ( UCHAR ) ( lgsModbus_inst.u16RegBuffer[iRegIndex] >> 8 );
//                *pucRegBuffer++ = ( UCHAR ) ( lgsModbus_inst.u16RegBuffer[iRegIndex] & 0xFF );
//                iRegIndex++;
//                usNRegs--;
//            }
//            break;

//        // Update current register values with new values from the protocol stack.
//        case MB_REG_WRITE:
//            while( usNRegs > 0 )
//            {
//								if(lgsModbus_inst.u8RegProperty[iRegIndex] == 0)					//if reg is RO, quit
//process
//								{
//										eStatus = MB_EIO;
//										break;
//								}
////								lgsModbus_inst.u16RegBuffer[iRegIndex] = *pucRegBuffer++ << 8;
////                lgsModbus_inst.u16RegBuffer[iRegIndex] |= *pucRegBuffer++;
////								lgsModbus_inst.u8RegStatus[iRegIndex] = MB_REG_WRITE;								//content change measn new cmd
///request
////
////                iRegIndex++;
////                usNRegs--;
//								u16RegAddr=iRegIndex;

//								usValue = *pucRegBuffer++ << 8;
//                usValue |= *pucRegBuffer++;
//
//								u16Ret=Reg_Map_Write(conf_reg_map_inst[u16RegAddr].id, &usValue);
//								if(u16Ret == CPAD_ERR_NOERR)
//								{
//									lgsModbus_inst.u16RegBuffer[iRegIndex] = usValue;
//									lgsModbus_inst.u8RegStatus[iRegIndex] = MB_REG_WRITE;								//content change measn new cmd
//request
//
//									iRegIndex++;
//									usNRegs--;
//								}
//								else
//								{
//										eStatus = u16Ret;
//										break;
//								}

//            }
//            break;
//        }
//    }
//    else
//    {
//        eStatus = MB_ENOREG;
//    }
//    return eStatus;
//}

uint8_t COM_SINGLE_eMBRegHoldingCB(uint16_t usAddress, uint16_t usValue)
{
    extern local_reg_st l_sys;
    extern conf_reg_map_st conf_reg_map_inst[];
    eMBErrorCode eStatus = MB_ENOERR;
    uint16_t temp        = 0;
    //    uint16_t u16RegAddr = usAddress;
    //    uint16_t u16Value = usValue;

    switch (usAddress)
    {
        case FACTORY_RESET:  //出厂设置
        {
            temp = usValue;
            if (temp == 0x3C)  //恢复原始参数
            {
                //            reset_runtime(0xFF); //清零所有运行时间
                set_load_flag(0x02);
                osDelay(1000);
                NVIC_SystemReset();
                return MB_ENOERR;
            }
            else if (temp == 0x5A)  //恢复出厂设置
            {
                set_load_flag(0x01);
                osDelay(1000);
                NVIC_SystemReset();
                return MB_ENOERR;
            }
            else if (temp == 0x69)  //保存出厂设置
            {
                save_conf_reg(0x01);
                osDelay(1000);
                NVIC_SystemReset();
                return MB_ENOERR;
            }
            else if (temp == 0x2D)  //重启
            {
                osDelay(1000);
                NVIC_SystemReset();
                return MB_ENOERR;
            }
            else if (temp == 0x15)  //清零运行时间
            {
                reset_runtime(0xFF);  //清零所有运行时间
                osDelay(100);
                return MB_ENOERR;
            }
            else
            {
                eStatus = MB_ENORES;
            }
        }
        break;
        case MANUAL_TSET:  //测试模式
        {
            temp = usValue;
            if (temp == TEST_MANUAL_UNABLE)
            {
                osDelay(500);
                NVIC_SystemReset();
                return MB_ENOERR;
            }
            else
            {
                if (reg_map_write(conf_reg_map_inst[usAddress].id, &usValue, 1) == CPAD_ERR_NOERR)
                {
                    //										iRegIndex++;
                    //										usNRegs--;
                    eStatus = MB_ENOERR;
                }
                else
                {
                    eStatus = MB_ENORES;
                    //	 while( usNRegs > 0 )
                }
            }
        }
        break;
        case CLEAR_RT:  //清零部件运行时间
        {
            temp = usValue;
            if (temp)  //清零部件运行时间
            {
                reset_runtime(temp);
                return MB_ENOERR;
            }
            else
            {
                eStatus = MB_ENORES;
            }
        }
        break;
        case CLEAR_ALARM:  //清除告警
        {
            temp = usValue;
            if (temp == 0x5A)  //清零部件运行时间
            {
                clear_alarm(0xFF);
                return MB_ENOERR;
            }
            else
            {
                eStatus = MB_ENORES;
            }
        }
        break;
        default: {
            if (reg_map_write(conf_reg_map_inst[usAddress].id, &usValue, 1) == CPAD_ERR_NOERR)
            {
                //										iRegIndex++;
                //										usNRegs--;
                eStatus = MB_ENOERR;
            }
            else
            {
                eStatus = MB_ENORES;
            }
        }
        break;
    }
    return eStatus;
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    extern conf_reg_map_st conf_reg_map_inst[];
    eMBErrorCode eStatus = MB_ENOERR;
    USHORT iRegIndex;
    uint16_t cmd_value;
    uint16_t u16RegAddr = usAddress;

    // uint16_t err_code;
    usAddress--;  // FreeModbus功能函数中已经加1，为保证与缓冲区首地址一致，故减1
    // if((usAddress >= REG_HOLDING_START)&&(usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ))
    if (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS)
    {
        iRegIndex = (USHORT)(usAddress - REG_HOLDING_START);
        switch (eMode)
        {
            // Pass current register values to the protocol stack.
            case MB_REG_READ:
                while (usNRegs > 0)
                {
#ifdef E2PROM
                    cmd_value       = mbs_read_reg(iRegIndex);
                    *pucRegBuffer++ = (unsigned char)(cmd_value >> 8);
                    *pucRegBuffer++ = (unsigned char)(cmd_value & 0xFF);
#else
                    *pucRegBuffer++ = (UCHAR)(lgsModbus_inst.u16RegBuffer[iRegIndex] >> 8);
                    *pucRegBuffer++ = (UCHAR)(lgsModbus_inst.u16RegBuffer[iRegIndex] & 0xFF);
#endif

                    iRegIndex++;
                    usNRegs--;
                }
                break;

            // Update current register values with new values from the protocol stack.
            case MB_REG_WRITE:
                while (usNRegs > 0)
                {
#ifdef E2PROM
                    //超出可写范围报错判断
                    if ((usAddress + usNRegs) <= (REG_HOLDING_START + CPAD_REG_HOLDING_WRITE_NREGS))
                    {
                        if ((usAddress + usNRegs) >= (REG_HOLDING_START + CONFIG_REG_MAP_OFFSET + 1))
                        {
                            cmd_value = (*pucRegBuffer) << 8;
                            cmd_value += *(pucRegBuffer + 1);
                            //写入保持寄存器中同时跟新到内存和flash保存
                            // 写入寄存器和EEPROM中。
                            //
                            u16RegAddr = iRegIndex - CONFIG_REG_MAP_OFFSET;
                            if (COM_SINGLE_eMBRegHoldingCB(u16RegAddr, cmd_value) == MB_ENOERR)
                            {
                                usNRegs--;
                            }
                            else
                            {
                                eStatus = MB_ENORES;
                                break;  //	 while( usNRegs > 0 )
                            }
                        }
                        else
                        {
                            //                        pusRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                            //                        pusRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                            iRegIndex++;
                            usNRegs--;
                        }
                    }
                    else
                    {
                        eStatus = MB_ENOREG;
                        break;  //  while( usNRegs > 0 )
                    }
#else
                    //								if(lgsModbus_inst.u8RegProperty[iRegIndex] == 0)					//if reg is RO, quit
                    //process
                    //								{
                    //										eStatus = MB_EIO;
                    //										break;
                    //								}
                    //								lgsModbus_inst.u16RegBuffer[iRegIndex] = *pucRegBuffer++ << 8;
                    //                lgsModbus_inst.u16RegBuffer[iRegIndex] |= *pucRegBuffer++;
                    //								lgsModbus_inst.u8RegStatus[iRegIndex] = MB_REG_WRITE;								//content
                    //change measn new cmd request
                    //
                    //                iRegIndex++;
                    //                usNRegs--;

                    cmd_value = *pucRegBuffer++ << 8;
                    cmd_value |= *pucRegBuffer++;

                    err_code = Reg_Map_Write(conf_reg_map_inst[iRegIndex].id, &cmd_value);
                    if (err_code == CPAD_ERR_NOERR)
                    {
                        lgsModbus_inst.u16RegBuffer[iRegIndex] = cmd_value;
                        lgsModbus_inst.u8RegStatus[iRegIndex]  = MB_REG_WRITE;  // content change measn new cmd request
                        if (iRegIndex == FACTORY_RESET)                         //复位默认参数
                        {
                            //										temp =
                            //(uint8_t)(*conf_reg_map_inst[FACTORY_RESET].reg_ptr);
                            if (cmd_value == 0x5A)
                            {
                                __set_PRIMASK(1);
                                SaveSettings(0);
                                Delay(50);
                                NVIC_SystemReset();  //复位
                            }
                        }

                        iRegIndex++;
                        usNRegs--;
                    }
                    else
                    {
                        eStatus = err_code;
                        break;
                    }
#endif
                }
                break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    return MB_ENOREG;
}

/*********************************************************
 * @name   SetWorkMode
 * @brief  switch to factory mode, in which all modbus regs could be accessed
 * @calls  ConfigRegister()
 * @called None
 * @param  None
 * @retval None
 *********************************************************/
// static void SetWorkMode(uint8_t WorkMode)
//{
////    uint16_t i;
//#ifdef  E2PROM

//#else
//		if(WorkMode)//用户模式
//		{
//				lgsModbus_inst.u16RegBuffer[MB_CFG_WorkMode] &= ~0x0001;				//reset status flag bit that indcates factory
//mode 				for(i=0;i<REG_HOLDING_NREGS;i++)
//				{
//						lgsModbus_inst.u8RegProperty[i] = MB_REG_READ;
//				}
////				lgsModbus_inst.u8RegProperty[MB_CFG_ADDR] = MB_REG_WRITE;
////				lgsModbus_inst.u8RegProperty[MB_CFG_BAUDRATE] = MB_REG_WRITE;
////				lgsModbus_inst.u8RegProperty[MB_CFG_CMD] = MB_REG_WRITE;			//active device seting change
///permission /				lgsModbus_inst.u8RegProperty[MB_CFG_FANSTEP] = MB_REG_WRITE; /
///lgsModbus_inst.u8RegProperty[MB_CFG_FANOUT] = MB_REG_WRITE;
//		}
//		else//工厂模式
//		{
//				lgsModbus_inst.u16RegBuffer[MB_CFG_WorkMode] |= 0x0001;			//set status flag bit that indcates factory
//mode 				for(i=0;i<REG_HOLDING_NREGS;i++)
//				{
//						lgsModbus_inst.u8RegProperty[i] = MB_REG_WRITE;
//				}
//		}
//#endif

//}
static uint16_t init_load_default(void)
{
    uint16_t i, ret;
    ret = 1;
    for (i = 0; i < REG_HOLDING_NREGS; i++)  // initialize global variable with default values
    {
        if (conf_reg_map_inst[i].reg_ptr != NULL)
        {
            *(conf_reg_map_inst[i].reg_ptr) = conf_reg_map_inst[i].dft;
        }
    }
    return ret;
}
/*********************************************************
  * @name   MBREGDeflaut
    * @brief  reset to initial factory parameters
    * @calls  ResetDefaultParameter();

  * @called SetWorkMode(1);//用户模式();
  * @param  None
  * @retval None
*********************************************************/
static void MBREGDeflaut(void)
{
//	  uint16_t i;
#ifdef E2PROM
    //		//默认参数初始化
    init_load_default();
#else
    lgsModbus_inst.u8RegStart = REG_HOLDING_START;

    memcpy((u8 *)&lgsModbus_inst.u16RegBuffer[MB_CFG_Num], (u8 *)&g_sVariable.gPara.Num, sizeof(sParameter));  //
    SetWorkMode(1);  //用户模式()
#endif
}

/*********************************************************
  * @name   cmd_mb_stack_restart
    * @brief  restart modbus protocal stack, update device address or/and communication baudrate
    * @calls  MBGetAddrsee(),
            MBGetBaudrate(),
            eMBDisable(),
            eMBInit();
                        eMBEnable();
  * @called ();
  * @param  None
  * @retval None
*********************************************************/
// static eMBErrorCode MBStackRestart(void)
//{
//	  eMBErrorCode eStatus=MB_ENOERR;
//
//		osDelay(50);
//		eStatus = eMBDisable();
//		if(eStatus != MB_ENOERR)
//		{
//				return eStatus;
//		}
//		eStatus = eMBInit( MB_RTU, MBGetAddrsee(), USART0_CH, MBGetBaudrate(), MB_PAR_NONE );
//		if(eStatus != MB_ENOERR)
//		{
//				return eStatus;
//		}
//		eStatus = eMBEnable();
//		return eStatus;
// }

/*********************************************************
 * @name   SaveSettings
 * @brief  save current modbus register data into flashrom
 * @calls  ()
 * @called flash_write()
 * @param  None
 * @retval None
 *********************************************************/
void SaveSettings(uint8_t u8Type)
{
#ifdef E2PROM

#else
    if (u8Type == 1)
    {
        lgsModbus_inst.u8PgFlag = MB_FLASH_WR_FLAG;
    }
    else
    {
        lgsModbus_inst.u8PgFlag = 0;
    }
    Flash_Write(FLASH_USER_START, (uint16_t *)&lgsModbus_inst, (sizeof(sModbus) / 2));
#endif
}

/*********************************************************
  * @name   ResetDefaultParameter
    * @brief  reset system parameters to default values
    * @calls  ConfigRegister()
  * @called MBREGDeflaut(),
                        eMBClose(),
            eMBDisable(),
            eMBInit(),
                        eMBEnable();
  * @param  None
  * @retval None
*********************************************************/
eMBErrorCode ResetDefaultParameter(void)
{
    eMBErrorCode eStatus = MB_ENOERR;

    osDelay(50);
    eStatus = eMBDisable();
    if (eStatus != MB_ENOERR)
    {
        return eStatus;
    }
    eStatus = eMBClose();
    if (eStatus != MB_ENOERR)
    {
        return eStatus;
    }
    MBREGDeflaut();
    eStatus = eMBInit(MB_RTU, MBGetAddrsee(), USART0_CH, MBGetBaudrate(), MB_PAR_NONE);
    if (eStatus != MB_ENOERR)
    {
        return eStatus;
    }
    eStatus = eMBEnable();
    return eStatus;
}

/*********************************************************
  * @name   SystemReset
    * @brief  system software reset
    * @calls  ConfigRegister()
  * @called eMBDisable(),
                        eMBClose(),
                        NVIC_SystemReset();
  * @param  None
  * @retval None
*********************************************************/
// static void SystemReset(void)
void SystemReset(void)
{
    osDelay(100);
    eMBDisable();
    eMBClose();
    NVIC_SystemReset();
}

/*********************************************************
  * @name   ConfigRegister
    * @brief  modbus configuration register command execution
    * @calls
  * @called ResetDefaultParameter(),
                        SystemReset(),
                        SetWorkMode;
  * @param  None
  * @retval None
*********************************************************/
// static void ConfigRegister(uint16_t u16CMD)
//{
//     switch(u16CMD)
//		{
//		    case (CMD_MB_SAVE_FLASH):
//		    {
//		        SaveSettings(1);//保存设置
//				    break;
//		    }
//		    case (CMD_MB_SYS_RESET):
//		    {
//		        SystemReset();//系统复位
//				    break;
//		    }
//		    case (CMD_MB_RESET_DEFAULT):
//		    {
//		        ResetDefaultParameter();//复位默认参数
//						Delay(50);
//						SaveSettings(1);
//				    break;
//		    }
//		    case (CMD_MB_USER_MODE):
//		    {
//		        SetWorkMode(1);//用户模式
//				    break;
//		    }
//		    case (CMD_MB_FACTORY_MODE):
//		    {
//		        SetWorkMode(0);//工厂模式
//				    break;
//		    }
//				default:
//				{
//				    break;
//				}
//		}
// }

/*********************************************************
  * @name   MBGetBaudrate
    * @brief  get current modbus communication baudrate
    * @calls  MBStackRestart(),
                        ResetDefaultParameter(),
  * @called None
  * @param  None
  * @retval baudrate
*********************************************************/
uint16_t MBGetBaudrate(void)
{
#ifdef E2PROM
    g_sVariable.gPara.Bardrate = 19200;
    return g_sVariable.gPara.Bardrate;
#else
    lgsModbus_inst.u16RegBuffer[MB_CFG_BAUDRATE] = g_sVariable.gPara.Bardrate;
    return lgsModbus_inst.u16RegBuffer[MB_CFG_BAUDRATE];
#endif
}

/*********************************************************
  * @name   MBGetAddrsee
    * @brief  get current modbus communication address
    * @calls  MBStackRestart(),
                        ResetDefaultParameter(),
  * @called None
  * @param  None
  * @retval device modbus slave address
*********************************************************/
uint8_t MBGetAddrsee(void)
{
#ifdef E2PROM
    g_sVariable.gPara.CommAddress = 1;
    return g_sVariable.gPara.CommAddress;
#else
    g_sVariable.gPara.CommAddress = 1;
    lgsModbus_inst.u16RegBuffer[MB_CFG_ADDR] = g_sVariable.gPara.CommAddress;
    return lgsModbus_inst.u16RegBuffer[MB_CFG_ADDR];
#endif
}

/*********************************************************
  * @name   MBRegsiterUpdate
    * @brief  update modbus user registers according to internal status and data aquired
    * @calls
                        com_proc(),
  * @called None
  * @param  None
  * @retval None
*********************************************************/
INT8U MBRegsiterUpdate(INT8U u8Byte)
{
#ifdef E2PROM

#else
    memcpy((u8 *)&lgsModbus_inst.u16RegBuffer[MB_CFG_RS], (u8 *)&g_sVariable.gPara.u16RS,
           sizeof(sParameter));  //读风机板数据
#endif

    return TRUE;
}

/*********************************************************
  * @name   MBRegsiterInit
    * @brief  modbus registers initialization, load data from flashrom if is programed, otherwise, set modbus registers
with default value
    * @calls  MBREGDeflaut(),
                        com_proc(),
  * @called MBRegsiterUpdate(0);//MB数据更新
  * @param  None
  * @retval None
*********************************************************/
INT8U MBRegsiterInit(void)
{
#ifdef E2PROM
    sys_global_var_init();
#else
    sModbus *MBREGPtr = (sModbus *)(FLASH_USER_START);
    if (MBREGPtr->u8PgFlag != MB_FLASH_WR_FLAG)
    {
        MBREGDeflaut();  // load default
    }
    else
    {
        memcpy(&lgsModbus_inst, (const void *)FLASH_USER_START, sizeof(sModbus));  // load reg from flash
        memcpy((u8 *)&g_sVariable.gPara.Num, (u8 *)&lgsModbus_inst.u16RegBuffer[MB_CFG_Num], sizeof(sParameter));  //
    }
    SetWorkMode(1);  //用户模式()
    memset((uint8_t *)&g_sVariable.u8ReturnCall[0], 0, CONF_REG_MAP_NUM);
    MBRegsiterUpdate(0);  // MB数据更新
#endif

    return TRUE;
}
/*********************************************************
  * @name   MBResolve
    * @brief  scan writable modbus registers, if modified, call according handler
    * @calls  com_proc();
  * @called MBStackRestart(),
                        ConfigRegister(),
                        SaveSettings();
  * @param  None
  * @retval None
*********************************************************/
void MBResolve(void)
{
    ////		INT8U u8Register;
    //		uint16_t i;
    //
    //		for(i=0;i<REG_HOLDING_NREGS;i++)
    //		{
    //				if(lgsModbus_inst.u8RegStatus[i] != MB_REG_READ)
    //				{
    //						switch(i)
    //						{
    //								case (MB_CFG_ADDR):	//address change//通信地址
    //								{
    //										if((lgsModbus_inst.u16RegBuffer[i]<1)||(lgsModbus_inst.u16RegBuffer[i]>247))
    //										{
    //												lgsModbus_inst.u16RegBuffer[i] =0x01;
    //										}
    //										g_sVariable.gPara.CommAddress=lgsModbus_inst.u16RegBuffer[MB_CFG_ADDR];
    //										MBStackRestart();
    //										ConfigRegister(CMD_MB_SAVE_FLASH);
    //										break;
    //								}
    //								case (MB_CFG_BAUDRATE):	//baudrate change//通信速率
    //								{
    //										if((lgsModbus_inst.u16RegBuffer[i] !=9600)&&(lgsModbus_inst.u16RegBuffer[i] !=4800)
    //\
//											&&(lgsModbus_inst.u16RegBuffer[i] !=2400)&&(lgsModbus_inst.u16RegBuffer[i]
    //!=1200))
    //										{
    //												lgsModbus_inst.u16RegBuffer[i] =9600;
    //										}
    //										g_sVariable.gPara.Bardrate=lgsModbus_inst.u16RegBuffer[MB_CFG_BAUDRATE];
    //										MBStackRestart();
    //										ConfigRegister(CMD_MB_SAVE_FLASH);
    //										break;
    //								}
    //								default:
    //								{
    //										break;
    //								}
    //						}
    //				}
    //				lgsModbus_inst.u8RegStatus[i] = MB_REG_READ;
    //		}
}

// eMBErrorCode
// eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
//{
//     return MB_ENOREG;
// }

// eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
//{
////    extern g_var_st g_var_st_inst;
//
//		eMBErrorCode    eStatus = MB_ENOERR;
//    int             iRegIndex;

//    if( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NUM )
//    {
//
//        iRegIndex = ( int )( usAddress - g_sVariable.sModbus_inst.u8RegStart );
//        switch ( eMode )
//        {
//        // Pass current register values to the protocol stack.
//        case MB_REG_READ:
//            while( usNRegs > 0 )
//            {
//                *pucRegBuffer++ = ( UCHAR ) ( g_sVariable.sModbus_inst.u16RegBuffer[iRegIndex] >> 8 );
//                *pucRegBuffer++ = ( UCHAR ) ( g_sVariable.sModbus_inst.u16RegBuffer[iRegIndex] & 0xFF );
//                iRegIndex++;
//                usNRegs--;
//            }
//            break;

//        // Update current register values with new values from the protocol stack.
//        case MB_REG_WRITE:
//            while( usNRegs > 0 )
//            {
//								if(g_sVariable.sModbus_inst.u8RegProperty[iRegIndex] == 0)					//if reg is RO, quit
//process
//								{
//										eStatus = MB_EIO;
//										break;
//								}
//								g_sVariable.sModbus_inst.u16RegBuffer[iRegIndex] = *pucRegBuffer++ << 8;
//                g_sVariable.sModbus_inst.u16RegBuffer[iRegIndex] |= *pucRegBuffer++;
//								g_sVariable.sModbus_inst.u8RegStatus[iRegIndex] = MB_REG_WRITE;								//content change measn
//new cmd request

//                iRegIndex++;
//                usNRegs--;
//            }
//            break;
//        }
//    }
//    else
//    {
//        eStatus = MB_ENOREG;
//    }
//    return eStatus;
//}

// eMBErrorCode
// eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
//                eMBRegisterMode eMode )
//{
//     return MB_ENOREG;
// }

// eMBErrorCode
// eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
//{
//     return MB_ENOREG;
// }

/*********************************************************
  * @name   mb_get_baudrate
    * @brief  get current modbus communication baudrate
    * @calls  cmd_mb_stack_restart(),
                        restore_factory(),
  * @called None
  * @param  None
  * @retval baudrate
*********************************************************/
ULONG mb_get_baudrate(uint16_t baudrate)
{
    ULONG ulBaudRate = 0;
    switch (baudrate)
    {
        case BAUD_4800:
            ulBaudRate = 4800;
            break;
        case BAUD_9600:
            ulBaudRate = 9600;
            break;
        case BAUD_19200:
            ulBaudRate = 19200;
            break;
        case BAUD_38400:
            ulBaudRate = 38400;
            break;
        default:
            ulBaudRate = 9600;
            break;
    }
    ulBaudRate = 19200;
    return ulBaudRate;
}

/*********************************************************
  * @name   mb_get_device_addr
    * @brief  get current modbus communication address
    * @calls  cmd_mb_stack_restart(),
                        restore_factory(),
  * @called None
  * @param  None
  * @retval device modbus slave address
*********************************************************/
uint8_t mb_get_device_addr(void)
{
    //	return (UCHAR)g_sVariable.gPara.CommAddress;
    return (1);
}

static uint16_t mbs_read_reg(uint16_t read_addr)
{
    extern conf_reg_map_st conf_reg_map_inst[];
    //	extern  sts_reg_map_st status_reg_map_inst[];
    // if((read_addr >= CONFIG_REG_MAP_OFFSET)&&(read_addr<CONF_REG_MAP_NUM + CONFIG_REG_MAP_OFFSET))
    //	if(read_addr<CONF_REG_MAP_NUM + CONFIG_REG_MAP_OFFSET)
    if (read_addr < REG_MAP_NUM + CONFIG_REG_MAP_OFFSET)
    {
        return (*(conf_reg_map_inst[read_addr - CONFIG_REG_MAP_OFFSET].reg_ptr));
    }
    //	else if((STATUS_REG_MAP_OFFSET <= read_addr)&&(read_addr<( STATUS_REG_MAP_OFFSET+STATUS_REG_MAP_NUM)))
    //	{
    //		 return(*(status_reg_map_inst[read_addr-STATUS_REG_MAP_OFFSET].reg_ptr));
    //	}
    else
    {
        return (ABNORMAL_VALUE);
    }
}
