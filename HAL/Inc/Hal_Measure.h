#ifndef __HAL_MEASURE_H
#define __HAL_MEASURE_H

#include "stm32g0xx_hal.h"
#include <macro.h>
#include <stdio.h>

//#define  ATT7026E  1  		//计量芯片ATT7026E

#define MEASURE_RST_PIN GPIO_Pin_0
#define MEASURE_RST_PORT GPIOA
#define MEASURE_RST_SOURCE GPIO_PinSource0
#define MEASURE_RST_AF GPIO_AF_0

#define MEASURE_IRQ_PIN GPIO_Pin_1
#define MEASURE_IRQ_PORT GPIOB
#define MEASURE_IRQ_SOURCE GPIO_PinSource1
#define MEASURE_IRQ_AF GPIO_AF_0

#define MEASURE_RST_LOW() GPIO_ResetBits(MEASURE_RST_PORT, MEASURE_RST_PIN)
#define MEASURE_RST_HIGH() GPIO_SetBits(MEASURE_RST_PORT, MEASURE_RST_PIN)

#define MEASURE_IRQ_READ GPIO_ReadInputDataBit(MEASURE_IRQ_PORT, MEASURE_IRQ_PIN)

//所用到的寄存器地址
#define Check_CAL01 0x3E  //校正数据校验1
#define Check_CAL02 0x5E  //校正数据校验2

//
#define ATT_RMS 0x2000         // 2^13//ATT7026E计量参数
#define ATT_COMPLE 0x800000    // 2^23//ATT7026E计量参数
#define VOL_THRESHOLD 0x1E000  //电压门限15V*8192
#define CUR_THRESHOLD 1000     //电流门限
#define POW_THRESHOLD 200      //功率门限
#define POWER_K_5_60A \
    0.00961987791144536114570361  //功率参数系数K=2.592*10^10/(HFconst*EC*2^23)//HFConst=INT[2.592*10^10*G*G*Vu*Vi/(EC*Un*Ib)]
//#define POWER_K_5_60A 	9.6195048578499532142263731843185e-6
#define HFconst_5_60A 0x0323
#define POWER_K2 3089.90478515625  //功率系数二次拆分2.592*10^10/2^23
//#define VOL_DIFFERENCE  30   //电压差
#define VOL_DIFFERENCE 0.88  //电压差
#define VOL_ANGLE 0x100000   // 2^20

#define PHASEFALIURE 22  //

#define CUROFFSET 1000 * 1.03  //电流校正系数
#define ItoV 19.85             //电流转电压系数

//校表参数寄存器
typedef struct
{
    U8 REG_Addr;       //寄存器地址
    U16 DefaultValue;  //寄存器值
} REG_Default;

//电源采集结构体
#define PHASE 3  // A、B、C三相
typedef struct
{
    INT16U u16State;              //电源板状态
    INT16U u16Voltage[PHASE];     //电压
    INT16U u16GridFrequency;      //电网频率
    INT16U u16PhaseSequence;      //相序
    INT16U u16Current[PHASE];     //电流
    INT16U u16PhaseAngle[PHASE];  //相角差
    INT32U_UNION PTest;           // TEST
} sPower;

//校正数据结构体
typedef struct  //
{
    INT8U Calibrate_Register[(0x40) * 2];  //参数配置,ATT7026E手册第45-67页,(Regster:00H~39H)
                                           //		INT8U
    //Calibrate_Register[(0x40)*2+(0x71-0x60+0x01)*2];//参数配置,ATT7026E手册第45-67页,(Regster:00H~03H,16H,1DH~20H,30H~37H)
    INT8U Calibrate_Flag[2];  //校表标志、反码校验
    INT8U Phase_Register[(0x03) * 2];
    INT8U Calibrate_CRC[2];   // CRC校验
    INT8U Calibrate_Sum1[3];  // 0x00-0x39校验和
    //		INT8U Calibrate_Sum2[3];//0x60-0x61校验和
    ////		INT8U Calibrate_Date[5];//最后一次校正时间(?.?.?.?.?,5??)
} sMeasureCalibrate;
#define CAL_REGLEN sizeof(sMeasureCalibrate) - 13
#define CAL_LEN sizeof(sMeasureCalibrate) - 0x40 * 2
#define CAL_CRC_LEN CAL_LEN - 3 - 2

#define CAL_REL_LEN 7 + 6  //实际存入
#define PHASE_A_GAIN 0x17  // A相电压增益寄存器

#define READ_A_PHASE_VOLTAGE_RMS (0x0D)     //三相电压有效值寄存器
#define READ_A_PHASE_CURRENT_RMS (0x10)     //三相电流有效值寄存器
#define READ_A_PHASE_ACTIVE_POWER (0x01)    // ????,A,B,C,??????
#define READ_A_PHASE_REACTIVE_POWER (0x05)  // ????,A,B,C,??????
#define READ_APPARENT_POWER (0x09)          // ????A,B,C,??????
#define READ_A_PHASE_POWER_FACTOR (0x14)    //三相功率因素
#define READ_GRID_FREQUENCY (0x1C)          //电网频率
#define READ_STATE_REGISTER (0x2C)          //状态寄存器
#define READ_A_PHASE_VOLTAGE_ANGLE (0x26)   //三相电压夹角

extern void Measure_Init(void);
extern BOOL Measure_HardReset(void);
extern BOOL CalibrateWrite(U8 WriteRegister, U16 CAL_Value);
extern BOOL CalibrateClean(void);
extern void CalibrateInit(void);
// extern BOOL GetCalibrateFlag(void);
extern BOOL Calibrate_CheckSum_Error(void);
extern BOOL Calibrate_CheckSum(U8* pBuff);
extern void Measure(void);
extern void Write_MeasureResgster(unsigned char Address, unsigned char* pValue, char Length);
extern void Read_MeasureResgster(unsigned char Address, unsigned char* pValue, char Length);

#endif /* __HAL_MEASURE_H */
