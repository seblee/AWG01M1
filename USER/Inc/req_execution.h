#ifndef __REQ_EXE_H__
#define __REQ_EXE_H__
#include "stdint.h"

#define HALF_SEC 2

enum
{
    FAN_FSM_STATE = 0,
    COMPRESS_SIG_FSM_STATE,
    COMPRESS1_FSM_STATE,
    HEATER_FSM_STATE,
    L_FSM_STATE_MAX_NUM,
};

enum
{
    FAN_GEAR_NO = 0,
    FAN_GEAR_LOW,
    FAN_GEAR_MID,
    FAN_GEAR_HIGH,
};
#define FAN_GEAR_START FAN_GEAR_LOW

enum
{
    COMPRESSOR_FSM_STATE_IDLE = 0,
    COMPRESSOR_FSM_STATE_INIT,
    COMPRESSOR_FSM_STATE_STARTUP,
    COMPRESSOR_FSM_STATE_NORMAL,
    COMPRESSOR_FSM_STATE_SHUTING,
    COMPRESSOR_FSM_STATE_STOP,
};

//变频压机状态输出
enum
{
    INV_COMP_IDLE= 0,
    INV_COMP_STARTUP,
    INV_COMP_NORMAL,
    INV_COMP_RETOIL,
    INV_COMP_OFF,
};

//除霜条件
#define RUNOK_TIME  			5*60*2	
#define DUR_TIME  				3*60*2	
#define LASTDEFROST_TIME  45*60
#define HIGHPRESS  				380
#define LOPRESS  					20
#define EXTEMP  					1150
#define DEFROST_TIME  		3*60
#define HI_PRESS  				300

#define TASK_FD  	2

//上次状态
enum
{
    LAST_ONOFF= 0,
    LAST_DEFROST,
};

//机组正常运行时间
#define RUNTIME  600//10分钟

#define HP_THR  610//高压压力
#define LP_THR  200//低压压力
#define HLP_TIME  180//3分

//告警屏蔽时间
#define SHIELD_TIME   180

//EEV状态机
enum
{
    FSM_EEV_IDLE = 0,
    FSM_EEV_START_UP,
    FSM_EEV_NORM,
    FSM_EEV_SHUT,
};

//EEV模式
enum
{
    EEV_CLOSE = 0,
    EEV_OPEN,
    EEV_TEST,
};


enum
{
    EEV_AUTO = 0,				//线圈off
    EEV_MANUAL = 0xFF00,//线圈on
};
#define EEV_MIN_STEP	5
#define EEV_MAX_STEP	480

//风机状态机
enum
{
    FSM_FAN_IDLE = 0,
    FSM_FAN_INIT,
    FSM_FAN_START_UP,
    FSM_FAN_NORM,
    FSM_FAN_SHUT
};

//运行模式状态切换
enum
{
    MODE_STATE_IDLE = 0,
    MODE_STATE_NORMAL,
    MODE_STATE_HT1,
    MODE_STATE_HT2,
    MODE_STATE_SWNT1,
    MODE_STATE_SWNT2,
    MODE_STATE_SWTN1,
    MODE_STATE_SWTN2,
    MODE_STATE_STOP,
};

//制水模式
enum
{
    MODE_WATER= 0,	//制水优先
    MODE_ENERGY,		//能效优先
    MODE_EXHIBITION,		//展会模式
    MODE_HTEMP1,		//高温1
    MODE_HTEMP2,		//高温2
    MODE_SWHT1=0x10,//高温1切换
    MODE_SWHT2=0x20,//高温1切换
    MODE_SWTN1=0x40,//高温1切换
    MODE_SWTN2=0x80,//高温1切换
    MODE_MAX			//
};
//制水方式
enum
{
    WATER_AUTO= 0,	//自动
    WATER_MANUL,		//手动
    WATER_MAX			//
};

enum
{
    WL_000 = 65466, //低水位65466
    WL_001 = 150,
    WL_010 = 9,
    WL_011 = 449,
    WL_100 = 0x7FFF,
    WL_101 = 0x7FFF,
    WL_110 = 0x7FFF,
    WL_111 = 0x7FFF,
    WL_OFFSET = 50,
};
enum
{
    WL_L = WL_001, //低水位150
    WL_M = WL_011, //
    WL_U = WL_111, //满水
};

//水箱
enum
{
    SW_TANK = 0x00,//源水箱
    DW_TANK = 0x01,//饮水箱
};

//水源模式
enum
{
    WATER_AIR = 0x00,
    WATER_FILL = 0x01, //注水/外接
    WATER_FLUSH = 0x02, //冲洗
    WATER_Storage = 0x03, //清空水箱
    WATER_Disinfection = 0x04, //消毒
    WATER_MAXTYPE,
};

enum
{
    FC_WL = 0x01, //水位
    FC_PT = 0x02, //
    FC_TH = 0x04, //温湿度
    FC_WS = 0x08, //外置水箱
    FC_DF = 0x10, //化霜
    FC_NT = 0x20, //高温1
	  FC_ALARMSTOP = 0x100, //停机告警
};

//压机状态
enum
{
    COMP_IDLE= 0,	//空闲
    COMP_START=0x01,		//启动
    COMP_OK=0x02,		//启动
    COMP_FI=0x04,		//启动完成
    COMP_STOP=0x08,		//关闭
    COMP_MAX			//
};

//UV状态
enum
{
    UV1_ON= 0x01,	//
    UV2_ON= 0x02,	//
    UV3_ON= 0x04,	//
    UV_OFFSET= 50,	////UV正常值
    UV_ERR= 0x80,	//
};

/************************************************************************/
//贮存状态机
enum
{
    WATER_STROGE_IDLE = 0,
    WATER_STROGE_1,
    WATER_STROGE_2,
    WATER_STROGE_3,
    WATER_STROGE_4,
    WATER_STROGE_5,
    WATER_STROGE_6,
    WATER_STROGE_7,
    WATER_STROGE_8,
    WATER_STROGE_STOP,
};


//饮水箱水位
#define	D_FULL	1000//满水

//出水模式
enum
{
    WATER_NO = 0x00,
    WATER_NORMAL_ICE = 0x01,
    WATER_HEAT = 0x02,
    WATER_ICE = 0x04, //冰水
};

//水路控制方案
enum
{
    HEART_POT = 0x01, //热灌
    HMI_KEY = 0x02,
    OPEN_PWP = 0x04, //开盖时，打开净化泵
    TWO_COLD = 0x08, //双路出水
};
#define ChildKey_Cnt 1
#define ChildKey_Lose 5

//水位
enum
{
    S_NO = 0,
    S_L = 0x01,
    S_M = 0x02,
    S_U = 0x03,
    S_E = 0x10,
};
//水位
enum
{
    D_NO = 0,
    D_L = 0x01,
    D_M = 0x02,
    D_U = 0x03,
    D_E = 0x10,
};

//流量脉冲
enum
{
    L200 = 380,
    L300 = 570,
    L500 = 1250,
    L1000 = 2750,
    L1500 = 4350,
    L2000 = 5450,
};
//流量因子
#define L300_FACTOR 0.405f
#define L500_FACTOR 0.4f
//#define L200_FACTOR   0.28f
//#define L300_FACTOR   0.33f
//#define L500_FACTOR   0.34f
#define L1000_FACTOR 0.36363636363636363636363636363636f
#define L1500_FACTOR 0.34482758620689655172413793103448f
#define L2000_FACTOR 0.36697247706422018348623853211009f

//加热器流量,500ml/MIN,60s*HEAT_FACTOR
#define HEAT_FACTOR_S 8.3333333333333333333333333333333f
#define HEAT_FACTOR_500MS 4.1666666666666666666666666666667f

//出水状态
enum
{
    WATER_IDLE = 0,
    HEATER_SEND,
    WATER_READ,
    WATER_OUT,
};

//Close first
enum
{
    PUMP_FIRET = 0x00,
    VALVE_FIRST = 0x01, //
};

#define EV_DELAY  3*TASK_FD//3s

//加热器控制
enum
{
    CLOSE_HEAT = 0,
    OPEN_HEAT,
};

//加热器出水状态
enum
{
    HEAT_NO = 0,
    HEAT_OUTWATER = 0x01,
};

#define RH_DEALY 10
#define WRITEHEAT_MAX 250
#define CLOSEHEAT_MAX 5

//杀菌
#define STERILIZE_BIT0 0x01
#define STERILIZE_BIT1 0x02
//单次出水时间限制
#define WATER_MAXTIME 600 * 2 //10分钟

#define SEC_FACTOR	1
#define MIN_FACTOR	60 * 2 
//定时保存时间
#define FIXED_SAVETIME 900


#define D_A  17.27
#define D_B  237.7

//制水测试状态
enum
{
    TEST_WATER_NO = 0,
    TEST_WATER_IN 	,
    TEST_WATER_OUT	,
};


void hum_capacity_calc(void);
void req_execution(void);
void req_bitmap_op(uint8_t component_bpos, uint8_t action);
void Close_DIS_PWR(void);
void UV_req_exe(uint8_t u8Type,uint8_t u8Delay);
void WaterPressure_exe(void);
void Storage_exe(uint8_t u8Type);
void WaterDrain_exe(uint8_t u8Type);
uint8_t Test_Produce_Water_Req(void);
void Circle_exe(void);
extern uint8_t Exit_Water(void);
#endif //__REQ_EXE_H__
