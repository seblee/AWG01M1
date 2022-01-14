#ifndef __GLOBAL_H
#define __GLOBAL_H

#include "macro.h"
//#include "App_TemperatureHumidity.h"
//#include "Hal_Measure.h"
#include "mb_cb.h"
#include "App_Communiction.h" 
#include "App_Core.h"
#include "i2c_bsp.h"
#include "adc.h"
#include "TH_SENSOR_BSP.h"
#include "sys_conf.h"
#include "alarms.h"

#define FACTORY_RESET 7u
#define MANUAL_TSET 17u
#define CLEAR_RT 22u
#define CLEAR_ALARM 23u

#define ABNORMAL_VALUE 0x7FFF  //异常值
#define CONF_REG_MAP_NUM 80
#define REG_MAP_NUM 120

typedef struct
{
    INT16U Num;              //地址范围
    INT16U Software;         //软件版本
    INT16U Hardware;         //硬件版本
    INT16U RS0[2];           //序列号
    uint16_t u16Power_Mode;  //开关机			u16Power_Mode
    INT16U WorkMode;         //通信地址
    INT16U CMD;              //通信地址
    INT16U CommAddress;      //通信地址
    INT16U Bardrate;         //波特率

    INT16U u16RS;  //
    dev_mask_st dev_mask;
    uint16_t u16Manual_Test_En;  // test mode enalbe
    uint16_t diagnose_mode_en;   // diagnose mode enalbe
    uint16_t u16Reset;           //清除告警
    uint16_t u16Clear_RT;        //清除部件时间
    uint16_t u16Clear_ALARM;     //清除告警

    uint16_t alarm_remove_bitmap[ALARM_TOTAL_WORD];  // reless alarm
    alarm_acl_conf_st alarm[ACL_TOTAL_NUM];
    TH_inst TH;
    Fan_inst fan;
    Compressor_inst CP;
    Water_inst Water;
} sParameter;

enum
{
    WORK_MODE_STS_REG_NO = 0,  //机组工作状态
    COM_STS_REG_NO,            //通信状态
    GEN_STS_REG_NO,            //机组工作模式
    MAX_REG_NO,                //
};

typedef struct
{
    uint16_t u16WL;
    uint16_t u16Cumulative_Water[3];  //累计取水
    uint16_t u16UVStatus;             // UV开启状态
    uint16_t u16Fliter;               //滤网状态
    uint16_t u16FliterElement[4];     //滤芯状态
    uint16_t u16TDS[2];               // TDS值
    uint16_t u16FC_ON;
} Water_st;
typedef struct
{
    uint16_t u16Status_remap[MAX_REG_NO];
    uint16_t u16AO;                      //输出
    uint16_t u16AI[AI_MAX_CNT];          //模拟量
    uint16_t u16DI_Bitmap[1];            // DI
    uint16_t u16DO_Bitmap[2];            // DO
    uint16_t u16Runtime[2][DO_MAX_CNT];  //使用时间
    uint16_t alarm_bitmap[2];            //
    Com_tnh_st u16TH[2];                 //温湿度
    Water_st Water;                      //水
    uint16_t Com_error;                  //通信异常
    uint16_t REQ_TEST[8];

} status_st;

typedef struct
{
    //		sTemperatureHumidity		sTemperatureHumidity_inst;			//TemperatureHumidity structure
    //		sPower									sPower_inst;			//system Power structure
    //		sMeasureCalibrate				sMeasureCalibrate_inst;
    //		INT8 u8SA;										//温湿度板标志
    //		INT8 u8Addr;										//温湿度板地址
    uint16_t u8UpdateAddr;
    sParameter gPara;
    status_st status;
    uint8_t u8ReturnCall[CONF_REG_MAP_NUM];
} sVariable;

// typedef struct
//{
//		uint16_t 	id;
//		uint16_t*	reg_ptr;
//		int16_t		min;
//		uint16_t	max;
//		uint16_t	dft;
//		uint8_t		rw;
//		uint8_t (*chk_ptr)(uint16_t pram);
// }Conf_Reg_Map_st;

typedef struct
{
    uint16_t id;
    uint16_t *reg_ptr;
    uint16_t min;
    uint16_t max;
    uint16_t dft;
    //	uint8_t		permission;
    uint8_t rw;
    uint8_t (*chk_ptr)(uint16_t pram);
} conf_reg_map_st;

#define HARDERR 0x8000

extern sVariable g_sVariable;

extern uint16_t save_conf_reg(uint8_t addr_sel);
extern uint16_t set_load_flag(uint8_t ee_load_flag);
extern uint16_t sys_global_var_init(void);
extern uint16_t sys_local_var_init(void);
extern int16_t eeprom_tripple_write(uint16_t reg_offset_addr, uint16_t wr_data, uint16_t rd_data);

extern int16_t eeprom_compare_read(uint16_t reg_offset_addr, uint16_t *rd_data);

extern uint16_t reg_map_write(uint16_t reg_addr, uint16_t *wr_data, uint8_t wr_cnt);
extern uint8_t load_factory_pram(void);
extern int16_t CallBack_map_write(void);
extern uint8_t reset_runtime(uint16_t param);

#endif /*__GLOBAL_VAR_H*/
