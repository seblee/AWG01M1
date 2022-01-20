#ifndef __SYS_CONF
#define __SYS_CONF

#include "sys_def.h"
//#include "alarms.h"
#include "string.h"
#include "Drv_Adc.h"
#include "stm32g0xx_hal.h"
#include "req_execution.h"

//数字输出映射
enum
{
    DO_ZYB_BPOS = 0,  //增压泵
    DO_RSB_BPOS,      //热水泵
    DO_LSB_BPOS,      //冷水泵
    DO_LSF_BPOS,      //冷水阀
    DO_ZLF_BPOS,      //制冷阀
    DO_JSF_BPOS,      //进水阀
    DO_RS2_BPOS,      // RS2
    DO_CSF_BPOS,      //除霜阀
    DO_RS1_BPOS,      // RS1
    DO_JSB_BPOS,      //进水泵
    DO_RS3_BPOS,      // RS3
    DO_RS4_BPOS,      // RS4
    DO_RS5_BPOS,      // RS5
    DO_LED_UV1_BPOS,  // LED_UV1
    DO_LED_UV2_BPOS,  // LED_UV2
    DO_LED_UV3_BPOS,  // LED_UV3

    DO_COMP_BPOS,   //压缩机
    DO_HEAT_BPOS,   //加热
    DO_FN1_BPOS,    //风机1
    DO_FN2_BPOS,    //风机2
    DO_UV1_BPOS,    // UV1
    DO_UV2_BPOS,    // UV2
    DO_UV3_BPOS,    // UV3
    DO_RSV_BPOS_1,  //预留
    DO_RSV_BPOS_2,  //预留
    DO_RSV_BPOS_3,  //预留
    DO_RSV_BPOS_4,  //预留
    DO_RSV_BPOS_5,  //预留
    DO_RSV_BPOS_6,  //预留
    DO_RSV_BPOS_7,  //预留
    DO_RSV_BPOS_8,  //预留
    DO_RSV_BPOS_9,  //预留

    DO_FILLTER_BPOS,            //滤网
    DO_FILLTER_ELEMENT_BPOS_0,  //滤芯  0
    DO_FILLTER_ELEMENT_BPOS_1,  //滤芯  1
    DO_FILLTER_ELEMENT_BPOS_2,  //滤芯  2
    DO_FILLTER_ELEMENT_BPOS_3,  //滤芯  3
    DO_MAX_CNT,
};

#define DO_FAN_BPOS DO_FN2_BPOS      //风机低档
#define DO_FAN_LOW_BPOS DO_FAN_BPOS  //风机低档
#define DO_COMP1_BPOS DO_COMP_BPOS   //压缩机

#define DO_WV_WaterOut_BPOS DO_LSF_BPOS  //出水阀

#define DO_LED_WaterOut_BPOS DO_LED_UV2_BPOS  //出水LED

#define DO_FILLTER_ELEMENT_MAX DO_FILLTER_ELEMENT_BPOS_3  //滤芯

///////////////////////////////////////////////////////////////
// system configuration
///////////////////////////////////////////////////////////////

enum
{
    PWR_STS_BPOS = 0,    //开机
    FAN_STS_BPOS,        //风机
    COOLING_STS_BPOS,    //制冷
    PROWATER_STS_BPOS,   //制水
    OUTWATER_STS_BPOS,   //出水
    STERILIZE_STS_BPOS,  //杀菌
    DEFROST1_STS_BPOS,   //除霜1
    DEFROST2_STS_BPOS,   //除霜2
    HEATING_STS_BPOS,    //加热
    EXITWATER_STS_BPOS,  //外接水源
    NET_STS_BPOS,        //网络
    OTA_STS_BPOS,        // OTA更新
    STORAGE_STS_BPOS,    //贮存

    ALARM_STUSE_BPOS = 14,
    ALARM_BEEP_BPOS  = 15,
};

//手动测试模式
enum
{
    TEST_MANUAL_UNABLE = 0x00,  //退出测试模式
    TEST_MODE_ENABLE   = 0x01,  //测试模式
    MANUAL_MODE_ENABLE = 0x02,  //手动模式
    TEST_Vacuum_ENABLE = 0x03,  //抽真空
};

enum
{
    BITMAP_REQ = 0,
    BITMAP_ALARM,
    BITMAP_MANUAL,
    BITMAP_FINAL,
    BITMAP_MASK,
    BITMAP_MAX_CNT,
};

typedef union
{
    uint16_t u16Data;
    uint8_t u8Data[2];
} U16_8Data;

typedef struct
{
    uint8_t Fan_Gear;          //风机档位,Alair，20161113
    uint8_t u8FanOK;           //风机OK
    uint16_t Fan_Close;        //风机开关信号
    uint16_t u16Shield_timer;  //告警启动屏蔽时间
} L_Fan_st;

typedef struct
{
    uint16_t u16RunOK[2];          //运行OK
    uint16_t u16Defrost_In[2];     //除霜
    uint8_t u8Defrost;             //除霜
    uint16_t u16Defrost_Start;     //除霜时间
    uint16_t u16Defrost_Duration;  //除霜持续时间
    uint16_t u16Defrost_End;       //上次除霜结束
    uint8_t u8CompOK;              //压机正常启动
    uint32_t u32CompRunTime;       //压机运行时间
    uint16_t comp_startup_interval;
    uint16_t comp_stop_interval;
    uint16_t comp_inv_interval;
    uint16_t u16Mode_state;
    uint16_t u16Mode_req;
    uint8_t u8Last_Status;  //上次运行状态
    uint16_t Comp_Close;    //压缩机开关信号
    //    uint16_t u16Mbm_EevMode[EEV_MAX_CNT];			//
} L_Comp_st;

typedef struct
{
    uint8_t Pwp_Open;        //净化泵开关信号
    uint16_t Pwp_Open_Time;  //净化泵打开时间
    uint8_t Sterilize;       //杀菌
    uint8_t OutWater_Flag;   //出水中
    uint16_t OutWater_Del;   //出水延时
    uint8_t OutWater_OK;     //出水完成
    uint8_t u8StartDelay;    //出水开始延时
    uint8_t u8CloseDelay;    //出水结束延时
    uint16_t TH_Check_Interval;
    uint8_t OutWater_Key;     //按键出水
    uint16_t OutWater_Delay;  //按键出水延时
} L_Water_st;

typedef struct
{
    uint16_t bitmap[2][BITMAP_MAX_CNT];
    uint16_t l_fsm_state[L_FSM_STATE_MAX_NUM];
    //		int16_t 	ao_list[AO_MAX_CNT][BITMAP_MAX_CNT];
    uint16_t comp_timeout[DO_MAX_CNT];
    L_Fan_st Fan;
    L_Comp_st Comp;
    L_Water_st Water;
} local_reg_st;

typedef struct
{
    uint16_t din_bitmap_polarity;
    uint16_t din;
    uint16_t din_pusl;
    uint16_t dout[2];
    uint16_t ain;
    uint16_t aout;
    //    uint16_t  pwm_out;
} dev_mask_st;

// fan
typedef struct
{
    uint16_t u16TH_Interal;   //温湿度检查间隔
    uint16_t u16Start_Temp;   //制水启动温度
    uint16_t u16Stop_Temp;    //制水停止温度
    uint16_t u16TH_Cali[2];   //温湿度校正
    uint16_t u16NTC_Cali[6];  // NTC校正
} TH_inst;

// fan
typedef struct
{
    uint16_t mode;              //风机模式
    uint16_t startup_delay;     //开机延时
    uint16_t stop_delay;        //停机延时
    uint16_t cold_start_delay;  //冷启动延时
    uint16_t set_speed;         //风机初始转速
    uint16_t min_speed;         //最小转速
    uint16_t max_speed;         //最大转速
} Fan_inst;

typedef struct
{
    uint16_t u16Start_Defrost_Temp;  //除霜启动温度
    uint16_t u16Stop_Defrost_Temp;   //除霜停止温度
    uint16_t u16Defrost_InDelay;     //除霜延迟开启时间
    uint16_t u16Defrost_Duration;    //除霜持续时间
    uint16_t u16Defrost_Max;         //最长除霜时间
} Defrost_inst;

// compressor
typedef struct
{
    uint16_t u16startup_delay;  //启动延迟
    uint16_t u16stop_delay;     //停机延迟
    uint16_t u16min_runtime;    //最短运行时间
    uint16_t u16min_stoptime;   //最短停机时间
    uint16_t startup_lowpress_shield;
    Defrost_inst Defrost;  //除霜
} Compressor_inst;

// compressor
typedef struct
{
    uint16_t u16Water_Ctrl;
    uint16_t u16Water_Mode;           //出水模式
    uint16_t u16Water_Flow;           //出水流量
    uint16_t u16HotWater_Temp;        //热水温度
    uint16_t u16HotWater_Cali;        //热水温度校正
    uint16_t u16CloseUV;              //关闭紫外灯
    uint16_t u16ExitWater_Mode;       //水源模式
    uint16_t u16Sterilize_Interval;   //杀菌间隔
    uint16_t u16Sterilize_Time;       //杀菌时间
    uint16_t u16StartDelay;           //开启延时
    uint16_t u16CloseDelay;           //关闭延时
    uint16_t u16FILTER_ELEMENT_Type;  //滤芯告警类型:1-流量L;0-时间h
    uint16_t u16TDS_Cail;             // TDS校正
} Water_inst;

// alarms: acl definition
/*
    @id:   alarm id
    @delay:  trigger&clear delay
    @timeout: delay timeout count down
    @trigger_time: alarm trigger time
    @enable mode: alarm enable mode
        `0x00:  enable
        `0x01:  suspend
        `0x02:  forbid
    @enable mask: alarm enable mask
        '0x03: all mode enable
        '0x02: enable or forbid
        '0x01: enable or suspend
        '0x00: only enable
    @alarm_param: related paramter(eg. threshold)
    @void (*alarm_proc): designated alarm routine check function
*/

typedef struct
{
    uint16_t id;
    uint16_t delay;
    uint16_t enable_mode;
    uint16_t alarm_param;
} alarm_acl_conf_st;

#endif  //	__SYS_CONF
