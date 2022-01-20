#ifndef __SYS_CONF
#define __SYS_CONF

#include "sys_def.h"
//#include "alarms.h"
#include "string.h"
#include "Drv_Adc.h"
#include "stm32g0xx_hal.h"
#include "req_execution.h"

//�������ӳ��
enum
{
    DO_ZYB_BPOS = 0,  //��ѹ��
    DO_RSB_BPOS,      //��ˮ��
    DO_LSB_BPOS,      //��ˮ��
    DO_LSF_BPOS,      //��ˮ��
    DO_ZLF_BPOS,      //���䷧
    DO_JSF_BPOS,      //��ˮ��
    DO_RS2_BPOS,      // RS2
    DO_CSF_BPOS,      //��˪��
    DO_RS1_BPOS,      // RS1
    DO_JSB_BPOS,      //��ˮ��
    DO_RS3_BPOS,      // RS3
    DO_RS4_BPOS,      // RS4
    DO_RS5_BPOS,      // RS5
    DO_LED_UV1_BPOS,  // LED_UV1
    DO_LED_UV2_BPOS,  // LED_UV2
    DO_LED_UV3_BPOS,  // LED_UV3

    DO_COMP_BPOS,   //ѹ����
    DO_HEAT_BPOS,   //����
    DO_FN1_BPOS,    //���1
    DO_FN2_BPOS,    //���2
    DO_UV1_BPOS,    // UV1
    DO_UV2_BPOS,    // UV2
    DO_UV3_BPOS,    // UV3
    DO_RSV_BPOS_1,  //Ԥ��
    DO_RSV_BPOS_2,  //Ԥ��
    DO_RSV_BPOS_3,  //Ԥ��
    DO_RSV_BPOS_4,  //Ԥ��
    DO_RSV_BPOS_5,  //Ԥ��
    DO_RSV_BPOS_6,  //Ԥ��
    DO_RSV_BPOS_7,  //Ԥ��
    DO_RSV_BPOS_8,  //Ԥ��
    DO_RSV_BPOS_9,  //Ԥ��

    DO_FILLTER_BPOS,            //����
    DO_FILLTER_ELEMENT_BPOS_0,  //��о  0
    DO_FILLTER_ELEMENT_BPOS_1,  //��о  1
    DO_FILLTER_ELEMENT_BPOS_2,  //��о  2
    DO_FILLTER_ELEMENT_BPOS_3,  //��о  3
    DO_MAX_CNT,
};

#define DO_FAN_BPOS DO_FN2_BPOS      //����͵�
#define DO_FAN_LOW_BPOS DO_FAN_BPOS  //����͵�
#define DO_COMP1_BPOS DO_COMP_BPOS   //ѹ����

#define DO_WV_WaterOut_BPOS DO_LSF_BPOS  //��ˮ��

#define DO_LED_WaterOut_BPOS DO_LED_UV2_BPOS  //��ˮLED

#define DO_FILLTER_ELEMENT_MAX DO_FILLTER_ELEMENT_BPOS_3  //��о

///////////////////////////////////////////////////////////////
// system configuration
///////////////////////////////////////////////////////////////

enum
{
    PWR_STS_BPOS = 0,    //����
    FAN_STS_BPOS,        //���
    COOLING_STS_BPOS,    //����
    PROWATER_STS_BPOS,   //��ˮ
    OUTWATER_STS_BPOS,   //��ˮ
    STERILIZE_STS_BPOS,  //ɱ��
    DEFROST1_STS_BPOS,   //��˪1
    DEFROST2_STS_BPOS,   //��˪2
    HEATING_STS_BPOS,    //����
    EXITWATER_STS_BPOS,  //���ˮԴ
    NET_STS_BPOS,        //����
    OTA_STS_BPOS,        // OTA����
    STORAGE_STS_BPOS,    //����

    ALARM_STUSE_BPOS = 14,
    ALARM_BEEP_BPOS  = 15,
};

//�ֶ�����ģʽ
enum
{
    TEST_MANUAL_UNABLE = 0x00,  //�˳�����ģʽ
    TEST_MODE_ENABLE   = 0x01,  //����ģʽ
    MANUAL_MODE_ENABLE = 0x02,  //�ֶ�ģʽ
    TEST_Vacuum_ENABLE = 0x03,  //�����
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
    uint8_t Fan_Gear;          //�����λ,Alair��20161113
    uint8_t u8FanOK;           //���OK
    uint16_t Fan_Close;        //��������ź�
    uint16_t u16Shield_timer;  //�澯��������ʱ��
} L_Fan_st;

typedef struct
{
    uint16_t u16RunOK[2];          //����OK
    uint16_t u16Defrost_In[2];     //��˪
    uint8_t u8Defrost;             //��˪
    uint16_t u16Defrost_Start;     //��˪ʱ��
    uint16_t u16Defrost_Duration;  //��˪����ʱ��
    uint16_t u16Defrost_End;       //�ϴγ�˪����
    uint8_t u8CompOK;              //ѹ����������
    uint32_t u32CompRunTime;       //ѹ������ʱ��
    uint16_t comp_startup_interval;
    uint16_t comp_stop_interval;
    uint16_t comp_inv_interval;
    uint16_t u16Mode_state;
    uint16_t u16Mode_req;
    uint8_t u8Last_Status;  //�ϴ�����״̬
    uint16_t Comp_Close;    //ѹ���������ź�
    //    uint16_t u16Mbm_EevMode[EEV_MAX_CNT];			//
} L_Comp_st;

typedef struct
{
    uint8_t Pwp_Open;        //�����ÿ����ź�
    uint16_t Pwp_Open_Time;  //�����ô�ʱ��
    uint8_t Sterilize;       //ɱ��
    uint8_t OutWater_Flag;   //��ˮ��
    uint16_t OutWater_Del;   //��ˮ��ʱ
    uint8_t OutWater_OK;     //��ˮ���
    uint8_t u8StartDelay;    //��ˮ��ʼ��ʱ
    uint8_t u8CloseDelay;    //��ˮ������ʱ
    uint16_t TH_Check_Interval;
    uint8_t OutWater_Key;     //������ˮ
    uint16_t OutWater_Delay;  //������ˮ��ʱ
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
    uint16_t u16TH_Interal;   //��ʪ�ȼ����
    uint16_t u16Start_Temp;   //��ˮ�����¶�
    uint16_t u16Stop_Temp;    //��ˮֹͣ�¶�
    uint16_t u16TH_Cali[2];   //��ʪ��У��
    uint16_t u16NTC_Cali[6];  // NTCУ��
} TH_inst;

// fan
typedef struct
{
    uint16_t mode;              //���ģʽ
    uint16_t startup_delay;     //������ʱ
    uint16_t stop_delay;        //ͣ����ʱ
    uint16_t cold_start_delay;  //��������ʱ
    uint16_t set_speed;         //�����ʼת��
    uint16_t min_speed;         //��Сת��
    uint16_t max_speed;         //���ת��
} Fan_inst;

typedef struct
{
    uint16_t u16Start_Defrost_Temp;  //��˪�����¶�
    uint16_t u16Stop_Defrost_Temp;   //��˪ֹͣ�¶�
    uint16_t u16Defrost_InDelay;     //��˪�ӳٿ���ʱ��
    uint16_t u16Defrost_Duration;    //��˪����ʱ��
    uint16_t u16Defrost_Max;         //���˪ʱ��
} Defrost_inst;

// compressor
typedef struct
{
    uint16_t u16startup_delay;  //�����ӳ�
    uint16_t u16stop_delay;     //ͣ���ӳ�
    uint16_t u16min_runtime;    //�������ʱ��
    uint16_t u16min_stoptime;   //���ͣ��ʱ��
    uint16_t startup_lowpress_shield;
    Defrost_inst Defrost;  //��˪
} Compressor_inst;

// compressor
typedef struct
{
    uint16_t u16Water_Ctrl;
    uint16_t u16Water_Mode;           //��ˮģʽ
    uint16_t u16Water_Flow;           //��ˮ����
    uint16_t u16HotWater_Temp;        //��ˮ�¶�
    uint16_t u16HotWater_Cali;        //��ˮ�¶�У��
    uint16_t u16CloseUV;              //�ر������
    uint16_t u16ExitWater_Mode;       //ˮԴģʽ
    uint16_t u16Sterilize_Interval;   //ɱ�����
    uint16_t u16Sterilize_Time;       //ɱ��ʱ��
    uint16_t u16StartDelay;           //������ʱ
    uint16_t u16CloseDelay;           //�ر���ʱ
    uint16_t u16FILTER_ELEMENT_Type;  //��о�澯����:1-����L;0-ʱ��h
    uint16_t u16TDS_Cail;             // TDSУ��
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
