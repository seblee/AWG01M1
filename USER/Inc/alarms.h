#ifndef __ALAMRS_H__
#define __ALAMRS_H__

#include "sys_conf.h"

#define ACL_INACTIVE 0
#define ACL_PREACTIVE 1
#define ACL_ACTIVE 2
#define ACL_POSTACTIVE 3

#define ACL_ENABLE 0
#define ACL_SUPPRESS 1
#define ACL_DISABLE 2

//alarm acl def
enum
{
    //其他
    ACL_E0 = 0, //无出水告警
    ACL_E1,
    ACL_E2,
    ACL_E3,
    ACL_E4, //
    ACL_E5, //
    ACL_E6,
    ACL_E7,//紫外灯杀菌未开UV1
    ACL_E8, //压缩机
	  ACL_E9, //滤芯1
    ACL_E10,
    ACL_E11, //
    ACL_E12, //
    ACL_E13, //滤网
    //异常
    ACL_TOTAL_NUM,

};


#define ACL_WATER_LEAK		 					ACL_E2
#define ACL_FAN01_OD 								ACL_E6 //风机故障
#define ACL_FILLTER_ELEMENT_0		 		ACL_E9
#define ACL_FILLTER_ELEMENT_1		 		ACL_E10
#define ACL_FILLTER_ELEMENT_2		 		ACL_E11
#define ACL_FILLTER_ELEMENT_3		 		ACL_E12
#define ACL_FILLTER		 							ACL_E13

#define ACL_FRONT_ELEMENT		 		ACL_FILLTER_ELEMENT_0
#define ACL_BACK_ELEMENT		 		ACL_FILLTER_ELEMENT_1
#define ACL_RO_ELEMENT		 		  ACL_FILLTER_ELEMENT_2
#define ACL_CTR_ELEMENT		 		  ACL_FILLTER_ELEMENT_3

#define RT_MS 	1000

//报警记录

#define CRITICAL_ALARM_lEVEL 0x00 //严重告警
#define MAJOR_ALARM_LEVEL 0x01    //一般告警
#define MIOOR_ALARM_LEVEL 0x02    //提示告警

#define ALARM_FIFO_DEPTH 30

#define ALARM_STATUS_LEN (sizeof(alram_node) - 4)

#define ALARM_RECORD_LEN sizeof(alarm_log_st)
	
#define ALARM_TOTAL_WORD 	1	//(ACL_TOTAL_NUM/15+1)
	
//报警记录结构体
typedef struct
{
    uint16_t alarm_id;
    uint32_t trigger_time;
    uint32_t end_time;
    uint16_t rev;
} alarm_log_st;

typedef struct node
{
    uint32_t trigger_time;
    uint16_t alarm_id;
    uint16_t alarm_value;
    struct node *next;
} alram_node;

void alarm_acl_init(void);
void alarm_acl_exe(void);

void In_alarm_stats(void);

uint8_t get_alarm_bitmap(uint8_t alarm_id);

uint8_t clear_alarm(uint8_t alarm_id);
uint8_t get_alarm_bitmap_mask(uint8_t component_bpos);
uint8_t get_alarm_bitmap_op(uint8_t component_bpos);
uint8_t alarm_Off(void);
extern uint8_t Get_alarm_arbiration(void);

#endif //__ALAMRS_H__
