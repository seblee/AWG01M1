#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "threads.h" 
#include "global.h"
#include "calc.h"
#include "sys_conf.h"
#include "req_execution.h"
#include "Drv_DIO.h"
#include "sys_status.h"
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "alarms.h"

//#define ACL_ENMODE_ENABLE	0x0000
//#define ACL_ENMODE_SUPPRESS	0x0001
//#define ACL_ENMODE_DISABLE	0x0002
//enable_mask
enum
{
    ACL_ENMODE_ENABLE = 0, //使能
    ACL_ENMODE_SUPPRESS,   //阻塞
    ACL_ENMODE_DISABLE,    //禁止
    ACL_ENMODE_OTHER,      //其他
};

#define ACL_ENMODE_AUTO_RESET_ALARM 0X0004
#define ACL_ENMODE_HAND_RESET_ALARM 0X0000

#define MBM_DEV_STS_DEFAULT 0x8000

#define ACL_TEM_MAX 1400 //0.1℃
#define ACL_TEM_MIN -280

#define ACL_HUM_MAX 1000 // 0.1%
#define ACL_HUM_MIN 0

#define IO_CLOSE 1
#define IO_OPEN 0

#define OVER_TIME_ACCURACY 86400 //DATE

#define POWER_PARAM 0xFF80
#define POWER_DOT 0x7f
#define POWER_DOT_BIT 7

typedef struct alarm_acl_td
{
    uint16_t id;
    uint16_t state;
    uint16_t alram_value;
    uint16_t timeout;
    uint16_t enable_mask;
    uint16_t reset_mask;
    uint8_t alarm_level;
    uint16_t dev_type;
    uint16_t (*alarm_proc)(struct alarm_acl_td *str_ptr);

} alarm_acl_status_st;

typedef enum
{
    SML_MIN_TYPE = 0,
    THR_MAX_TYPE,
    OUT_MIN_MAX_TYPE,
    IN_MIN_MAX_TYPE,
} Compare_type_st;

typedef struct
{
    uint32_t lock_time[3]; //

    uint16_t last_state;
    char lock_flag;

} alarm_lock_st;

typedef struct
{
    char cycle_flag;         //短周期触发标志。
    uint16_t compress_state; //1\2压缩机状态。
    uint32_t start_time[10]; //压缩机1、2的启动时间
    uint16_t alarm_timer;
} compress_cycle_alarm_st;

#define MAX_LOCK_CNT 6
typedef struct
{

    uint8_t compress_high_flag[2]; //0表示压缩机1高压报警 1表示压缩机二高压告警
    uint16_t tem_sensor_fault;
    uint16_t hum_sensor_fault;
    uint32_t fan_timer;         // 1second++
    uint32_t compressor1_timer; //1second++
    uint32_t compressor2_timer; //1second++
    alarm_lock_st alarm_lock[MAX_LOCK_CNT];
    /*
	{0xffffffff,0xffffffff,0xffffffff},//ACL_HI_LOCK1                  0
	{0xffffffff,0xffffffff,0xffffffff},//ACL_HI_LOCK2                 1
	{0xffffffff,0xffffffff,0xffffffff},//ACL_LO_LOCK1                  2
	{0xffffffff,0xffffffff,0xffffffff},//ACL_LO_LOCK2                  3
	{0xffffffff,0xffffffff,0xffffffff},//ACL_EXTMP_LOCK1               4
	{0xffffffff,0xffffffff,0xffffffff},//ACL_EXTMP_LOCK2               5

*/
    compress_cycle_alarm_st cmpress_cycle_alarm[2];
    alarm_acl_status_st alarm_sts[ACL_TOTAL_NUM];

} alarm_st;

// alarm status chian
static alram_node head_node;

static alarm_st alarm_inst;

extern local_reg_st l_sys;
static uint16_t io_calc(uint8_t data, uint8_t refer);
static uint16_t compare_calc(int16_t meter, int16_t min, int16_t max, Compare_type_st type);

//static uint16_t alarm_lock(uint16_t alarm_id);

static void alarm_status_bitmap_op(uint8_t alarm_id, uint8_t option);

//检测函数
static uint16_t acl00(alarm_acl_status_st *acl_ptr);
static uint16_t acl01(alarm_acl_status_st *acl_ptr);
static uint16_t acl02(alarm_acl_status_st *acl_ptr);
static uint16_t acl03(alarm_acl_status_st *acl_ptr);
static uint16_t acl04(alarm_acl_status_st *acl_ptr);
static uint16_t acl05(alarm_acl_status_st *acl_ptr);
static uint16_t acl06(alarm_acl_status_st *acl_ptr);
static uint16_t acl07(alarm_acl_status_st *acl_ptr);
static uint16_t acl08(alarm_acl_status_st *acl_ptr);
static uint16_t acl09(alarm_acl_status_st *acl_ptr);
static uint16_t acl10(alarm_acl_status_st *acl_ptr);
static uint16_t acl11(alarm_acl_status_st *acl_ptr);
static uint16_t acl12(alarm_acl_status_st *acl_ptr);
static uint16_t acl13(alarm_acl_status_st *acl_ptr);

//告警输出仲裁
static void alarm_arbiration(void);
static void alarm_bitmap_op(uint8_t component_bpos, uint8_t action);
static void alarm_bitmap_mask_op(uint8_t component_bpos, uint8_t action);

enum
{
    ALARM_ACL_ID_POS = 0,  //告警序号
    ALARM_ACL_EN_MASK_POS, //继电器控制
    ALARM_ACL_RESET_POS,   //解除方式
    AlARM_ACL_LEVEL_POS,   //告警等级
    ALARM_ACL_DEV_POS,     //告警类型
    ALARM_ACL_MAX
};

#define ALARM_FSM_INACTIVE 0x0001
#define ALARM_FSM_PREACTIVE 0x0002
#define ALARM_FSM_ACTIVE 0x0003
#define ALARM_FSM_POSTACTIVE 0x0004
#define ALARM_FSM_ERROR 0x0005

#define ALARM_ACL_TRIGGERED 0x0001
#define ALARM_ACL_CLEARED 0x0000
#define ALARM_ACL_HOLD 0x0002

//uint16_t alarm_tem_erro,alarm_hum_erro;

static uint16_t (*acl[ACL_TOTAL_NUM])(alarm_acl_status_st *) =
    {

        //回风和送风报警(温度和湿度)
        acl00, //		ACL_E0
        acl01, //		ACL_E1
        acl02, //		ACL_E2
        acl03, //		ACL_E3
        acl04, //
        acl05, //
        acl06, //
        acl07, //
        acl08, //
        acl09, //
        acl10, //
        acl11, //
        acl12, //
        acl13, //
};

#define DEV_TYPE_COMPRESSOR 0x0000
#define DEV_TYPE_FAN 0x0400
#define DEV_TYPE_OUT_FAN 0x0800
#define DEV_TYPE_HEATER 0x0c00
#define DEV_TYPE_HUM 0x1000
#define DEV_TYPE_POWER 0x1400
#define DEV_TYPE_TEM_SENSOR 0x1800
#define DEV_TYPE_WATER_PUMP 0x1c00
#define DEV_TYPE_OTHER 0x3c00

const uint16_t ACL_CONF[ACL_TOTAL_NUM][ALARM_ACL_MAX] =
    //	id ,en_mask,reless_mask,DEV_type
    {
        0, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E0
        1, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E1
        2, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E2
        3, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E3
        4, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E4
        5, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E5
        6, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E6
        7, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E7
        8, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E8
        9, ACL_ENMODE_OTHER, ACL_ENMODE_HAND_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E9
        10, ACL_ENMODE_OTHER, ACL_ENMODE_HAND_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER, ////E10
        11, ACL_ENMODE_OTHER, ACL_ENMODE_AUTO_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E11
        12, ACL_ENMODE_OTHER, ACL_ENMODE_HAND_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E12
        13, ACL_ENMODE_OTHER, ACL_ENMODE_HAND_RESET_ALARM, CRITICAL_ALARM_lEVEL, DEV_TYPE_OTHER,  //ACL_E13
};
/*
  * @brief  alarm data structure initialization
	* @param  none
  * @retval none
  */

static void init_alarm(alarm_st *alarm_spr)
{
    uint16_t i;

    //初始ACL
    for (i = 0; i < ACL_TOTAL_NUM; i++)
    {

        alarm_spr->alarm_sts[i].timeout = 0;
        alarm_spr->alarm_sts[i].state = ALARM_FSM_INACTIVE;
        alarm_spr->alarm_sts[i].id = ACL_CONF[i][ALARM_ACL_ID_POS];
        alarm_spr->alarm_sts[i].enable_mask = ACL_CONF[i][ALARM_ACL_EN_MASK_POS];
        alarm_spr->alarm_sts[i].reset_mask = ACL_CONF[i][ALARM_ACL_RESET_POS];
        alarm_spr->alarm_sts[i].alarm_level = ACL_CONF[i][AlARM_ACL_LEVEL_POS];
        alarm_spr->alarm_sts[i].dev_type = ACL_CONF[i][ALARM_ACL_DEV_POS];
        alarm_spr->alarm_sts[i].alarm_proc = acl[i];
        alarm_spr->alarm_sts[i].alram_value = 0xffff;
    }

    //初始化lock类变量
    for (i = 0; i < MAX_LOCK_CNT; i++)
    {
        alarm_spr->alarm_lock[i].last_state = ALARM_FSM_INACTIVE;
        alarm_spr->alarm_lock[i].lock_flag = 0;
        alarm_spr->alarm_lock[i].lock_time[0] = 0xffffffff;
        alarm_spr->alarm_lock[i].lock_time[1] = 0xffffffff;
        alarm_spr->alarm_lock[i].lock_time[2] = 0xffffffff;
    }
    //初始化压缩机类 短周期
    //alarm_spr->cmpress_cycle_alarm[0].alarm_timer
    alarm_spr->cmpress_cycle_alarm[0].cycle_flag = 0;
    alarm_spr->cmpress_cycle_alarm[1].cycle_flag = 0;

    alarm_spr->cmpress_cycle_alarm[0].compress_state = COMPRESSOR_FSM_STATE_IDLE;
    alarm_spr->cmpress_cycle_alarm[1].compress_state = COMPRESSOR_FSM_STATE_IDLE;

    for (i = 0; i < 10; i++)
    {
        alarm_spr->cmpress_cycle_alarm[0].start_time[i] = 0xffffffff;
        alarm_spr->cmpress_cycle_alarm[1].start_time[i] = 0xffffffff;
    }
    //高压触发报警初始化
    alarm_spr->compress_high_flag[0] = 0;
    alarm_spr->compress_high_flag[1] = 0;
    //初始化启动定时器
    alarm_spr->fan_timer = 0;
    alarm_spr->compressor1_timer = 0;
    alarm_spr->compressor2_timer = 0;
}

void chain_init(void)
{

    //ACL_TOTAL_NUM*sizeof(alram_node)
    //sizeof(alram_node)
    //初始化内块

    head_node.next = NULL;
    head_node.alarm_value = 0; //报警计数
}
void alarm_acl_init(void)
{
    uint8_t i;
    // 初始化静态内存分配空间
    chain_init();
    init_alarm(&alarm_inst);
    //初始化手动解除报警
    for (i = 0; i < ALARM_TOTAL_WORD; i++)
    {
        g_sVariable.gPara.alarm_remove_bitmap[i] = 0;
    }
}


uint8_t clear_alarm(uint8_t alarm_id)
{
		uint8_t byte_offset,bit_offset,i;

		if(alarm_id == 0xFF)
		{
					for(i=0;i<ALARM_TOTAL_WORD;i++)
					{
						g_sVariable.gPara.alarm_remove_bitmap[i] = 0xffff;
					}

				return(1);
		}
		else
		if(alarm_id < ACL_TOTAL_NUM)
		{
				byte_offset = alarm_id >> 4;
				bit_offset = alarm_id & 0x0f;
				g_sVariable.gPara.alarm_remove_bitmap[byte_offset] |= 0x0001<< bit_offset;
			  return(1);
		}
		else
		{
				return(0);
		}
}

static uint8_t get_alarm_remove_bitmap(uint8_t alarm_id)
{
    uint8_t byte_offset, bit_offset;
    if (alarm_id < ACL_TOTAL_NUM)
    {

        byte_offset = alarm_id >> 4;
        bit_offset = alarm_id & 0x0f;
        if ((g_sVariable.gPara.alarm_remove_bitmap[byte_offset] >> bit_offset) & 0x0001)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }
    else
    {
        return (0);
    }
}

static void clear_alarm_remove_bitmap(uint8_t alarm_id)
{
    uint8_t byte_offset, bit_offset;

    if (alarm_id < ACL_TOTAL_NUM)
    {
        byte_offset = alarm_id >> 4;
        bit_offset = alarm_id & 0x0f;
        g_sVariable.gPara.alarm_remove_bitmap[byte_offset] = g_sVariable.gPara.alarm_remove_bitmap[byte_offset] & (~(0x0001 << bit_offset));
    }
}

uint16_t Alarm_acl_delay(uint8_t ACL_Num)
{
    uint16_t ACL_Delay;

    switch (ACL_Num)
    {
    case (ACL_WATER_LEAK):
        {
            ACL_Delay = 10;
            break;
        }
    case (ACL_FILLTER_ELEMENT_0):
    case (ACL_FILLTER_ELEMENT_1):
    case (ACL_FILLTER_ELEMENT_2):
    case (ACL_FILLTER_ELEMENT_3):
    case (ACL_FILLTER):
        {
            ACL_Delay = 10;
            break;
        }
    default:
    {
        ACL_Delay = 5;
        break;
    }
    }
    return ACL_Delay;
}

void alarm_acl_exe(void)
{
    uint16_t acl_trigger_state;
    uint16_t i;
    uint16_t c_state;
    uint16_t log_id;

    //		acl_power_on_timer();

    for (i = 0; i < ACL_TOTAL_NUM; i++)
    {
        //if acl disabled, continue loop

        g_sVariable.gPara.alarm[i].enable_mode = ACL_ENMODE_AUTO_RESET_ALARM;
        if (((g_sVariable.gPara.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE) && (alarm_inst.alarm_sts[i].state == ALARM_FSM_INACTIVE))
        {
            continue;
        }

        acl_trigger_state = acl[i](&alarm_inst.alarm_sts[i]);
        c_state = alarm_inst.alarm_sts[i].state;
        log_id = alarm_inst.alarm_sts[i].id | (alarm_inst.alarm_sts[i].alarm_level << 8) | alarm_inst.alarm_sts[i].dev_type;
        switch (c_state)
        {
        case (ALARM_FSM_INACTIVE):
        {
            if (acl_trigger_state == ALARM_ACL_TRIGGERED)
            {
                //										alarm_inst.alarm_sts[i].timeout = g_sVariable.gPara.alarm[i].delay;
                alarm_inst.alarm_sts[i].timeout = Alarm_acl_delay(i);
                alarm_inst.alarm_sts[i].state = ALARM_FSM_PREACTIVE;
            }
            else if (acl_trigger_state == ALARM_ACL_HOLD)
            {
                alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
            }
            else
            {
                ;
            }

            break;
        }
        case (ALARM_FSM_PREACTIVE):
        {
            //状态机回到 ALARM_FSM_INACTIVE 状态
            if ((g_sVariable.gPara.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)
            {
                alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
            }
            else if (acl_trigger_state == ALARM_ACL_TRIGGERED)
            {

                if (alarm_inst.alarm_sts[i].timeout > 0)
                {
                    alarm_inst.alarm_sts[i].timeout--;
                    alarm_inst.alarm_sts[i].state = ALARM_FSM_PREACTIVE;
                }
                else
                {
                    alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
                    //XDP
//                    add_alarmlog_fifo(log_id, ALARM_TRIGER, alarm_inst.alarm_sts[i].alram_value);
                    alarm_status_bitmap_op(alarm_inst.alarm_sts[i].id & 0x00ff, 1);
//                    node_append(log_id, alarm_inst.alarm_sts[i].alram_value);
                }
            }
            else if (acl_trigger_state == ALARM_ACL_HOLD)
            {
                ;
            }
            else
            {
                alarm_inst.alarm_sts[i].timeout = 0;
                alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
            }
            break;
        }
        case (ALARM_FSM_ACTIVE):
        {
            //状态机回到 ALARM_FSM_INACTIVE 状态
            if ((g_sVariable.gPara.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)
            {
                alarm_inst.alarm_sts[i].timeout = 0;
                alarm_inst.alarm_sts[i].state = ALARM_FSM_POSTACTIVE;
            }
            else if (acl_trigger_state == ALARM_ACL_TRIGGERED)
            {
                alarm_inst.alarm_sts[i].timeout = 0;
                alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
            }
            else if (acl_trigger_state == ALARM_ACL_CLEARED)
            {
                //自动解除报警
                if ((g_sVariable.gPara.alarm[i].enable_mode & alarm_inst.alarm_sts[i].reset_mask) == ACL_ENMODE_AUTO_RESET_ALARM)
                {
                    //										alarm_inst.alarm_sts[i].timeout = g_sVariable.gPara.alarm[i].delay;
                    alarm_inst.alarm_sts[i].timeout = Alarm_acl_delay(i);

                    alarm_inst.alarm_sts[i].state = ALARM_FSM_POSTACTIVE;
                }
                else
                {
                    alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
                }
            }
            else if (acl_trigger_state == ALARM_ACL_HOLD)
            {
                ;
            }
            else
            {
                ;
            }
            break;
        }
        case (ALARM_FSM_POSTACTIVE):
        {
            //状态机回到 ALARM_FSM_INACTIVE 状态
            if ((g_sVariable.gPara.alarm[i].enable_mode & alarm_inst.alarm_sts[i].enable_mask) == ACL_ENMODE_DISABLE)
            {
//                add_alarmlog_fifo(log_id, ALARM_END, alarm_inst.alarm_sts[i].alram_value);
//                //删除状态节点
//                node_delete(log_id);
                alarm_status_bitmap_op(alarm_inst.alarm_sts[i].id & 0x00ff, 0);

                alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
                alarm_inst.alarm_sts[i].timeout = 0;
            }
            else if (acl_trigger_state == ALARM_ACL_CLEARED) //yxq
            {

                if (alarm_inst.alarm_sts[i].timeout > 0)
                {
                    alarm_inst.alarm_sts[i].timeout--;
                    alarm_inst.alarm_sts[i].state = ALARM_FSM_POSTACTIVE;
                }
                else
                {
                    //XDP
//                    add_alarmlog_fifo(log_id, ALARM_END, alarm_inst.alarm_sts[i].alram_value);
                    //
                    //												//删除状态节点
                    alarm_status_bitmap_op(i, 0);
//                    node_delete(log_id);

                    alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;
                }
            }
            else if (acl_trigger_state == ALARM_ACL_TRIGGERED)
            {
                alarm_inst.alarm_sts[i].timeout = 0;
                alarm_inst.alarm_sts[i].state = ALARM_FSM_ACTIVE;
            }
            else if (acl_trigger_state == ALARM_ACL_HOLD)
            {
                ;
            }
            else
            {
                ;
            }
            break;
        }
        default: //yxq
        {
            alarm_inst.alarm_sts[i].state = ALARM_FSM_INACTIVE;

            break;
        }
        }
    }

    alarm_arbiration();

}
//获取报警位
uint8_t get_alarm_bitmap(uint8_t alarm_id)
{
    uint8_t byte_offset, bit_offset;

    {
        byte_offset = alarm_id >> 4;
        bit_offset = alarm_id & 0x0f;

        if ((g_sVariable.status.alarm_bitmap[byte_offset] >> bit_offset) & (0x0001))
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }
}

uint8_t Get_alarm_arbiration(void)
{
    uint8_t close_dev = 0;

		if ((get_alarm_bitmap(ACL_FAN01_OD))||(get_alarm_bitmap(ACL_WATER_LEAK))) //风机、漏水		
    {
        close_dev = 1;
    }
    else
    {
        close_dev = 0;
    }
		return close_dev;
}
//告警处理
static void alarm_arbiration(void)
{
    uint8_t compress1_alarm = 0,  close_dev = 0;
    //		uint8_t index;

//    if (get_alarm_bitmap(ACL_HI_PRESS1)) //高压
//    {
//        compress1_alarm = 1;
//    }
//    else
//    {
//        compress1_alarm = 0;
//    }

		if ((get_alarm_bitmap(ACL_FAN01_OD))||(get_alarm_bitmap(ACL_WATER_LEAK))) //风机、漏水		
    {
        close_dev = 1;
    }
    else
    {
        close_dev = 0;
    }

//    //压缩机1的控制
//    if ((close_dev) || (compress1_alarm))
//    {
//        alarm_bitmap_op(DO_COMP1_BPOS, 0);
////        alarm_bitmap_mask_op(DO_COMP1_BPOS, 1);
//    }
//    else
//    {
//        alarm_bitmap_op(DO_COMP1_BPOS, 1);
//        alarm_bitmap_mask_op(DO_COMP1_BPOS, 0);
//    }

//    //风机控制
//    if ((close_dev))
//    {
//        alarm_bitmap_op(DO_FAN_BPOS, 0);
////        alarm_bitmap_mask_op(DO_FAN_BPOS, 1);
//    }
//    else
//    {
//        alarm_bitmap_op(DO_FAN_BPOS, 1);
//        alarm_bitmap_mask_op(DO_FAN_BPOS, 0);
//    }
}

//关机告警处理
uint8_t alarm_Off(void)
{
//    if ((get_alarm_bitmap(ACL_WATER_LEAK))||(get_alarm_bitmap(ACL_SYS01_EXHAUST_HI))||(get_alarm_bitmap(ACL_SYS01_EXHAUST_HI_LOCK))||
//			(get_alarm_bitmap(ACL_E7))||(get_alarm_bitmap(ACL_E9))) 
//		{
//				return TRUE;
//		}
		return FALSE;
}


//
uint8_t get_alarm_bitmap_op(uint8_t component_bpos)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = component_bpos >> 4;
    bit_offset = component_bpos & 0x0f;
    if ((l_sys.bitmap[byte_offset][BITMAP_ALARM] >> bit_offset) & 0x01)
    {
        return (1);
    }
    else
    {
        return (0);
    }
    //		if((l_sys.bitmap[BITMAP_ALARM]>>component_bpos)&0x01)
    //		{
    //				return(1);
    //		}
    //		else
    //		{
    //				return(0);
    //		}
}

uint8_t get_alarm_bitmap_mask(uint8_t component_bpos)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = component_bpos >> 4;
    bit_offset = component_bpos & 0x0f;
    if ((l_sys.bitmap[byte_offset][BITMAP_MASK] >> bit_offset) & 0x01)
    {
        return (1);
    }
    else
    {
        return (0);
    }

    //		if((l_sys.bitmap[BITMAP_MASK] >>component_bpos)&0x01)
    //		{
    //				return(1);
    //		}
    //		else
    //		{
    //				return(0);
    //		}
}

//DO_FAN_BPOS//offset
//报警位操作

static void alarm_bitmap_op(uint8_t component_bpos, uint8_t action)
{
    extern local_reg_st l_sys;
    uint8_t byte_offset, bit_offset;

    byte_offset = component_bpos >> 4;
    bit_offset = component_bpos & 0x0f;
    if (action == 0)
    {
        l_sys.bitmap[byte_offset][BITMAP_ALARM] &= ~(0x0001 << bit_offset);
    }
    else
    {
        l_sys.bitmap[byte_offset][BITMAP_ALARM] |= (0x0001 << bit_offset);
    }

    //		if(action == 0)
    //		{
    //				l_sys.bitmap[BITMAP_ALARM] &= ~(0x0001<<component_bpos);
    //		}
    //		else
    //		{
    //				l_sys.bitmap[BITMAP_ALARM] |= (0x0001<<component_bpos);
    //		}
}

static void alarm_bitmap_mask_op(uint8_t component_bpos, uint8_t action)
{
    extern local_reg_st l_sys;
    uint8_t byte_offset, bit_offset;

    byte_offset = component_bpos >> 4;
    bit_offset = component_bpos & 0x0f;
    if (action == 0)
    {
        l_sys.bitmap[byte_offset][BITMAP_MASK] &= ~(0x0001 << bit_offset);
    }
    else
    {
        l_sys.bitmap[byte_offset][BITMAP_MASK] |= (0x0001 << bit_offset);
    }
    //		if(action == 0)
    //		{
    //				l_sys.bitmap[BITMAP_MASK] &= ~(0x0001<<component_bpos);
    //		}
    //		else
    //		{
    //				l_sys.bitmap[BITMAP_MASK] |= (0x0001<<component_bpos);
    //		}
}

//告警状态位
static void alarm_status_bitmap_op(uint8_t alarm_id, uint8_t option)
{
    uint8_t byte_offset, bit_offset;

    byte_offset = alarm_id >> 4;
    bit_offset = alarm_id & 0x0f;
    if (option == 1)
    {
        g_sVariable.status.alarm_bitmap[byte_offset] |= (0x0001 << bit_offset);
    }
    else
    {
        g_sVariable.status.alarm_bitmap[byte_offset] &= ~(0x0001 << bit_offset);
    }
}

/**
  * @brief  alarm 0 rule check function
	* @param  str_ptr: alarm_acl_st* structure pointer 
  * @retval none
  */

static uint16_t io_calc(uint8_t data, uint8_t refer)
{

    if (data == refer)
    {

        return 1;
    }
    else
    {

        return 0;
    }
}

/**
  * @brief  alarm 1 rule check function
	* @param  str_ptr: alarm_acl_st* structure pointer 
  * @retval none
  */

// 模拟量抱紧检测函数
static uint16_t compare_calc(int16_t meter, int16_t min, int16_t max, Compare_type_st type)
{

    if (type == THR_MAX_TYPE) //大于最大门限触发
    {
        if (meter > max)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }
    else if (type == SML_MIN_TYPE) //小于最小门限触发
    {
        if (meter < min)
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }
    else if (type == IN_MIN_MAX_TYPE) //在区间内报警
    {
        if ((meter > min) && (meter < max))
        {
            return (1);
        }
        else
        {
            return (0);
        }
    }
    else //在区间以外报警
    {
        if ((meter < min) || (meter > max))
        {
            return (1);
        }
        else
        {

            return (0);
        }
    }
}

/**
  * @brief  alarm 2 rule check function
	* @param  str_ptr: alarm_acl_st* structure pointer 
  * @retval none
  */
  //锁死类报警
static uint16_t alarm_lock(uint16_t alarm_id)
{
	uint8_t index=0xff;
	uint64_t	u64Over_lock_time=0; 
	
	switch(alarm_id)
	{
//		case ACL_HI_LOCK1://高压1锁死
//			index = 0;
//		break;
//		case  ACL_HI_LOCK2://高压2锁死
//			index = 1;
//		break;
//		case ACL_SYS01_PRESS_LO_LOCK://低压1锁死
//			index = 2;
//		break;
//		case ACL_SYS02_PRESS_LO_LOCK://低压2锁死
//			index=3;
//		break;
//		case ACL_SYS01_EXHAUST_HI_LOCK://排气温度1锁死
//			index = 4;
//		break;
		default:
			index = 0xff;
		break;
			
	}
	
	if(index != 0xff)
	{
			//解除锁定告警
			if(get_alarm_remove_bitmap(alarm_id) == 1)
			{
					alarm_inst.alarm_lock[index].lock_time[0] = 0xffffffff;
					alarm_inst.alarm_lock[index].lock_time[1] = 0xffffffff;
					alarm_inst.alarm_lock[index].lock_time[2] = 0xffffffff;
					alarm_inst.alarm_lock[index].lock_flag =0;
					
				//	clear_alarm_remove_bitmap(alarm_id);
			}
			
			if(alarm_inst.alarm_lock[index].lock_flag)
			{
					return(ALARM_ACL_TRIGGERED);
			}
			if(alarm_inst.alarm_lock[index].lock_time[0] != 0xffffffff)
			{
					if(alarm_inst.alarm_lock[index].lock_time[2]<alarm_inst.alarm_lock[index].lock_time[0])
					{
							u64Over_lock_time=alarm_inst.alarm_lock[index].lock_time[2]|0xffffffff;
						if((u64Over_lock_time - alarm_inst.alarm_lock[index].lock_time[0]) <= 3600*RT_MS)
						{
								alarm_inst.alarm_lock[index].lock_flag = 1;
								return(ALARM_ACL_TRIGGERED);
						}							
					}
					else
					{
						if((alarm_inst.alarm_lock[index].lock_time[2] - alarm_inst.alarm_lock[index].lock_time[0]) <= 3600*RT_MS)
						{
								alarm_inst.alarm_lock[index].lock_flag = 1;
								return(ALARM_ACL_TRIGGERED);
						}	
					}
			}
   }
	return(ALARM_ACL_CLEARED);
}

static uint8_t acl_clear(alarm_acl_status_st *acl_ptr)
{
    if ((get_alarm_remove_bitmap(acl_ptr->id) == 1) && (acl_ptr->state > ALARM_FSM_PREACTIVE))
    {
        acl_ptr->state = ALARM_FSM_POSTACTIVE;
        acl_ptr->timeout = 0;
        clear_alarm_remove_bitmap(acl_ptr->id);
        return (1);
    }
    clear_alarm_remove_bitmap(acl_ptr->id);
    return (0);
}

//ACL_E0 无出水告警
static uint16_t acl00(alarm_acl_status_st *acl_ptr)
{
    return ALARM_ACL_CLEARED;
}

//ACL_E1 饮水箱水位>上浮球
static uint16_t acl01(alarm_acl_status_st *acl_ptr)
{
//    uint8_t data;
//    uint16_t u16WL;

//    // 解除 报警
//    if (acl_clear(acl_ptr))
//    {
//        return (ALARM_ACL_CLEARED);
//    }
//    //水位
//		u16WL = Get_Water_level(DW_TANK);
//		if ((u16WL >= D_U))
//		{
//				data = 1;				
//		}
//    else
//    {
//        data = 0;
//    }
//    return data;
}

//ACL_E2 源水箱水位>上浮球，漏水
static uint16_t acl02(alarm_acl_status_st *acl_ptr)
{
    uint8_t data;
    uint16_t u16WL;

    // 解除 报警
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
//		//水位
//		u16WL = Get_Water_level(SW_TANK);
//		if ((u16WL >= S_U))
		{
				data = sys_get_di_sts(DI_WATER_LEAK_BPOS);			
		}
//		else
//		{
//				data = 0;
//		}
    return data;
}

//ACL_E3，NTC异常
static uint16_t acl03(alarm_acl_status_st *acl_ptr)
{
    int16_t min;
    int16_t max;
    uint8_t req;
    int16_t meter;

    // 解除 报警
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }

    max = ACL_TEM_MAX;
    min = ACL_TEM_MIN;

    req = 0;

    //NTC
    if ((g_sVariable.gPara.dev_mask.ain) & (0x01 << AI_NTC1))
    {
        meter = g_sVariable.status.u16AI[AI_NTC1];
        if (compare_calc(meter, min, max, OUT_MIN_MAX_TYPE))
        {
            req |= 0x01;
        }
    }
    if ((g_sVariable.gPara.dev_mask.ain) & (0x01 << AI_NTC2))
    {
        meter = g_sVariable.status.u16AI[AI_NTC2];
        if (compare_calc(meter, min, max, OUT_MIN_MAX_TYPE))
        {
            req |= 0x02;
        }

    }
    if ((g_sVariable.gPara.dev_mask.ain) & (0x01 << AI_NTC3))
    {
        meter = g_sVariable.status.u16AI[AI_NTC3];
        if (compare_calc(meter, min, max, OUT_MIN_MAX_TYPE))
        {
            req |= 0x04;
        }
    }

    if ((g_sVariable.gPara.dev_mask.ain) & (0x01 << AI_NTC4))
    {
        meter = g_sVariable.status.u16AI[AI_NTC4];
        if (compare_calc(meter, min, max, OUT_MIN_MAX_TYPE))
        {
            req |= 0x08;
        }
    }
		if(req)
		{
			req=1;
		}
		else
		{
			req=0;			
		}		
		//消除告警
		if(g_sVariable.gPara.dev_mask.ain&0x8000)
		{
				req=0;					
		}
    return (req);
}

// ACL_E4,未制水
static uint16_t acl04(alarm_acl_status_st *acl_ptr)
{
    uint8_t data = 0;
    uint8_t u16WL;
		//水位
		u16WL = Get_Water_level(DW_TANK);
		if ((u16WL < D_U))
		{
			if (sys_get_remap_status(WORK_MODE_STS_REG_NO, PROWATER_STS_BPOS) == 0)	
			{
				data = 1;
			}
		}
		else
		{
				data = 0;
		}
    return data;
}
//ACL_E5 温湿度传感器故障
static uint16_t acl05(alarm_acl_status_st *acl_ptr)
{
    uint8_t req;
    uint16_t u16mb_comp;

    //uint16_t HUM_erro=0;
    req = 0;
    // 解除 报警
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }

    //		u16mb_comp=g_sVariable.gPara.dev_mask.mb_comp;
    u16mb_comp = 0x01;

    if (g_sVariable.status.u16Status_remap[COM_STS_REG_NO] != u16mb_comp)
    {
        req = 1;
    }
    else
    {
        req = 0;
    }
    return (req);
}

//ACL_E6 ,FAN
static uint16_t acl06(alarm_acl_status_st *acl_ptr)
{

    return (ALARM_ACL_CLEARED);
}
//ACL_E7 UV
static uint16_t acl07(alarm_acl_status_st *acl_ptr)
{
    // 解除 报警
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
		Get_UV_Status();
		if(g_sVariable.status.Water.u16UVStatus&UV_ERR)	
		{
     return (ALARM_ACL_TRIGGERED);			
		}
		
    return (ALARM_ACL_CLEARED);
}
//ACL_E8 CP
static uint16_t acl08(alarm_acl_status_st *acl_ptr)
{
     return (ALARM_ACL_CLEARED);
}

//ACL_E9  ACL_FILLTER_ELEMENT_0
static uint16_t acl09(alarm_acl_status_st *acl_ptr)
{
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
		if(Get_Filiter_Status(ACL_FILLTER_ELEMENT_0)==0)
		{
     return (ALARM_ACL_TRIGGERED);
		}
     return (ALARM_ACL_CLEARED);
}
//ACL_E10
static uint16_t acl10(alarm_acl_status_st *acl_ptr)
{
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
		if(Get_Filiter_Status(ACL_FILLTER_ELEMENT_1)==0)
		{
     return (ALARM_ACL_TRIGGERED);
		}
     return (ALARM_ACL_CLEARED);
}
//ACL_E11
static uint16_t acl11(alarm_acl_status_st *acl_ptr)
{
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
		if(Get_Filiter_Status(ACL_FILLTER_ELEMENT_2)==0)
		{
     return (ALARM_ACL_TRIGGERED);
		}
     return (ALARM_ACL_CLEARED);
}

//ACL_E12
static uint16_t acl12(alarm_acl_status_st *acl_ptr)
{
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
		if(Get_Filiter_Status(ACL_FILLTER_ELEMENT_3)==0)
		{
     return (ALARM_ACL_TRIGGERED);
		}
     return (ALARM_ACL_CLEARED);
}

//ACL_E13
static uint16_t acl13(alarm_acl_status_st *acl_ptr)
{
    if (acl_clear(acl_ptr))
    {
        return (ALARM_ACL_CLEARED);
    }
		if(Get_Filiter_Status(ACL_FILLTER)==0)
		{
     return (ALARM_ACL_TRIGGERED);
		}
     return (ALARM_ACL_CLEARED);
}
