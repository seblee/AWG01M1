#ifndef __SYS_STATUS
#define __SYS_STATUS
#include "sys_def.h"

void sys_set_remap_status(uint8_t reg_no, uint8_t sbit_pos, uint8_t bit_action);
uint8_t sys_get_pwr_signal(void);
uint8_t Sys_Get_Storage_Signal(uint8_t u8Type);
uint16_t sys_get_remap_status(uint8_t reg_no, uint8_t rbit_pos);
void sys_running_mode_update(void);
uint16_t sys_get_pwr_sts(void);
uint8_t sys_get_di_sts(uint8_t din_channel);
void sys_option_di_sts(uint8_t din_channel, uint8_t option);
uint8_t sys_get_do_sts(uint8_t dout_channel);
uint8_t sys_get_mbm_online(uint8_t mbm_dev);
uint16_t devinfo_get_compressor_cnt(void);
uint16_t Get_Water_level(uint8_t u8Type);
extern uint16_t Get_UV_Status(void);
extern uint16_t Get_Filiter_Status(uint8_t u8Type);
#endif //	__SYS_CONF
