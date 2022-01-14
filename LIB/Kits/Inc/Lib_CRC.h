#ifndef __LIB_CRC_H
#define	__LIB_CRC_H

#include "macro.h"

extern INT16U CRC16(INT8U *puchMsg, INT16U usDataLen);
extern void Cal_16CRC(INT8U *pSrc, INT8U u16Length,INT8U *pDest);
extern uint8_t checksum_u8(uint8_t* data_ptr, uint16_t data_num);
extern uint16_t checksum_u16(uint16_t* data_ptr, uint16_t data_num);
extern uint8_t xor_checksum(uint8_t* data_ptr, uint16_t data_num);
#endif

