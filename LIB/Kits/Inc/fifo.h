#ifndef __FIFO_H
#define	__FIFO_H

#include "stdint.h"

typedef struct fifo16_cb{
	uint16_t* buffer_ptr;
	volatile  uint16_t* bhead_ptr;
	volatile  uint16_t* btail_ptr;
	uint16_t	block_size;
	uint16_t	depth;
	volatile  uint16_t	length;	
}fifo16_cb_td;

typedef struct fifo8_cb{
	uint8_t* buffer_ptr;
	volatile uint8_t* bhead_ptr;
	volatile uint8_t* btail_ptr;
	uint16_t	block_size;
	uint16_t	depth;
	volatile uint16_t	length;	
}fifo8_cb_td;

uint8_t fifo16_init(volatile fifo16_cb_td* fifo16_cb_ptr, uint16_t block_size, uint16_t depth);
uint8_t	fifo16_push(volatile fifo16_cb_td* fifo_cb, uint16_t* src_addr);
uint8_t	fifo16_pop(volatile fifo16_cb_td* fifo_cb, uint16_t* dest_addr);
void fifo16_reset(volatile fifo16_cb_td* fifo_cb);
uint8_t	is_fifo16_empty(volatile fifo16_cb_td* fifo_cb);
uint8_t	is_fifo16_full(volatile fifo16_cb_td* fifo_cb);

uint8_t fifo8_init(volatile fifo8_cb_td* fifo16_cb_ptr, uint16_t block_size, uint16_t depth);
uint8_t	fifo8_push(volatile fifo8_cb_td* fifo_cb, uint8_t* src_addr);
uint8_t	fifo8_pop(volatile fifo8_cb_td* fifo_cb, uint8_t* dest_addr);
void fifo8_reset(volatile fifo8_cb_td* fifo_cb);
uint16_t	get_fifo8_length(volatile fifo8_cb_td* fifo_cb);
uint8_t	is_fifo8_empty(volatile fifo8_cb_td* fifo_cb);
uint8_t	is_fifo8_full(volatile fifo8_cb_td* fifo_cb);

extern volatile fifo8_cb_td fifo_rx_buf;

#endif/*__FIFO_H*/

