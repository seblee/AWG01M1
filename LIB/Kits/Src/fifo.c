#include "cmsis_os.h"  
#include "stm32g0xx_hal.h"
#include "stdlib.h"
#include "string.h"
#include "fifo.h"

uint8_t fifo16_init(volatile fifo16_cb_td* fifo_cb, uint16_t block_size, uint16_t depth)
{
	uint16_t *mem_ptr;
	mem_ptr = (uint16_t*)malloc(block_size*depth*2);
	if(mem_ptr == NULL)
		return 0;
	fifo_cb->buffer_ptr = mem_ptr;
	fifo_cb->bhead_ptr = mem_ptr;
	fifo_cb->btail_ptr = mem_ptr;
	fifo_cb->block_size = block_size;
	fifo_cb->depth = depth;
	fifo_cb->length = 0;
	return 1;
}

uint8_t	is_fifo16_full(volatile fifo16_cb_td* fifo_cb)
{
	if(fifo_cb->length >= fifo_cb->depth)
		return 1;
	else
		return 0;
}

uint8_t	is_fifo16_empty(volatile fifo16_cb_td* fifo_cb)
{
	if(fifo_cb->length == 0)
		return 1;
	else
		return 0;
}

uint8_t	fifo16_push(volatile fifo16_cb_td* fifo_cb, uint16_t* src_addr)
{
	if(is_fifo16_full(fifo_cb)==1)
		return 0;
	else
	{
		memcpy((void*)fifo_cb->bhead_ptr, src_addr, fifo_cb->block_size*2);
		if(fifo_cb->bhead_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)*2))
			fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
		else
			fifo_cb->bhead_ptr += fifo_cb->block_size*2;
		fifo_cb->length ++;
		return 1;
	}	
}


uint8_t	fifo16_pop(volatile fifo16_cb_td* fifo_cb, uint16_t* dest_addr)
{
	if(is_fifo16_empty(fifo_cb)==1)
		return 0;
	else
	{
		memcpy(dest_addr, (void*)fifo_cb->btail_ptr, fifo_cb->block_size*2);
		if(fifo_cb->btail_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)*2))
			fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
		else
			fifo_cb->btail_ptr += fifo_cb->block_size*2;
		fifo_cb->length --;
		return 1;
	}	
}

void	fifo16_reset(volatile fifo16_cb_td* fifo_cb)
{
	fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
	fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
	fifo_cb->length = 0;
}

uint8_t fifo8_init(volatile fifo8_cb_td* fifo_cb, uint16_t block_size, uint16_t depth)
{
	uint8_t *mem_ptr;
	mem_ptr = (uint8_t*)malloc(block_size*depth);
	if(mem_ptr == NULL)
		return 0;
	fifo_cb->buffer_ptr = mem_ptr;
	fifo_cb->bhead_ptr = mem_ptr;
	fifo_cb->btail_ptr = mem_ptr;
	fifo_cb->block_size = block_size;
	fifo_cb->depth = depth;
	fifo_cb->length = 0;
	return 1;
}

uint8_t	is_fifo8_full(volatile fifo8_cb_td* fifo_cb)
{
	if(fifo_cb->length >= fifo_cb->depth)
		return 1;
	else
		return 0;
}

uint8_t	is_fifo8_empty(volatile fifo8_cb_td* fifo_cb)
{
	if(fifo_cb->length == 0)
		return 1;
	else
		return 0;
}

uint16_t	get_fifo8_length(volatile fifo8_cb_td* fifo_cb)
{
		return fifo_cb->length;
}

uint8_t	fifo8_push(volatile fifo8_cb_td* fifo_cb, uint8_t* src_addr)
{
	if(is_fifo8_full(fifo_cb)==1)
		return 0;
	else
	{
		memcpy((void*)fifo_cb->bhead_ptr, src_addr, fifo_cb->block_size);
		if(fifo_cb->bhead_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
			fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
		else
			fifo_cb->bhead_ptr += fifo_cb->block_size;
		fifo_cb->length ++;
		return 1;
	}	
}


uint8_t	fifo8_pop(volatile fifo8_cb_td* fifo_cb, uint8_t* dest_addr)
{
	if(is_fifo8_empty(fifo_cb)==1)
		return 0;
	else
	{
		memcpy(dest_addr, (void*)fifo_cb->btail_ptr, fifo_cb->block_size);
		if(fifo_cb->btail_ptr >= (fifo_cb->buffer_ptr + fifo_cb->block_size*(fifo_cb->depth-1)))
			fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
		else
			fifo_cb->btail_ptr += fifo_cb->block_size;
		fifo_cb->length --;
		return 1;
	}	
}

void	fifo8_reset(volatile fifo8_cb_td* fifo_cb)
{
	fifo_cb->bhead_ptr = fifo_cb->buffer_ptr;
	fifo_cb->btail_ptr = fifo_cb->buffer_ptr;
	fifo_cb->length = 0;
}
