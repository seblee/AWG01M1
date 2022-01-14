#ifndef __DRV_DIO_H
#define __DRV_DIO_H

#include "stm32g0xx_hal.h"
#include "macro.h"

#define DI_BUF_DEPTH 50
#define DI_MAX_CNT 4

#define DI_MASK 0x0F
#define DI_POLARITY 0x0F

typedef struct
{
    uint16_t bitmap;
    uint16_t reg_array[DI_BUF_DEPTH];
} di_dev_st;

typedef struct
{
    uint16_t bitmap;
} do_dev_st;

typedef struct
{
    di_dev_st din;
    do_dev_st dout;
} dio_dev_st;

typedef struct
{
    uint16_t pin_id;
    void* pin_base;
} pin_map_st;

#define RSTORE 1

#define RST_PIN GPIO_PIN_13
#define RST_GPIO GPIOC
#define RST_READ GPIO_ReadInputDataBit(RST_GPIO, RST_PIN)

#define OE_GPIO_PORT GPIOC
#define OE_GPIO_CLK RCC_AHBPeriph_GPIOC  // G
#define OE_GPIO_PIN GPIO_PIN_14

#define STCP_GPIO_PORT GPIOC
#define STCP_GPIO_CLK RCC_AHBPeriph_GPIOC  // RCK
#define STCP_GPIO_PIN GPIO_PIN_15

#define SHCP_GPIO_PORT GPIOF
#define SHCP_GPIO_CLK RCC_AHBPeriph_GPIOF  // SCK
#define SHCP_GPIO_PIN GPIO_PIN_0

#define DS_GPIO_PORT GPIOF
#define DS_GPIO_CLK RCC_AHBPeriph_GPIOF  // SI
#define DS_GPIO_PIN GPIO_PIN_1

#define HC595_OE_Low() GPIO_ResetBits(OE_GPIO_PORT, OE_GPIO_PIN)
#define HC595_OE_High() GPIO_SetBits(OE_GPIO_PORT, OE_GPIO_PIN)
#define HC595_SHCP_Low() GPIO_ResetBits(SHCP_GPIO_PORT, SHCP_GPIO_PIN)
#define HC595_SHCP_High() GPIO_SetBits(SHCP_GPIO_PORT, SHCP_GPIO_PIN)
#define HC595_STCP_Low() GPIO_ResetBits(STCP_GPIO_PORT, STCP_GPIO_PIN)
#define HC595_STCP_High() GPIO_SetBits(STCP_GPIO_PORT, STCP_GPIO_PIN)
#define HC595_Data_Low() GPIO_ResetBits(DS_GPIO_PORT, DS_GPIO_PIN)
#define HC595_Data_High() GPIO_SetBits(DS_GPIO_PORT, DS_GPIO_PIN)

enum
{
    Rep_DI = 0,
    Rep_DO,
    Rep_AI,
    Rep_AO,
    Rep_MAX,
};

typedef struct
{
    uint8_t u8Mx_Bit;  // M3-STM32F103RCT6,M2-STM32F103VCT6;
    uint8_t u8M1_Bit;  // STM32F103VCT6;
} Bit_remap_st;

enum
{
    DI_WATER_LEAK_BPOS = 0,
    DI_KeyWater_BPOS,
    DI_RESERVE_01,
    DI_LED_UV2_BPOS,
    DI_MAX_BPOS,
};

#define DI_Cold_1_BPOS DI_KeyWater_BPOS

extern void Drv_DIO_Init(void);
extern void DIO_reg_init(void);
extern void DI_sts_update(void);
extern void Sync_Di_timeout(void);
extern void DI_reg_update(void);
extern uint8_t GetSEL(void);

extern uint16_t do_set(int16_t pin_id, GPIO_PinState value);
extern void HC595_GPIO_Config(void);
extern void HC595_Send_Multi_Byte(u8* data, u8 len);

extern void oc_update(void);
#endif
