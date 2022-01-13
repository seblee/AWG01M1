#ifndef __IIC_H
#define __IIC_H

#include "stdint.h"
//#include "sys_conf.h"
#include "sys_def.h"

// extern sys_reg_st					g_sys;
//#define ALARM_MAX_CNT 500//报警类记录总条数
//#define HUM_TEMP_MAX_CNT  48

#define STS_EE_ADDR 0x0000
#define CONF_REG_EE1_ADDR 0x0010
#define CONF_REG_EE2_ADDR (CONF_REG_EE1_ADDR + 2 * (CONF_REG_MAP_NUM + 1))  // CONF_REG_MAP_NUM+1 //Reg+check
#define CONF_REG_EE3_ADDR (CONF_REG_EE2_ADDR + 2 * (CONF_REG_MAP_NUM + 1))  // CONF_REG_MAP_NUM+1 //Reg+check

#define CONF_REG_FACT_ADDR (CONF_REG_EE3_ADDR + 2 * (CONF_REG_MAP_NUM + 1))  // CONF_REG_MAP_NUM+1 //Reg+check
#define STS_REG_EE1_ADDR (CONF_REG_FACT_ADDR + 2 * (CONF_REG_MAP_NUM + 1))   // CONF_REG_MAP_NUM+1 //Reg+check

#define STS_REG_EE2_ADDR (STS_REG_EE1_ADDR + 200)
#define E2PROM 1  //有EEPROM
//#define AT24C08			1

//驱动底层
#ifdef AT24C08
#define SLAVE_ADDR 0xa8
#define I2C2_EE_PageSize 16  // AT24C08
#else
#define SLAVE_ADDR 0xae
#define I2C2_EE_PageSize 64  // AT24C256
#endif

#define II_SCL_GPIO GPIOB

#define II_SDA_GPIO GPIOB

#define II_WP_GPIO GPIOB

#define II_SCL_Pin GPIO_PIN_8

#define II_SDA_Pin GPIO_PIN_9

#define II_WP_Pin GPIO_PIN_5

#define READ_SDA() GPIO_ReadInputDataBit(II_SDA_GPIO, II_SDA_Pin)

#define WP_Enable() GPIO_ResetBits(II_WP_GPIO, II_WP_Pin)

#define WP_Diable() GPIO_SetBits(II_WP_GPIO, II_WP_Pin)

// IIC?ùóD2ù×÷oˉêy

enum
{
    EEPROM_NOERRO = 0,
    EEPROM_BUSSERRO,
    EEPROM_MUXERRO,
};

void drv_i2c_init(void);
int8_t I2C_EE_BufWrite(uint8_t* write_buffer, uint16_t write_addr, uint16_t num_byte_write);
// int8_t I2C_EE_PageWrite(uint8_t* pBuffer, uint16_t WriteAddr, uint16_t NumByteToWrite);
int8_t I2C_EE_BufRead(uint8_t* read_buffer, uint16_t read_addr, uint16_t num_byte_read);
// void I2C_EE_WaitEepromStandbyState(void);

#endif
