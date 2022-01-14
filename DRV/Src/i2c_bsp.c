
#include "stm32g0xx_hal.h"
#include "i2c_bsp.h"
#include "cmsis_os.h"
/**
 * @brief  Initializes peripherals used by the I2C EEPROM driver.
 * @param  None
 * @retval : None
 */
#define I2CSPEED 2
// I2C模拟延时
#define I2CPAGEWriteDelay 5

static void I2C_Config(void)
{
    // Configure I2C1 pins: SCL

    // writ_disable
    HAL_GPIO_WritePin(II_WP_GPIO, II_WP_Pin, GPIO_PIN_SET);
}

static void IIC_SCL(uint8_t n)
{
    if (n == 1)
    {
        HAL_GPIO_WritePin(II_SCL_GPIO, II_SCL_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(II_SCL_GPIO, II_SCL_Pin, GPIO_PIN_RESET);
    }
}

static void IIC_SDA(uint8_t n)
{
    if (n == 1)
    {
        HAL_GPIO_WritePin(II_SDA_GPIO, II_SCL_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(II_SDA_GPIO, II_SCL_Pin, GPIO_PIN_RESET);
    }
}

void drv_i2c_init(void)
{
    I2C_Config();
    return;
}

static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin   = II_SDA_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(II_SDA_GPIO, &GPIO_InitStruct);
}

static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*Configure GPIO pins*/
    GPIO_InitStruct.Pin  = II_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(II_SDA_GPIO, &GPIO_InitStruct);
}

//延时函数有可能需要修改
static void delay_us(uint32_t nus)
{
    uint8_t i;
    uint32_t temp;
    //    for(i=0;i<5;i++)
    for (i = 0; i < 4; i++)
    {
        for (temp = nus; temp != 0; temp--)
        {
            ;
        }
    }
}

/*
void delay_us(uint32_t nus)		//延时函数有可能需要修改
{
    uint32_t tick,delayPeriod;
    tick = osKernelSysTick();
    delayPeriod = osKernelSysTickMicroSec(nus);
    while(osKernelSysTick() - tick < delayPeriod);
}
*/
static void IIC_Start(void)
{
    SDA_OUT();
    IIC_SDA(1);
    IIC_SCL(1);

    delay_us(4);
    IIC_SDA(0);
    // START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL(0);
}
static void IIC_Stop(void)
{
    SDA_OUT();
    IIC_SCL(0);
    IIC_SDA(0);
    // STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL(1);
    IIC_SDA(1);
    delay_us(4);
}
static uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucerrtime = 0;
    SDA_IN();
    IIC_SDA(1);
    delay_us(1);
    IIC_SCL(1);
    delay_us(1);
    while (READ_SDA())
    {
        ucerrtime++;
        if (ucerrtime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL(0);
    return 0;
}
static void IIC_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}
static void IIC_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}
static void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;

    SDA_OUT();
    IIC_SCL(0);
    for (t = 0; t < 8; t++)
    {
        if ((txd & 0x80) >> 7)
            IIC_SDA(1);
        else
            IIC_SDA(0);
        txd <<= 1;
        delay_us(2);
        //对TEA5767这三个延时都是必须的
        IIC_SCL(1);
        delay_us(2);
        IIC_SCL(0);
        delay_us(2);
    }
}
static uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();
    for (i = 0; i < 8; i++)
    {
        IIC_SCL(0);
        delay_us(2);
        IIC_SCL(1);
        receive <<= 1;
        if (READ_SDA())
        {
            receive++;
        }
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}

static int8_t I2c_write_byte(uint8_t data)
{
    IIC_Send_Byte(data);
    if (IIC_Wait_Ack() == 0)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

int8_t WriteEEROMPage(uint8_t* write_buffer, uint16_t write_addr, uint8_t num_byte_write)
{
    uint16_t len;
    WP_Enable();
    IIC_Start();
    __nop();
    IIC_Start();
    // I2C Slave addr
#ifdef AT24C08
    u8EE_Addr = (write_addr >> 8) << 1;
    if (I2c_write_byte(SLAVE_ADDR | u8EE_Addr) == 0)
    {
        IIC_Stop();
        WP_Diable();
        // rt_kprintf("\n IICBSP: I2C_EE_BufWrite  ERR \n");
        return (EEPROM_BUSSERRO);
    }
#else
    if (I2c_write_byte(SLAVE_ADDR & 0xFE) == 0)
    {
        IIC_Stop();
        WP_Diable();
        // rt_kprintf("\n IICBSP: I2C_EE_BufWrite  ERR \n");
        return (EEPROM_BUSSERRO);
    }
    // Data high 8 addr
    if (I2c_write_byte(write_addr >> 8) == 0)
    {
        IIC_Stop();
        WP_Diable();
        //        rt_kprintf("\n IICBSP: WriteEEROMPage high 8\n");
        return (EEPROM_BUSSERRO);
    }
#endif
    // data low 8 addr
    if (I2c_write_byte(write_addr & 0x00ff) == 0)
    {
        IIC_Stop();
        WP_Diable();
        //        rt_kprintf("\n IICBSP: WriteEEROMPage low 8 \n");
        return (EEPROM_BUSSERRO);
    }

    for (len = 0; len < num_byte_write; len++)
    {
        if (I2c_write_byte(*(write_buffer + len)) == 0)
        {
            IIC_Stop();
            WP_Diable();
            //            rt_kprintf("\n IICBSP: WriteEEROMPage len=%d \n", len);
            return (EEPROM_BUSSERRO);
        }
    }
    IIC_Stop();
    WP_Diable();
    delay_us(3000);
    return (EEPROM_NOERRO);
}

int8_t I2C_EE_BufWrite_bsp(uint8_t* write_buffer, uint16_t write_addr, uint16_t num_byte_write)
{
    uint8_t byte_count;
    uint16_t page_number;
    uint8_t cnt;
    // 计算第一次需要写入的长度
    byte_count = I2C2_EE_PageSize - (write_addr % I2C2_EE_PageSize);

    if (num_byte_write <= byte_count)
    {
        byte_count = num_byte_write;
    }
    // 先写入第一页的数据
    if (WriteEEROMPage(write_buffer, write_addr, byte_count) != EEPROM_NOERRO)
    {
        // 写入错误,返回错误
        return EEPROM_BUSSERRO;
    }

    // 重新计算还需要写入的字节
    //页延时

    num_byte_write -= byte_count;
    if (!num_byte_write)
    {
        // 数据已经写入完毕
        return EEPROM_NOERRO;
    }
    write_addr += byte_count;
    write_buffer += byte_count;
    // 计算需要进行的页写次数
    page_number = num_byte_write / I2C2_EE_PageSize;
    // 循环写入数据
    while (page_number--)
    {
        cnt = 10;
        while (cnt)
        {
            //如果写不成功
            if ((WriteEEROMPage(write_buffer, write_addr, I2C2_EE_PageSize)) != EEPROM_NOERRO)
            {
                // 延时
                cnt--;
                delay_us(10);
            }
            //写成功了
            else
            {
                break;
            }
        }
        //检查写失败
        if (cnt == 0)
        {
        }
        num_byte_write -= I2C2_EE_PageSize;
        write_addr += I2C2_EE_PageSize;
        write_buffer += I2C2_EE_PageSize;
    }
    delay_us(10);
    if (!num_byte_write)
    {
        return EEPROM_NOERRO;
    }
    // 写入最后一页的数据
    if ((WriteEEROMPage(write_buffer, write_addr, num_byte_write)) != EEPROM_NOERRO)
    {
        // 写入错误,返回错误
        return EEPROM_BUSSERRO;
    }
    delay_us(3000);
    return EEPROM_NOERRO;
}

int8_t I2C_EE_BufWrite(uint8_t* write_buffer, uint16_t write_addr, uint16_t num_byte_write)
{
    int8_t erro;

    erro = EEPROM_NOERRO;

    erro = I2C_EE_BufWrite_bsp(write_buffer, write_addr, num_byte_write);
    return (erro);
}

int8_t I2C_EE_BufRead_bsp(uint8_t* read_buffer, uint16_t read_addr, uint16_t num_byte_Read)
{
    uint16_t len;
    //获取互斥量
    WP_Enable();
    IIC_Start();
    // I2C Slave addr
#ifdef AT24C08
    u8EE_Addr = (read_addr >> 8) << 1;
    if (I2c_write_byte(SLAVE_ADDR | u8EE_Addr) == 0)
    {
        IIC_Stop();
        WP_Diable();
        // rt_kprintf("\n IICBSP: I2C_EE_BufWrite  ERR \n");
        return (EEPROM_BUSSERRO);
    }
#else
    if (I2c_write_byte(SLAVE_ADDR & 0xFE) == 0)
    {
        WP_Diable();
        IIC_Stop();

        //        rt_kprintf("\n IICBSP: I2C_EE_BufRead SLAVE_ADDR\n");
        return (EEPROM_BUSSERRO);
    }
    // Data high 8 addr
    if (I2c_write_byte(read_addr >> 8) == 0)
    {
        WP_Diable();
        IIC_Stop();
        //        rt_kprintf("\n IICBSP: I2C_EE_BufRead ADDR high 8\n");
        return (EEPROM_BUSSERRO);
    }
#endif

    // data low 8 addr
    if (I2c_write_byte(read_addr & 0x00ff) == 0)
    {
        WP_Diable();
        IIC_Stop();
        //        rt_kprintf("\n IICBSP: I2C_EE_BufRead ADDR LOW  8\n");
        return (EEPROM_BUSSERRO);
    }

    IIC_Start();

    if (I2c_write_byte(SLAVE_ADDR | 0x01) == 0)
    {
        WP_Diable();
        IIC_Stop();

        //        rt_kprintf("\n IICBSP: I2C_EE_BufRead SLAVE_ADDR  123\n");
        return (EEPROM_BUSSERRO);
    }

    WP_Diable();  // write protect

    for (len = 0; len < num_byte_Read - 1; len++)
    {
        *(read_buffer++) = IIC_Read_Byte(1);
    }
    *(read_buffer) = IIC_Read_Byte(0);
    IIC_Stop();
    //    delay_us(300);
    return (EEPROM_NOERRO);
}

int8_t I2C_EE_BufRead(uint8_t* read_buffer, uint16_t read_addr, uint16_t num_byte_read)
{
    int8_t erro;
    erro = EEPROM_NOERRO;
    erro = I2C_EE_BufRead_bsp(read_buffer, read_addr, num_byte_read);
    return (erro);
}
