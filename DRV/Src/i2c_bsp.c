
#include "stm32g0xx_hal.h"
#include "i2c_bsp.h"
#include "cmsis_os.h"
/**
 * @brief  Initializes peripherals used by the I2C EEPROM driver.
 * @param  None
 * @retval : None
 */
#define I2CSPEED 2
// I2Cģ����ʱ
#define I2CPAGEWriteDelay 5

static void I2C_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    // Configure I2C1 pins: SCL
    GPIO_InitStructure.GPIO_Pin   = II_SCL_Pin | II_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_Init(II_SCL_GPIO, &GPIO_InitStructure);
    GPIO_SetBits(II_SCL_GPIO, II_SCL_Pin | II_SDA_Pin);

    GPIO_InitStructure.GPIO_Pin   = II_WP_Pin;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(II_WP_GPIO, &GPIO_InitStructure);
    // writ_disable
    GPIO_SetBits(II_WP_GPIO, II_WP_Pin);
}

static void IIC_SCL(uint8_t n)
{
    if (n == 1)
    {
        GPIO_SetBits(II_SCL_GPIO, II_SCL_Pin);
    }
    else
    {
        GPIO_ResetBits(II_SCL_GPIO, II_SCL_Pin);
    }
}

static void IIC_SDA(uint8_t n)
{
    if (n == 1)
    {
        GPIO_SetBits(II_SDA_GPIO, II_SDA_Pin);
    }
    else
    {
        GPIO_ResetBits(II_SDA_GPIO, II_SDA_Pin);
    }
}

void drv_i2c_init(void)
{
    I2C_Config();
    return;
}

static void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = II_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_Init(II_SDA_GPIO, &GPIO_InitStructure);
}

static void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin  = II_SDA_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(II_SDA_GPIO, &GPIO_InitStructure);
}

//��ʱ�����п�����Ҫ�޸�
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
void delay_us(uint32_t nus)		//��ʱ�����п�����Ҫ�޸�
{
    uint32_t tick,delayPeriod;
    tick = osKernelSysTick();
    delayPeriod = osKernelSysTickMicroSec(nus);
    while(osKernelSysTick() - tick < delayPeriod);
}
*/
// 2������IIC?e��?D?o?
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
    //?����?I2C����??��?��?��?����?��?��?����?��y?Y
}
// 2������IIC����?1D?o?
static void IIC_Stop(void)
{
    SDA_OUT();
    // sda??��?3?
    IIC_SCL(0);
    IIC_SDA(0);
    // STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL(1);
    IIC_SDA(1);
    //����?��I2C����???����?D?o?
    delay_us(4);
}
//�̨���y��|��eD?o?��?����
//����???�̡�o1��??����?��|��e����㨹
//        0��??����?��|��e3��1|
static uint8_t IIC_Wait_Ack(void)
{
    uint8_t ucerrtime = 0;
    SDA_IN();
    // SDA����???a��?��?
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
    //����?����?3?0
    return 0;
}
// 2������ACK��|��e
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
// 2?2������ACK��|��e
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
// IIC����?����???��??��
//����??�䨮?����D?T��|��e
// 1��?��D��|��e
// 0��??T��|��e
static void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;

    SDA_OUT();
    IIC_SCL(0);
    //��-�̨�����?��?a��?��y?Y��?��?
    for (t = 0; t < 8; t++)
    {
        if ((txd & 0x80) >> 7)
            IIC_SDA(1);
        else
            IIC_SDA(0);
        txd <<= 1;
        delay_us(2);
        //��TEA5767��������ʱ���Ǳ����
        IIC_SCL(1);
        delay_us(2);
        IIC_SCL(0);
        delay_us(2);
    }
}
//?��1??��??����?ack=1������?����?��ACK��?ack=0��?����?��nACK
static uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_IN();
    // SDA����???a��?��?
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
    //����?��nACK
    else
        IIC_Ack();
    //����?��ACK
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
    uint8_t u8EE_Addr;
    //    WP_Enable();
    //    IIC_Start();
    //	IIC_Send_Byte(SLAVE_ADDR&0xFE);
    //		IIC_Wait_Ack();
    //		IIC_Send_Byte(write_addr>>8);
    //		IIC_Wait_Ack();
    //		IIC_Send_Byte(write_addr&0x00ff);
    //		IIC_Wait_Ack();
    //		for(len=0;len<num_byte_write;len++)
    //    {
    //        IIC_Send_Byte(*(write_buffer+len));
    //		IIC_Wait_Ack();
    //    }
    //	IIC_Stop();
    //    WP_Diable();
    //	delay_us(3000);
    //	return(EEPROM_NOERRO);
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
    // �����һ����Ҫд��ĳ���
    byte_count = I2C2_EE_PageSize - (write_addr % I2C2_EE_PageSize);

    if (num_byte_write <= byte_count)
    {
        byte_count = num_byte_write;
    }
    // ��д���һҳ������
    if (WriteEEROMPage(write_buffer, write_addr, byte_count) != EEPROM_NOERRO)
    {
        // д�����,���ش���
        return EEPROM_BUSSERRO;
    }

    // ���¼��㻹��Ҫд����ֽ�
    //ҳ��ʱ

    num_byte_write -= byte_count;
    if (!num_byte_write)
    {
        // �����Ѿ�д�����
        return EEPROM_NOERRO;
    }
    write_addr += byte_count;
    write_buffer += byte_count;
    // ������Ҫ���е�ҳд����
    page_number = num_byte_write / I2C2_EE_PageSize;
    // ѭ��д������
    while (page_number--)
    {
        cnt = 10;
        while (cnt)
        {
            //���д���ɹ�
            if ((WriteEEROMPage(write_buffer, write_addr, I2C2_EE_PageSize)) != EEPROM_NOERRO)
            {
                // ��ʱ
                cnt--;
                delay_us(10);
            }
            //д�ɹ���
            else
            {
                break;
            }
        }
        //���дʧ��
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
    // д�����һҳ������
    if ((WriteEEROMPage(write_buffer, write_addr, num_byte_write)) != EEPROM_NOERRO)
    {
        // д�����,���ش���
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
    //    uint16_t len ;
    //    //��ȡ������
    //    WP_Enable();
    //    IIC_Start();
    //    //I2C Slave addr
    //    IIC_Send_Byte(SLAVE_ADDR&0xFE);
    //	IIC_Wait_Ack();
    //	// Data high 8 addr
    //	IIC_Send_Byte(read_addr>>8);
    //	IIC_Wait_Ack();
    //
    //    // data low 8 addr
    //	IIC_Send_Byte(read_addr&0x00ff);
    //	IIC_Wait_Ack();
    //    IIC_Start();
    //
    //	IIC_Send_Byte(SLAVE_ADDR|0x01);
    //	IIC_Wait_Ack();
    //    WP_Diable();
    //    //write protect
    //    for(len=0;len<num_byte_Read-1;len++)
    //    {
    //        *(read_buffer++)=IIC_Read_Byte(1);
    //    }
    //    *(read_buffer)=IIC_Read_Byte(0);
    //    IIC_Stop();
    //
    //    delay_us(3000);
    //    return(EEPROM_NOERRO);
    uint16_t len;
    uint8_t u8EE_Addr;
    //��ȡ������
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
