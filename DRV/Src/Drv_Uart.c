#include "Drv_Uart.h"
#include "stm32g0xx_hal.h"
#include "stm32f0xx_misc.h"
#include <stdarg.h>
#include <stdio.h>
#include <macro.h>
#include <global.h>
//#include "calc.h"

/* Private function prototypes -----------------------------------------------*/

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/

/*********************************************************
 * @name   uart1_gpio_init
 * @brief  USART1 GPIO initialization. Both uart port pins and direction control pins are configured
 * @calls  device drivers
 * @called uart1_init()
 * @param  None
 * @retval None
 *********************************************************/
static void uart1_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    // USART1 GPIO and port clock enable
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    //	//USART1 GPIO function remapping
    //	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_1);
    //	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_1);

    // USART1 GPIO configuration
    GPIO_InitStructure.GPIO_Pin   = UART1_GPIO_TX | UART1_GPIO_RX;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART1_GPIO, &GPIO_InitStructure);

    // RS485 direction control
    //	RCC_AHBPeriphClockCmd( RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = UART1_DIR_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(UART1_DIR_GPIO, &GPIO_InitStructure);
    RE485
}

/*********************************************************
 * @name   uart1_nvic_init
 * @brief  USART1 related IRQ vector initialization, include DMA and USART IRQs
 * @calls  device drivers
 * @called uart1_init()
 * @param  None
 * @retval None
 *********************************************************/
static void uart1_nvic_init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable the USART1 tx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel         = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

/*********************************************************
  * @name   uart1_init
    * @brief  General USART1 initialization, after which USART1 is ready to use
    * @calls  uart1_gpio_init()
            uart1_nvic_init()
  * @called Communiction_proc()
  * @param  None
  * @retval None
*********************************************************/
void uart1_init(uint16_t baudrate)  //串口初始化函数
{
    USART_InitTypeDef USART_InitStructure;

    uart1_gpio_init();

    USART_InitStructure.USART_BaudRate            = baudrate;                        //设置串口波特率
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;             //设置数据位
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;                //设置停止位
    USART_InitStructure.USART_Parity              = USART_Parity_No;                 //设置效验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //设置流控制
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;   //设置工作模式
    USART_Init(USART1, &USART_InitStructure);                                        //配置入结构体
    USART_Cmd(USART1, ENABLE);                                                       //使能串口1
    uart1_nvic_init();
}

/*********************************************************
  * @name   uart1_dma_send
    * @brief  send data from USART1 port with DMA and auto direction control(RS485)
    * @calls  device drivers
  * @called framing()
  * @param  s_addr  : data buffer to be sent out through uart port
                        length : data length
  * @retval None
*********************************************************/
void uart1_dma_send(uint8_t *s_addr, uint16_t length)
{
    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel2);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->TDR);
    DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)s_addr;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize         = length;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  // DISABLE UART RX IT
    DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    TE485
    DMA_Cmd(DMA1_Channel2, ENABLE);
}

///**********************************TDS通信***********************************************/

static uint8_t u8FrameCheckSum;
static uint8_t u8FrameDataLength;
uint8_t g_ComBuff[PROTOCOL_FRAME_MaxLen];
PROTOCOL_STATUS g_ComStat;  //通讯通道工作状态
// uint8_t g_ComGap[1];          //通讯帧接收间隔定时器
ProtocolLayer Protocol;

void Usart2_Rcc_Conf(void)
{
    /* Enable UART GPIO clocks */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    return;
}

void Usart2_Gpio_Conf(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    Usart2_Rcc_Conf();
    // USART1 Tx(PA.14)
    // USART1 Rx(PA.15)
    //		GPIO_PinRemapConfig(GPIO_Remap_SWJ_DisableENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);
    return;
}

void Usart2_Init_Conf(void)
{
    // USART1配置
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    Usart2_Gpio_Conf();
    USART_InitStructure.USART_BaudRate            = 9600;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Tx | USART_Mode_Rx;

    ENTER_CRITICAL_SECTION();  //关全局中断

    USART_Init(UART_TDS, &USART_InitStructure);
    USART_ITConfig(UART_TDS, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART_TDS, ENABLE);

    //=====================中断初始化======================================
    //设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级
    /* Enable the USART2 tx Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel         = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd      = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    USART_ITConfig(UART_TDS, USART_IT_RXNE, ENABLE);
    EXIT_CRITICAL_SECTION();  //开全局中断
    return;
}

void UsartX_Var_Init(void)
{
    g_ComStat = INIT_Apply;
    //    g_ComGap[0] = 0;
    Protocol.StatckStatus = PROTOCOL_STACK_IDLE;
    memset(&g_ComBuff[0], 0x00, PROTOCOL_FRAME_MaxLen);
    return;
}
//通信串口初始化
void TDS_Usart_Init(void)
{
    UsartX_Var_Init();
    Usart2_Init_Conf();
    return;
}

void TDS_Usart_Close(void)
{
    USART_ClearITPendingBit(UART_TDS, USART_IT_RXNE);
    USART_Cmd(UART_TDS, DISABLE);
    return;
}
//关闭串口
void xPort_Usart_Close(void)
{
    TDS_Usart_Close();
    return;
}

//串口发送数据
uint8_t UART_Send(uint8_t *u8Buf, uint8_t u8Len)
{
    uint8_t u8Se;
    uint8_t i;
    if (u8Len == 0)
    {
        return u8Se;
    }
    for (i = 0; i < u8Len; i++)
    {
        USART_SendData(UART_TDS, u8Buf[i]);  //发送数据
        while (USART_GetFlagStatus(UART_TDS, USART_FLAG_TC) != SET)
            ;  //等待发送结束
        u8Se = 1;
    }
    return u8Se;
}

//串口发送数据
uint8_t TDS_Send(void)
{
    uint8_t u8SendBuf[SEND_NUM] = {0};
    uint8_t u8Head              = 0xA0;
    uint8_t u8Sum               = 0x00;
    uint8_t i;

    //		if(g_ComStat == SEND_Wait)
    {
        u8SendBuf[0] = u8Head;
        u8SendBuf[1] = 0x00;
        u8SendBuf[2] = 0x00;
        u8SendBuf[3] = 0x00;
        u8SendBuf[4] = 0x00;
        for (i = 0; i < SEND_NUM - 1; i++)
        {
            u8Sum += u8SendBuf[i];
        }
        u8SendBuf[SEND_NUM - 1] = u8Sum;

        u8Sum     = UART_Send(&u8SendBuf[0], SEND_NUM);
        g_ComStat = SEND_Going;
    }
    return u8Sum;
}

uint8_t xPortSerialGetByte(char *pucByte, uint8_t ucMB_Number)
{
    *pucByte = USART_ReceiveData(UART_TDS);

    return 1;
}

uint8_t AnalyseProtocol(uint8_t *u8Buff)
{
    // TDS值

    return 1;
}

///*************************************************
// Function:void Comm_Service(void)// 函数名称
// Description:// 函数功能、性能等的描述
//			执行通讯任务
//			状态机转换（正常情况）: RECV_Wait->RECV_Going->RECV_Over->
//                                     SEND_Wait->SEND_Going->SEND_Over->RECV_Wait
// Author:xdp（作者）
// Calls:// 被本函数调用的函数清单
// Called By:// 调用本函数的函数清单
//			main();
// Input:// 输入参数说明，包括每个参数的作
//     // 用、取值说明及参数间关系。
// Output:// 对输出参数的说明。
// Return:// 函数返回值的说明
// Others:// 其它说明
//*************************************************/
void Comm_Service(void)
{
    uint8_t i     = 0;
    uint8_t u8Sum = 0x00;
    if (g_ComStat == RECV_Over)
    {
        for (i = 0; i < SEND_NUM - 1; i++)
        {
            u8Sum += g_ComBuff[i];
        }
        if (u8Sum == g_ComBuff[SEND_NUM - 1])
        {
            g_sVariable.status.Water.u16TDS[0] = g_ComBuff[1] << 8 | g_ComBuff[2];
            g_sVariable.status.Water.u16TDS[0] += (int16_t)(g_sVariable.gPara.Water.u16TDS_Cail);
        }

        //			u8Sum=0;
        //			for (i = 6; i < 11; i++)
        //			{
        //					u8Sum += g_ComBuff[i];
        //			}
        //			if(u8Sum==g_ComBuff[11])
        //			{
        //				g_sVariable.status.Water.u16TDS[1]=g_ComBuff[7]<<8|g_ComBuff[7];
        //
        ////				g_sVariable.status.Water.u16TDS[1]+=(int16_t)(g_sVariable.gPara.Water.u16TDS_Cail[1]);
        //			}

        memset(&g_ComBuff[0], 0x00, PROTOCOL_FRAME_MaxLen);
        g_ComStat = SEND_Wait;
    }
    return;
}

// TDS串口接收数据
uint8_t PortSerialReceiveFSM(void)
{
    uint8_t RXDByte;
    //    uint8_t u8CheckSum;

    /* Always read the character. */
    (void)xPortSerialGetByte((char *)&RXDByte, 0);
    //    g_ComGap[0] = 0;
    switch (Protocol.StatckStatus)
    {
        case PROTOCOL_STACK_IDLE:
            // 空闲状态,等待接收帧首字符(01)
            if (0xAA == RXDByte)
            {
                g_ComStat             = RECV_Going;  //置"接收进行"状态
                g_ComBuff[0]          = RXDByte;
                u8FrameCheckSum       = RXDByte;
                Protocol.DataCount    = 1;
                u8FrameDataLength     = 11;
                Protocol.StatckStatus = PROTOCOL_STACK_RCV;
            }
            break;
        case PROTOCOL_STACK_CK:
            break;
        case PROTOCOL_STACK_RCV:
            // 累加校验和
            u8FrameCheckSum += RXDByte;
            g_ComBuff[(Protocol.DataCount)++] = RXDByte;
            //				memcpy((u8*)&g_sVariable.status.TEST[1],(u8*)&g_ComBuff[port][1],8);	//
            u8FrameDataLength--;
            // 接收数据域的数据
            if (!u8FrameDataLength)
            {
                // 接收校验和
                //            u8FrameDataLength = 1;
                //            Protocol[port].StatckStatus++;
                Protocol.StatckStatus = PROTOCOL_STACK_IDLE;
                g_ComStat             = RECV_Over;  //置"接收完成"状态
            }
            break;
        case PROTOCOL_STACK_CS:
            g_ComBuff[(Protocol.DataCount)++] = RXDByte;
            u8FrameCheckSum += RXDByte;
            u8FrameDataLength--;
            // 接收数据域的数据
            if (!u8FrameDataLength)
            {
            }
            break;
        case PROTOCOL_STACK_END:

            break;
        default:
            Protocol.StatckStatus = PROTOCOL_STACK_IDLE;
            g_ComStat             = INIT_Apply;  //置"初始化"状态
            break;
    }
    return 1;
}

void USART2_IRQHandler(void)
{
    ENTER_CRITICAL_SECTION();  //关全局中断
    //溢出错误
    if (USART_GetFlagStatus(UART_TDS, USART_FLAG_ORE) == SET)
    {
        ;
    }
    //接收中断
    if (USART_GetITStatus(UART_TDS, USART_IT_RXNE) == SET)
    {
        PortSerialReceiveFSM();
        USART_ClearITPendingBit(UART_TDS, USART_IT_RXNE);
    }
    //发送中断
    if (USART_GetITStatus(UART_TDS, USART_IT_TXE) == SET)
    {
        ;
    }
    EXIT_CRITICAL_SECTION();  //开全局中断
}
///**************************************TDS通信END*********************************************/
