#include "Drv_Uart.h"
#include "stm32g0xx_hal.h"
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

#ifdef __cplusplus
extern "C"
{
#endif  //__cplusplus

    PUTCHAR_PROTOTYPE
    {
        // RS485A_DE = DOsnt;
        //发送脚使能，RS485为发送数据状态
        // delay_us(10);
        //等待发送脚的电平稳定
        HAL_UART_Transmit(&huart4, (uint8_t *)&ch, 1, 0xFFFF);
        //调用STM32的HAL库，发送一个字节
        // delay_us(10);
        //避免数据信号震荡造成回环数据
        // RS485A_DE = DOrec;
        //发送脚除能，RS485恢复到接收数据状态
        return (ch);
    }
#ifdef __cplusplus
}
#endif  //__cplusplus
/* Private functions ---------------------------------------------------------*/
volatile uint8_t uart1TXBuff;
volatile uint8_t uart1RXBuff;
volatile uint8_t uart2RXBuff;
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
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
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
    uart1_gpio_init();
    uart1_nvic_init();
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1RXBuff, 1);
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
    // DMA_InitTypeDef DMA_InitStructure;
    // DMA_DeInit(DMA1_Channel2);
    // DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->TDR);
    // DMA_InitStructure.DMA_MemoryBaseAddr     = (uint32_t)s_addr;
    // DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
    // DMA_InitStructure.DMA_BufferSize         = length;
    // DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    // DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    // DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    // DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;
    // DMA_InitStructure.DMA_M2M                = DMA_M2M_Disable;
    // DMA_Init(DMA1_Channel2, &DMA_InitStructure);

    // USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);  // DISABLE UART RX IT
    // DMA_ITConfig(DMA1_Channel2, DMA_IT_TC, ENABLE);
    // USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);

    TE485
    // DMA_Cmd(DMA1_Channel2, ENABLE);
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
    return;
}

extern UART_HandleTypeDef huart2;
void Usart2_Init_Conf(void)
{
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2RXBuff, 1);
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
    // USART_ClearITPendingBit(UART_TDS, USART_IT_RXNE);
    // USART_Cmd(UART_TDS, DISABLE);
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
    uint8_t u8Se = 0;
    if (u8Len == 0)
    {
        return u8Se;
    }
    if (HAL_UART_Transmit(&huart2, u8Buf, u8Len, 0xffff) != HAL_OK)
    {
        return 0;
    }

    return 1;
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
    *pucByte = uart2RXBuff;
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

        memset(&g_ComBuff[0], 0x00, PROTOCOL_FRAME_MaxLen);
        g_ComStat = SEND_Wait;
    }
    return;
}

// TDS串口接收数据
uint8_t PortSerialReceiveFSM(void)
{
    uint8_t RXDByte;
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
            // memcpy((u8 *)&g_sVariable.status.TEST[1], (u8 *)&g_ComBuff[port][1], 8);  //
            u8FrameDataLength--;
            // 接收数据域的数据
            if (!u8FrameDataLength)
            {
                // 接收校验和
                // u8FrameDataLength = 1;
                // Protocol[port].StatckStatus++;
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

/**
 * @brief  Rx Transfer completed callbacks.
 * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
 *                the configuration information for the specified UART module.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* Prevent unused argument(s) compilation warning */
    UNUSED(huart);
    /* NOTE: This function should not be modified, when the callback is needed,
             the HAL_UART_RxCpltCallback could be implemented in the user file
     */
    if (huart == &huart1)
    {
        g_sVariable.status.Com_error = 0;
        prvvUARTRxISR();
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&uart1RXBuff, 1);
    }
    if (huart == &huart2)
    {
        PortSerialReceiveFSM();
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&uart2RXBuff, 1);
    }
}

//**************************************TDS通信END*********************************************/
