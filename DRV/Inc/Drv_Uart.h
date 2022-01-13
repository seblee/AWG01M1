#ifndef __UART_H
#define __UART_H

#include "stm32g0xx_hal.h"
#include <stdio.h>
#include <macro.h>

/* USART1 */
#define UART1_GPIO_TX GPIO_PIN_6
#define UART1_GPIO_RX GPIO_PIN_7
#define UART1_GPIO GPIOB
#define UART1_DIR_GPIO GPIOA
#define UART1_DIR_GPIO_PIN GPIO_PIN_12

#define TE485 GPIO_SetBits(UART1_DIR_GPIO, UART1_DIR_GPIO_PIN);
#define RE485 GPIO_ResetBits(UART1_DIR_GPIO, UART1_DIR_GPIO_PIN);

#define UART_TDS USART2
#define SEND_NUM 6

#define PROTOCOL_FRAME_MaxLen 15

#define TDS_OFFSET 6
typedef enum
{
    PROTOCOL_STACK_IDLE = 0, /*!< Receiver is in idle state. */
    PROTOCOL_STACK_CK,       /*!< Frame is beeing received. */
    PROTOCOL_STACK_RCV,      /*!< Frame is beeing received. */
    PROTOCOL_STACK_CS,       /*!< Frame is beeing received. */
    PROTOCOL_STACK_END,      /*!< If the frame is invalid. */
} UartRcvState;

typedef enum
{
    RECV_Wait,   //接收等待
    RECV_Going,  //接收进行
    RECV_Over,   //接收完成
    SEND_Wait,   //发送等待
    SEND_Going,  //发送进行
    SEND_Over,   //发送完成
    INIT_Apply   //初始化
} PROTOCOL_STATUS;

typedef struct
{
    //    uint8_t CommID;       // 串口号
    //    uint8_t Baudrate;     // 通道通讯的波特率
    uint8_t StatckStatus;  // 指示当前协议栈解析状态(空闲->地址解析->)
    uint8_t StatckType;  // 地址类型(私有地址[bit0],全AA或部分AA为组地址[bit1],全99为广播地址[bit2],[bit3...bit6]
                         // 未使用,保留为0,帧接收成功[bit7])
    uint8_t DataCount;  // 接收到的数据帧长度()
} ProtocolLayer;

#define PROTOCOL_FRAME_ByteGap 250
#define PROTOCOL_FRAME_SendGap 10

void uart1_init(uint16_t baudrate);
void uart1_dma_send(uint8_t *s_addr, uint16_t length);
int fputc(int ch, FILE *f);
void UART_send_byte(uint8_t byte);
uint8_t UART_Send(uint8_t *u8Buf, uint8_t u8Len);
uint8_t UART_Recive(void);
extern void TDS_Usart_Init(void);
extern void Comm_Service(void);
extern uint8_t TDS_Send(void);
extern int16_t AVGfilter_TDS(uint8_t i8Type, int16_t i16Value);

#endif /* __UART_H */
