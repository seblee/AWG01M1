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
    RECV_Wait,   //���յȴ�
    RECV_Going,  //���ս���
    RECV_Over,   //�������
    SEND_Wait,   //���͵ȴ�
    SEND_Going,  //���ͽ���
    SEND_Over,   //�������
    INIT_Apply   //��ʼ��
} PROTOCOL_STATUS;

typedef struct
{
    //    uint8_t CommID;       // ���ں�
    //    uint8_t Baudrate;     // ͨ��ͨѶ�Ĳ�����
    uint8_t StatckStatus;  // ָʾ��ǰЭ��ջ����״̬(����->��ַ����->)
    uint8_t StatckType;  // ��ַ����(˽�е�ַ[bit0],ȫAA�򲿷�AAΪ���ַ[bit1],ȫ99Ϊ�㲥��ַ[bit2],[bit3...bit6]
                         // δʹ��,����Ϊ0,֡���ճɹ�[bit7])
    uint8_t DataCount;  // ���յ�������֡����()
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
