#ifndef _ml_uart_h_
#define _ml_uart_h_
#include "headfile.h"


typedef enum
{
		UART_1  =  0x00,
	  UART_2  =  0x01,
	  UART_3  =  0x02,
}UARTn_enum;

void uart_pin_init(UARTn_enum uartn);
void uart_baud_config(UARTn_enum uartn,int baud);
void uart_init(UARTn_enum uartn,int baud,uint8_t priority);
void uart_sendbyte(UARTn_enum uartn,uint8_t Byte);
uint8_t uart_getbyte(UARTn_enum uartn);
void uart_sendstr(UARTn_enum uartn, char* str);

#endif
