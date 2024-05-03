#include "headfile.h"

int main(void)
{
	OLED_Init();
	
	uart_init(UART_1,115200,0x01);
	
	while (1)
	{
		delay_ms(500);
		uart_sendbyte(UART_1,0x01);
		uart_sendbyte(UART_1,0x02);
		uart_sendbyte(UART_1,0x03);
		uart_sendbyte(UART_1,0x04);
	} 
}
