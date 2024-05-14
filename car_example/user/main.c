#include "headfile.h"

int main(void)
{
	OLED_Init();
	
	motor_init();
	encoder_init();
	motor_duty(10000);
	uart_init(UART_1,115200,0x00);
	
	pid_init(&motorA, DELTA_PID, 10, 10, 5);
	motor_target_set(-250);
	
	tim_interrupt_ms_init(TIM_3,10,0);
	while (1)
	{
		printf("speed_now:%d\r\n", (int)motorA.now);
	} 
}
