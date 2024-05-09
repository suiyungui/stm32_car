#include "headfile.h"

int main(void)
{
	OLED_Init();
	
	motor_init();
	motor_duty(30000);
	
	while (1)
	{

	} 
}
