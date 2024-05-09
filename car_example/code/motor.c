#include "motor.h"

uint8_t motor_dir = 0; // 1为正转 0为反转

void motor_init()
{
	pwm_init(TIM_2,TIM2_CH1,1000);   
	gpio_init(GPIO_A,Pin_6,OUT_PP);
	gpio_init(GPIO_A,Pin_7,OUT_PP);
}

void motor_duty(int duty)
{
	pwm_update(TIM_2,TIM2_CH1,duty);  
	gpio_set(GPIO_A,Pin_6,motor_dir);
	gpio_set(GPIO_A,Pin_7,!motor_dir);
}
