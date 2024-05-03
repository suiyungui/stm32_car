#ifndef _pwm_h_
#define _pwm_h_
#include "headfile.h"

#define MAX_DUTY  50000

typedef enum
{
		TIM2_CH1  =  0x00,
	  TIM2_CH2  =  0x01,
	  TIM2_CH3  =  0x02,
	  TIM2_CH4  =  0x03,
	  TIM3_CH1  =  0x04,
	  TIM3_CH2  =  0x05,
	  TIM3_CH3  =  0x06,
	  TIM3_CH4  =  0x07,
	  TIM4_CH1  =  0x08,
	  TIM4_CH2  =  0x09,
	  TIM4_CH3  =  0x0a,
	  TIM4_CH4  =  0x0b,
}TIMn_CHn_enum;  //枚举定义定时器通道

void pwm_pin_init(TIMn_CHn_enum timn_chn);
void pwm_init(TIMn_enum timn,TIMn_CHn_enum timn_chn,int fre,int duty);
void pwm_duty_update(TIMn_enum timn,TIMn_CHn_enum timn_chn,int duty);

#endif
