#include "headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// @brief		PWM���ų�ʼ��
// @param	  timn_chn		ѡ��ʱ��ͨ��
// @return		void  
// Sample usage:	pwm_pin_init(timn_chn);  �ڲ����� �����ֶ�����
//-------------------------------------------------------------------------------------------------------------------
void pwm_pin_init(TIMn_CHn_enum timn_chn)
{
		switch(timn_chn)
		{
			case TIM2_CH1:
        gpio_init(GPIO_A,Pin_0,AF_PP);
			  break;
			case TIM2_CH2:
        gpio_init(GPIO_A,Pin_1,AF_PP);
			  break;
			case TIM2_CH3:
        gpio_init(GPIO_A,Pin_2,AF_PP);
			  break;
			case TIM2_CH4:
        gpio_init(GPIO_A,Pin_3,AF_PP);
			  break;
			case TIM3_CH1:
        gpio_init(GPIO_A,Pin_6,AF_PP);
			  break;
			case TIM3_CH2:
        gpio_init(GPIO_A,Pin_7,AF_PP);
			  break;
			case TIM3_CH3:
        gpio_init(GPIO_B,Pin_0,AF_PP);
			  break;
			case TIM3_CH4:
        gpio_init(GPIO_B,Pin_1,AF_PP);
			  break;
			case TIM4_CH1:
        gpio_init(GPIO_B,Pin_6,AF_PP);
			  break;
			case TIM4_CH2:
        gpio_init(GPIO_B,Pin_7,AF_PP);
			  break;
			case TIM4_CH3:
        gpio_init(GPIO_B,Pin_8,AF_PP);
			  break;
			case TIM4_CH4:
        gpio_init(GPIO_B,Pin_9,AF_PP);
			  break;

		}
}

 

//-------------------------------------------------------------------------------------------------------------------
// @brief		PWM��ʼ��
// @param	  timn		ѡ��ʱ��(��ѡ�õĶ�ʱ���ο�ml_tim.h�е�ö�ٶ���)
// @param	  timn_chn		ѡ��ʱ��ͨ��(��ѡ�õ�ͨ���ο�ml_pwm.h�е�ö�ٶ���)
// @param	  fre		����Ƶ��
// @return		void  
// Sample usage:	pwm_init(TIM_2,TIM2_CH1,50);     
//-------------------------------------------------------------------------------------------------------------------
void pwm_init(TIMn_enum timn,TIMn_CHn_enum timn_chn,int fre)
{ 
	uint8_t ch = timn_chn%4+1;
	RCC->APB1ENR |= 1<<timn;
	tim_index[timn]->ARR = 10000/fre-1; //�Զ���װ��ֵ
	tim_index[timn]->PSC = 7200-1;       //Ԥ��Ƶ��ֵ
  pwm_pin_init(timn_chn);
	
	switch(ch)
	{
	  case 1:
		{
			tim_index[timn]->CCMR1 |= 7<<4;	   //����PWM2ģʽ
			tim_index[timn]->CCMR1 |= 1<<3;       //Ԥװ��ʹ��
			tim_index[timn]->CCR1 = 0;            //��ʼ��CCRֵΪ0
			break;
		}
		case 2:
		{
			tim_index[timn]->CCMR1 |= 7<<12;	   //����PWM2ģʽ	
      tim_index[timn]->CCMR1 |= 1<<11;			 //Ԥװ��ʹ��
			tim_index[timn]->CCR2 = 0;            //��ʼ��CCRֵΪ0
			break;
		}
		case 3:
		{
			tim_index[timn]->CCMR2 |= 7<<4;	   //����PWM2ģʽ
      tim_index[timn]->CCMR2 |= 1<<3;			 //Ԥװ��ʹ��
			tim_index[timn]->CCR3 = 0;            //��ʼ��CCRֵΪ0
			break;		
		}
		case 4:
		{
			tim_index[timn]->CCMR2 |= 7<<12;	   //����PWM2ģʽ		
      tim_index[timn]->CCMR2 |= 1<<11;			 //Ԥװ��ʹ��
			tim_index[timn]->CCR4 = 0;            //��ʼ��CCRֵΪ0
			break;				
		}
	
	}
	tim_index[timn]->CCER |= 0x01<<(4*(ch-1)); //PWM���ʹ��
	tim_index[timn]->CR1 |= 0x81;        //ʹ�ܼ�������ARPE
}


//-------------------------------------------------------------------------------------------------------------------
// @brief		PWMռ�ձȸ���
// @param	  timn		ѡ��ʱ��(��ѡ�õĶ�ʱ���ο�ml_tim.h�е�ö�ٶ���)
// @param	  timn_chn		ѡ��ʱ��ͨ��(��ѡ�õ�ͨ���ο�ml_pwm.h�е�ö�ٶ���)
// @param	  duty    ���¹���
// @return		void  
// Sample usage:	pwm_duty_update(TIM_2,TIM2_CH1,0);     
//-------------------------------------------------------------------------------------------------------------------
void pwm_duty_update(TIMn_enum timn,TIMn_CHn_enum timn_chn,int duty)
{
	uint8_t ch = timn_chn%4+1;
	uint16_t temp = tim_index[timn]->ARR;
	if(duty>MAX_DUTY)	duty=MAX_DUTY;
	switch(ch)
	{
		case 1:
			tim_index[timn]->CCR1 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               //����CCRֵ
			break;
		case 2:	
			tim_index[timn]->CCR2 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               //����CCRֵ
			break;
		case 3:	
			tim_index[timn]->CCR3 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               //����CCRֵ
			break;		
		case 4:
			tim_index[timn]->CCR4 = (uint16_t)((float)duty/MAX_DUTY*(temp+1));               //����CCRֵ
			break;					
	}
}


