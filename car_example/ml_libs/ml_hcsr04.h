#ifndef _ML_HCSR04_H_
#define _ML_HCSR04_H_

#include "headfile.h"

// ����ʹ�õ����źͶ�ʱ��
#define US_TIMER                TIM3

#define US_TRIG_PORT            GPIO_B
#define US_TRIG_PIN             Pin_5      // ��ͨGPIO���(��PWM)

#define US_ECHO_PORT            GPIO_B
#define US_ECHO_PIN             Pin_4      // TIM3_CH1 (��������)

// ����������״̬
typedef enum {
    HCSR04_IDLE = 0,        // ����״̬
    HCSR04_TRIGGERED,       // �Ѵ����ȴ��ز�
    HCSR04_ECHO_RECEIVED,   // �ز����յ����������
    HCSR04_TIMEOUT          // ������ʱ
} HCSR04_State;

/**
 * ��ʼ��HC-SR04������������
 */
void hcsr04_init(void);

/**
 * ����һ�β�������������ʽ
 */
void hcsr04_trigger(void);

/**
 * ������������״̬����Ҫ����ѭ���ж��ڵ���
 * ���ص�ǰ״̬
 */
HCSR04_State hcsr04_process(void);

/**
 * ��ȡ���һ�β����ľ��루��λ�����ף�
 * ����ֵС��0��ʾδ�յ������źŻ��߲���δ���
 */
int32_t hcsr04_get_distance(void);

#endif /* _ML_HCSR04_H_ */ 


