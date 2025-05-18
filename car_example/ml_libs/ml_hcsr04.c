#include "ml_hcsr04.h"

// ȫ�ֱ���
static HCSR04_State hcsr04_state = HCSR04_IDLE;
static uint32_t echo_start_time = 0;
static uint32_t echo_stop_time = 0;
static int32_t distance_mm = -1;
static uint32_t timeout_counter = 0;

// ��ʼ��������ʱ��
static void init_measure_timer(void)
{
    // ����TIM3ʱ��
    RCC->APB1ENR |= 1<<1;  // TIM3ʱ��ʹ��
    
    // ����Ԥ��Ƶ��ʹ�ü�����Ƶ��Ϊ1MHz(1us��һ����)
    uint16_t prescaler = SystemCoreClock / 1000000 - 1; // 1 tick = 1us (0.165mm�ֱ���)
    
    TIM3->PSC = prescaler;    // Ԥ��Ƶ��
    TIM3->ARR = 0xFFFF;       // �Զ���װ��ֵ
    
    // ������ʱ��������λ������
    TIM3->CR1 |= (1<<0);      // CEN=1��ʹ�ܶ�ʱ��
}

// ��ʼ������
static void init_pins(void)
{
    // ���ô�������Ϊ�������
    gpio_init(US_TRIG_PORT, US_TRIG_PIN, OUT_PP);
    gpio_set(US_TRIG_PORT, US_TRIG_PIN, 0); // ��ʼ״̬Ϊ�͵�ƽ
    
    // ���ûز�����Ϊ��������
    gpio_init(US_ECHO_PORT, US_ECHO_PIN, IU);
}

// ��ʼ��HC-SR04
void hcsr04_init(void)
{
    // ʹ������ʱ��(GPIOB)
    RCC->APB2ENR |= (1<<3);        // GPIOBʱ��ʹ��
    
    init_pins();
    init_measure_timer();
    
    // ��ʼ��״̬Ϊ����
    hcsr04_state = HCSR04_IDLE;
    distance_mm = -1;
}

// ����10us�Ĵ�������
static void send_trigger_pulse(void)
{
    gpio_set(US_TRIG_PORT, US_TRIG_PIN, 1); // ����Ϊ�ߵ�ƽ
    delay_us(10);                           // �ȴ�10us
    gpio_set(US_TRIG_PORT, US_TRIG_PIN, 0); // ����Ϊ�͵�ƽ
}

// ��������������
void hcsr04_trigger(void)
{
    // ֻ���ڿ���״̬���ܴ����µĲ���
    if (hcsr04_state == HCSR04_IDLE) {
        // ���ʹ�������
        send_trigger_pulse();
        
        // ����״̬�ͳ�ʱ������
        hcsr04_state = HCSR04_TRIGGERED;
        timeout_counter = 0;
        
        // ����δ��ɣ�������Ч
        distance_mm = -1;
    }
}

// ���������������̣�������
HCSR04_State hcsr04_process(void)
{
    // ���ݵ�ǰ״̬���д���
    switch (hcsr04_state) {
        case HCSR04_IDLE:
            // ����״̬��û�в���
            break;
            
        case HCSR04_TRIGGERED:
            // �Ѵ������ȴ��ز�������
            if (gpio_get(US_ECHO_PORT, US_ECHO_PIN) == 1) {
                // ��⵽�ز������أ���¼��ʼʱ��
                echo_start_time = TIM3->CNT;
                hcsr04_state = HCSR04_ECHO_RECEIVED;
                timeout_counter = 0;
            } else {
                // ���ӳ�ʱ������
                timeout_counter++;
                if (timeout_counter > 50000) {
                    // ��ʱ��δ��⵽�ز�
                    hcsr04_state = HCSR04_TIMEOUT;
                }
            }
            break;
            
        case HCSR04_ECHO_RECEIVED:
            // �ѽ��յ��ز������أ��ȴ��½���
            if (gpio_get(US_ECHO_PORT, US_ECHO_PIN) == 0) {
                // ��⵽�ز��½��أ��������
                echo_stop_time = TIM3->CNT;
                
                // ������������
                if (echo_stop_time >= echo_start_time) {
                    uint32_t echo_duration = echo_stop_time - echo_start_time;
                    // �����ڿ������ٶ�ԼΪ340m/s��������Ҫ2��ʱ��
                    // ����(mm) = ʱ��(us) * 0.17
                    distance_mm = echo_duration * 170 / 1000;
                    
                    // ���볬��4�׿��ܲ�׼ȷ
                    if (distance_mm > 4000) {
                        distance_mm = -1;
                    }
                } else {
                    // �������������Ч����
                    distance_mm = -1;
                }
                
                // �ص�����״̬�����Խ�����һ�β���
                hcsr04_state = HCSR04_IDLE;
            } else {
                // ���ӳ�ʱ������
                timeout_counter++;
                if (timeout_counter > 50000) {
                    // ��ʱ��δ��⵽�ز��½���
                    hcsr04_state = HCSR04_TIMEOUT;
                }
            }
            break;
            
        case HCSR04_TIMEOUT:
            // ������ʱ���ص�����״̬
            hcsr04_state = HCSR04_IDLE;
            distance_mm = -1;
            break;
            
        default:
            // δ֪״̬������Ϊ����
            hcsr04_state = HCSR04_IDLE;
            break;
    }
    
    return hcsr04_state;
}

// ��ȡ���һ�β����ľ���
int32_t hcsr04_get_distance(void)
{
    return distance_mm;
} 


