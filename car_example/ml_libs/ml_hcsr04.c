#include "ml_hcsr04.h"

// 全局变量
static HCSR04_State hcsr04_state = HCSR04_IDLE;
static uint32_t echo_start_time = 0;
static uint32_t echo_stop_time = 0;
static int32_t distance_mm = -1;
static uint32_t timeout_counter = 0;

// 初始化测量定时器
static void init_measure_timer(void)
{
    // 启用TIM3时钟
    RCC->APB1ENR |= 1<<1;  // TIM3时钟使能
    
    // 设置预分频器使得计数器频率为1MHz(1us记一次数)
    uint16_t prescaler = SystemCoreClock / 1000000 - 1; // 1 tick = 1us (0.165mm分辨率)
    
    TIM3->PSC = prescaler;    // 预分频器
    TIM3->ARR = 0xFFFF;       // 自动重装载值
    
    // 启动定时器但不复位计数器
    TIM3->CR1 |= (1<<0);      // CEN=1，使能定时器
}

// 初始化引脚
static void init_pins(void)
{
    // 配置触发引脚为推挽输出
    gpio_init(US_TRIG_PORT, US_TRIG_PIN, OUT_PP);
    gpio_set(US_TRIG_PORT, US_TRIG_PIN, 0); // 初始状态为低电平
    
    // 配置回波引脚为上拉输入
    gpio_init(US_ECHO_PORT, US_ECHO_PIN, IU);
}

// 初始化HC-SR04
void hcsr04_init(void)
{
    // 使能外设时钟(GPIOB)
    RCC->APB2ENR |= (1<<3);        // GPIOB时钟使能
    
    init_pins();
    init_measure_timer();
    
    // 初始化状态为空闲
    hcsr04_state = HCSR04_IDLE;
    distance_mm = -1;
}

// 发送10us的触发脉冲
static void send_trigger_pulse(void)
{
    gpio_set(US_TRIG_PORT, US_TRIG_PIN, 1); // 设置为高电平
    delay_us(10);                           // 等待10us
    gpio_set(US_TRIG_PORT, US_TRIG_PIN, 0); // 设置为低电平
}

// 触发超声波测量
void hcsr04_trigger(void)
{
    // 只有在空闲状态才能触发新的测量
    if (hcsr04_state == HCSR04_IDLE) {
        // 发送触发脉冲
        send_trigger_pulse();
        
        // 更新状态和超时计数器
        hcsr04_state = HCSR04_TRIGGERED;
        timeout_counter = 0;
        
        // 测量未完成，距离无效
        distance_mm = -1;
    }
}

// 处理超声波测量过程，非阻塞
HCSR04_State hcsr04_process(void)
{
    // 根据当前状态进行处理
    switch (hcsr04_state) {
        case HCSR04_IDLE:
            // 空闲状态，没有操作
            break;
            
        case HCSR04_TRIGGERED:
            // 已触发，等待回波上升沿
            if (gpio_get(US_ECHO_PORT, US_ECHO_PIN) == 1) {
                // 检测到回波上升沿，记录开始时间
                echo_start_time = TIM3->CNT;
                hcsr04_state = HCSR04_ECHO_RECEIVED;
                timeout_counter = 0;
            } else {
                // 增加超时计数器
                timeout_counter++;
                if (timeout_counter > 50000) {
                    // 超时，未检测到回波
                    hcsr04_state = HCSR04_TIMEOUT;
                }
            }
            break;
            
        case HCSR04_ECHO_RECEIVED:
            // 已接收到回波上升沿，等待下降沿
            if (gpio_get(US_ECHO_PORT, US_ECHO_PIN) == 0) {
                // 检测到回波下降沿，计算距离
                echo_stop_time = TIM3->CNT;
                
                // 处理计数器溢出
                if (echo_stop_time >= echo_start_time) {
                    uint32_t echo_duration = echo_stop_time - echo_start_time;
                    // 声波在空气中速度约为340m/s，来回需要2倍时间
                    // 距离(mm) = 时间(us) * 0.17
                    distance_mm = echo_duration * 170 / 1000;
                    
                    // 距离超过4米可能不准确
                    if (distance_mm > 4000) {
                        distance_mm = -1;
                    }
                } else {
                    // 计数器溢出，无效测量
                    distance_mm = -1;
                }
                
                // 回到空闲状态，可以进行下一次测量
                hcsr04_state = HCSR04_IDLE;
            } else {
                // 增加超时计数器
                timeout_counter++;
                if (timeout_counter > 50000) {
                    // 超时，未检测到回波下降沿
                    hcsr04_state = HCSR04_TIMEOUT;
                }
            }
            break;
            
        case HCSR04_TIMEOUT:
            // 测量超时，回到空闲状态
            hcsr04_state = HCSR04_IDLE;
            distance_mm = -1;
            break;
            
        default:
            // 未知状态，重置为空闲
            hcsr04_state = HCSR04_IDLE;
            break;
    }
    
    return hcsr04_state;
}

// 获取最近一次测量的距离
int32_t hcsr04_get_distance(void)
{
    return distance_mm;
} 


