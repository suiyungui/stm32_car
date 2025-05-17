#include "motor.h"

#define PI 3.14159265358979323846f

uint8_t motorA_dir = 1; // 1为正转 0为反转
uint8_t motorB_dir = 0; // 1为正转 0为反转

int Encoder_count1 = 0;
int Encoder_count2 = 0;

int speed_now;

WheelEncoder wheelA = {
    .wheel_diameter = 48.0f,     // 轮子直径48mm
    .encoder_resolution = 13.0f,  // 13线霍尔编码器
    .gear_ratio = 20.0f,         // 1:20减速比
    .distance_per_pulse = 0.0f,   // 将在初始化时计算
    .total_distance = 0.0f
};

WheelEncoder wheelB = {
    .wheel_diameter = 48.0f,
    .encoder_resolution = 13.0f,
    .gear_ratio = 20.0f,
    .distance_per_pulse = 0.0f,
    .total_distance = 0.0f
};

// 添加目标距离变量
float target_distance = 0.0f;
uint8_t is_moving = 0;

void motor_init()
{
	pwm_init(TIM_2,TIM2_CH1,1000);   
	gpio_init(GPIO_A,Pin_6,OUT_PP);
	gpio_init(GPIO_A,Pin_7,OUT_PP);
	
	pwm_init(TIM_2,TIM2_CH2,1000);   
	gpio_init(GPIO_B,Pin_0,OUT_PP);
	gpio_init(GPIO_B,Pin_1,OUT_PP);
}

void motorA_duty(int duty)
{
	pwm_update(TIM_2,TIM2_CH1,duty);  
	gpio_set(GPIO_A,Pin_6,motorA_dir);
	gpio_set(GPIO_A,Pin_7,!motorA_dir);
}

void motorB_duty(int duty)
{
	pwm_update(TIM_2,TIM2_CH2,duty);  
	gpio_set(GPIO_B,Pin_0,motorB_dir);
	gpio_set(GPIO_B,Pin_1,!motorB_dir);
}


void encoder_init()
{
    exti_init(EXTI_PA2, FALLING, 0);
    gpio_init(GPIO_A, Pin_2, IU);  // A相
    gpio_init(GPIO_A, Pin_3, IU);  // B相
    
    exti_init(EXTI_PA4, FALLING, 0);
    gpio_init(GPIO_A, Pin_4, IU);  // A相
    gpio_init(GPIO_A, Pin_5, IU);  // B相
}

void wheel_encoder_init(void)
{
    // 计算每个脉冲对应的距离
    // 距离 = (π * 轮子直径) / (编码器分辨率 * 减速比)
    float circumference = PI * wheelA.wheel_diameter;
    wheelA.distance_per_pulse = circumference / (wheelA.encoder_resolution * wheelA.gear_ratio);
    wheelB.distance_per_pulse = wheelA.distance_per_pulse; // 假设两个轮子参数相同
}

void wheel_encoder_update(void)
{
    // 更新累计距离
    if(motorA_dir) {
        wheelA.total_distance += Encoder_count1 * wheelA.distance_per_pulse;
    } else {
        wheelA.total_distance -= Encoder_count1 * wheelA.distance_per_pulse;
    }
    
    if(motorB_dir) {
        wheelB.total_distance += Encoder_count2 * wheelB.distance_per_pulse;
    } else {
        wheelB.total_distance -= Encoder_count2 * wheelB.distance_per_pulse;
    }
}

void move_distance(float distance_cm)
{
    // 重置编码器计数和总距离
    wheelA.total_distance = 0.0f;
    wheelB.total_distance = 0.0f;
    Encoder_count1 = 0;
    Encoder_count2 = 0;
    
    // 设置目标距离（转换为毫米）
    target_distance = distance_cm * 10.0f;
    is_moving = 1;
}

void check_distance(void)
{
    if (!is_moving) return;
    
    // 更新编码器数据
    wheel_encoder_update();
    
    // 检查是否达到目标距离
    float current_distance = (wheelA.total_distance + wheelB.total_distance) / 2.0f;
    if (current_distance >= target_distance) {
        // 停止运动
        motor_target_set(0, 0);
        is_moving = 0;
        
        // 触发转弯状态
        track_state = 1;
    }
}
