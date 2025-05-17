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
    /* 舵机pwm配置 */
    pwm_init(TIM_4,TIM4_CH1,50); 
}

// 舵机控制函数，参数为脉宽(微秒)
// 标准舵机一般使用500-2500微秒脉宽控制
void servo_duty(int pulse_width)  
{
    // 将脉宽(微秒)转换为对应的duty值
    // 50Hz时周期为20000微秒，将脉宽映射到MAX_DUTY值范围
    uint16_t duty = (uint16_t)((float)pulse_width / 20000.0f * MAX_DUTY);
    
    // 限制duty值在有效范围内
    if(duty > MAX_DUTY) duty = MAX_DUTY;
    
    // 更新PWM占空比
    pwm_update(TIM_4, TIM4_CH1, duty);
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
        
        // 如果是第一段距离(turn_count=0)，控制舵机闭合并延迟继续
        if(turn_count == 0) {
            // 停车后等待稳定
            delay_ms(500);
            
            // 控制舵机闭合
            servo_angle(80);
            
            // 等待舵机动作完成
            delay_ms(1000);
            
            // 继续执行转弯
            track_state = 1;
        }
        // 如果是第二段距离(turn_count=1)，控制舵机松开并延迟继续
        else if(turn_count == 1) {
            // 停车后等待稳定
            delay_ms(500);
            
            // 控制舵机松开
            servo_angle(10);
            
            // 等待舵机动作完成
            delay_ms(1000);
            
            // 启动倒车
            move_backward(20);
            
            // 使用专用函数等待倒车完成，避免递归调用
            wait_backward_complete();
            
            // 恢复电机方向为正向
            motorA_dir = 1;
            motorB_dir = 1;
            
            // 继续执行转弯
            track_state = 1;
        }
        // 如果是第三段距离(turn_count=2)，控制舵机闭合
        else if(turn_count == 2) {
            // 停车后等待稳定
            delay_ms(500);
            
            // 控制舵机闭合
            servo_angle(80);
            
            // 等待舵机动作完成
            delay_ms(1000);
            
            // 继续执行转弯
            track_state = 1;
        }
        // 如果是第四段距离(turn_count=3)，控制舵机松开
        else if(turn_count == 3) {
            // 停车后等待稳定
            delay_ms(500);
            
            // 控制舵机松开
            servo_angle(10);
            
            // 等待舵机动作完成
            delay_ms(1000);
            
            // 继续执行转弯
            track_state = 1;
        }
        else {
            // 不是特殊处理的段，直接触发转弯状态
            track_state = 1;
        }
    }
}

// 根据角度控制舵机(0-180度)
void servo_angle(uint8_t angle)
{
    // 将角度(0-180)映射到脉宽(500-2500微秒)
    int pulse_width = 500 + (angle * 2000 / 180);
    
    // 限制在有效范围内
    if(pulse_width < 500) pulse_width = 500;
    if(pulse_width > 2500) pulse_width = 2500;
    
    // 设置舵机脉宽
    servo_duty(pulse_width);
}

// 倒车指定距离的函数
void move_backward(float distance_cm)
{
    // 重置编码器计数和总距离
    wheelA.total_distance = 0.0f;
    wheelB.total_distance = 0.0f;
    Encoder_count1 = 0;
    Encoder_count2 = 0;
    
    // 设置电机为反转方向
    motorA_dir = 0; // 0为反转
    motorB_dir = 0; // 0为反转
    
    // 启动倒车，速度稍慢
    motor_target_set(-10, -10);
    
    // 设置目标距离（转换为毫米）
    target_distance = distance_cm * 10.0f;
    is_moving = 1;
    
    // 注意：该函数启动倒车后立即返回
    // 实际距离检测在check_distance()函数中进行
    // 可以通过检查is_moving判断是否完成
}

// 等待倒车完成的专用函数，避免递归调用
void wait_backward_complete(void)
{
    // 简单的等待循环，不递归调用
    while(is_moving) {
        // 只更新编码器数据
        wheel_encoder_update();
        
        // 检查是否达到目标距离
        float current_distance = (wheelA.total_distance + wheelB.total_distance) / 2.0f;
        if (current_distance >= target_distance) {
            // 停止运动
            motor_target_set(0, 0);
            is_moving = 0;
        }
        
        delay_ms(10);
    }
}
