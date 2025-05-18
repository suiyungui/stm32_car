#ifndef _ML_HCSR04_H_
#define _ML_HCSR04_H_

#include "headfile.h"

// 定义使用的引脚和定时器
#define US_TIMER                TIM3

#define US_TRIG_PORT            GPIO_B
#define US_TRIG_PIN             Pin_5      // 普通GPIO输出(非PWM)

#define US_ECHO_PORT            GPIO_B
#define US_ECHO_PIN             Pin_4      // TIM3_CH1 (回声输入)

// 超声波测量状态
typedef enum {
    HCSR04_IDLE = 0,        // 空闲状态
    HCSR04_TRIGGERED,       // 已触发等待回波
    HCSR04_ECHO_RECEIVED,   // 回波已收到，测量完成
    HCSR04_TIMEOUT          // 测量超时
} HCSR04_State;

/**
 * 初始化HC-SR04超声波传感器
 */
void hcsr04_init(void);

/**
 * 触发一次测量，非阻塞方式
 */
void hcsr04_trigger(void);

/**
 * 处理超声波测量状态，需要在主循环中定期调用
 * 返回当前状态
 */
HCSR04_State hcsr04_process(void);

/**
 * 获取最近一次测量的距离（单位：毫米）
 * 返回值小于0表示未收到回声信号或者测量未完成
 */
int32_t hcsr04_get_distance(void);

#endif /* _ML_HCSR04_H_ */ 


