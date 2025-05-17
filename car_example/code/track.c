#include "track.h"
#include <math.h>

uint8_t track_state = 0;    // 跟踪状态
uint16_t track_distance[5] = {50, 80, 50, 80, 0}; // 距离设置(cm)
float track_angle[5] = {-90.0f, -180.0f, 90.0f, 0.0f, 0.0f}; // 角度设置，-270度等价于90度
uint8_t is_turning = 0;     // 转向状态标志
uint8_t task_complete = 0;  // 任务完成标志

// 执行转弯完成操作
void complete_turning()
{
    // 停止电机
    motor_target_set(0, 0);
    delay_ms(300);
    
    // 转弯结束
    is_turning = 0;
    
    // 确保电机方向都设置为前进
    motorA_dir = 1; // 1为正转
    motorB_dir = 1; // 1为正转
    
    // 执行下一段距离运动
    if(turn_count < 3) {
        // 设置PID目标速度为前进
        motor_target_set(50, 50); // 设置两个电机以相同速度前进
        move_distance(track_distance[turn_count]);
        turn_count++;
    } else {
        task_complete = 1;
        // 任务完成，停止电机
        motor_target_set(0, 0);
    }
}

void track_init(void)
{
    // 初始化状态
    track_state = 0;
    is_turning = 0;
    task_complete = 0;
    turn_count = 0;
    
    delay_ms(500); // 确保IMU稳定
    
    // 设置PID目标速度为前进
    motor_target_set(50, 50); // 设置两个电机以相同速度前进
    
    // 开始第一段直线移动
    move_distance(track_distance[0]);
}

// 检测转向是否完成
void check_turn_complete(void)
{
    if(!is_turning) return;
    
    // 获取当前角度和目标角度
    float current_angle = yaw_Kalman;
    float target_angle = track_angle[turn_count];
    
    // 存储上一次有效的角度值，用于异常值检测
    static float last_valid_angle = 0.0f;
    static uint8_t is_initialized = 0;
    
    // 初始化上一次有效角度
    if(!is_initialized) {
        last_valid_angle = current_angle;
        is_initialized = 1;
        return;
    }
    
    // 检测异常值：与上一次有效值相差太大可能是跳变引起的临时异常
    float raw_diff = fabs(current_angle - last_valid_angle);
    if(raw_diff > 90.0f && raw_diff < 270.0f) {
        // 可能是异常值或正在穿越±180°边界，暂时跳过
        return;
    }
    
    // 更新上一次有效角度
    last_valid_angle = current_angle;
    
    // 计算角度差，需要考虑-180/+180跳变的情况
    float angle_diff = current_angle - target_angle;
    
    // 处理跳变情况
    if(angle_diff > 180.0f) {
        angle_diff -= 360.0f;
    } else if(angle_diff < -180.0f) {
        angle_diff += 360.0f;
    }
    
    // 转向完成条件：角度差的绝对值小于5度
    if(fabs(angle_diff) < 5.0f) {
        complete_turning();
    }
}

// 任务状态检查函数
void track_task_check(void)
{
    if(task_complete) return;
    
    // 先检查距离，再检查转向
    check_distance();
    check_turn_complete();
}




