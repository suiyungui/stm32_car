#include "track.h"
#include <math.h>

uint8_t track_state = 0;    // ����״̬
uint16_t track_distance[5] = {50, 80, 50, 80, 0}; // ��������(cm)
float track_angle[5] = {-90.0f, -180.0f, 90.0f, 0.0f, 0.0f}; // �Ƕ����ã�-270�ȵȼ���90��
uint8_t is_turning = 0;     // ת��״̬��־
uint8_t task_complete = 0;  // ������ɱ�־

// ִ��ת����ɲ���
void complete_turning()
{
    // ֹͣ���
    motor_target_set(0, 0);
    delay_ms(300);
    
    // ת�����
    is_turning = 0;
    
    // ȷ�������������Ϊǰ��
    motorA_dir = 1; // 1Ϊ��ת
    motorB_dir = 1; // 1Ϊ��ת
    
    // ִ����һ�ξ����˶�
    if(turn_count < 3) {
        // ����PIDĿ���ٶ�Ϊǰ��
        motor_target_set(50, 50); // ���������������ͬ�ٶ�ǰ��
        move_distance(track_distance[turn_count]);
        turn_count++;
    } else {
        task_complete = 1;
        // ������ɣ�ֹͣ���
        motor_target_set(0, 0);
    }
}

void track_init(void)
{
    // ��ʼ��״̬
    track_state = 0;
    is_turning = 0;
    task_complete = 0;
    turn_count = 0;
    
    delay_ms(500); // ȷ��IMU�ȶ�
    
    // ����PIDĿ���ٶ�Ϊǰ��
    motor_target_set(50, 50); // ���������������ͬ�ٶ�ǰ��
    
    // ��ʼ��һ��ֱ���ƶ�
    move_distance(track_distance[0]);
}

// ���ת���Ƿ����
void check_turn_complete(void)
{
    if(!is_turning) return;
    
    // ��ȡ��ǰ�ǶȺ�Ŀ��Ƕ�
    float current_angle = yaw_Kalman;
    float target_angle = track_angle[turn_count];
    
    // �洢��һ����Ч�ĽǶ�ֵ�������쳣ֵ���
    static float last_valid_angle = 0.0f;
    static uint8_t is_initialized = 0;
    
    // ��ʼ����һ����Ч�Ƕ�
    if(!is_initialized) {
        last_valid_angle = current_angle;
        is_initialized = 1;
        return;
    }
    
    // ����쳣ֵ������һ����Чֵ���̫������������������ʱ�쳣
    float raw_diff = fabs(current_angle - last_valid_angle);
    if(raw_diff > 90.0f && raw_diff < 270.0f) {
        // �������쳣ֵ�����ڴ�Խ��180��߽磬��ʱ����
        return;
    }
    
    // ������һ����Ч�Ƕ�
    last_valid_angle = current_angle;
    
    // ����ǶȲ��Ҫ����-180/+180��������
    float angle_diff = current_angle - target_angle;
    
    // �����������
    if(angle_diff > 180.0f) {
        angle_diff -= 360.0f;
    } else if(angle_diff < -180.0f) {
        angle_diff += 360.0f;
    }
    
    // ת������������ǶȲ�ľ���ֵС��5��
    if(fabs(angle_diff) < 5.0f) {
        complete_turning();
    }
}

// ����״̬��麯��
void track_task_check(void)
{
    if(task_complete) return;
    
    // �ȼ����룬�ټ��ת��
    check_distance();
    check_turn_complete();
}




