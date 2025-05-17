#ifndef _motor_h
#define _motor_h
#include "headfile.h"

void motor_init(void);
void motorA_duty(int duty);
void motorB_duty(int duty);
void encoder_init(void);
void servo_duty(int duty);

extern int Encoder_count1, Encoder_count2;
extern int speed_now;
extern uint8_t motorA_dir, motorB_dir;

typedef struct {
    float wheel_diameter;    // ����ֱ��(mm)
    float encoder_resolution;// �������ֱ���(ÿת������)
    float gear_ratio;       // ���ٱ�
    float distance_per_pulse;// ÿ�������Ӧ�ľ���(mm)
    float total_distance;   // �ۼƾ���(mm)
} WheelEncoder;

extern WheelEncoder wheelA;
extern WheelEncoder wheelB;

void wheel_encoder_init(void);
void wheel_encoder_update(void);
void move_distance(float distance_cm);
void check_distance(void);
void servo_angle(uint8_t angle);
void move_backward(float distance_cm);
void wait_backward_complete(void);
#endif
