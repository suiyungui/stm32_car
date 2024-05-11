#ifndef _motor_h
#define _motor_h
#include "headfile.h"

void motor_init(void);
void motor_duty(int duty);
void encoder_init(void);

extern int Encoder_count;
extern int speed_now;
#endif
