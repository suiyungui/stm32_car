#include "headfile.h"

pid_t motorA;
pid_t motorB;
pid_t angle;
uint8_t turn_count = 0; // 转弯次数


void datavision_send()  // 上位机波形发送函数
{
	// 数据包头
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);

	// 发送数据
	uart_sendbyte(UART_1, (uint8_t)motorA.target);  
	uart_sendbyte(UART_1, (uint8_t)motorA.now);
//	uart_sendbyte(UART_1, (uint8_t)motorB.target);  
//	uart_sendbyte(UART_1, (uint8_t)motorB.now);
	// 数据包尾
	uart_sendbyte(UART_1, 0xfc);
	uart_sendbyte(UART_1, 0x03);
}


void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}
	
	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

void pid_control()
{
	// 转弯状态处理
	if(track_state == 1)
	{
		angle_control(track_angle[turn_count]);
		track_state = 0;
		is_turning = 1;
	}
	
	// 编码器数据更新
	wheel_encoder_update();
	Encoder_count1 = Encoder_count1 * 10;
    Encoder_count2 = Encoder_count2 * 10;
	
	// 获取当前速度
	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
	if(motorB_dir){motorB.now = Encoder_count2;}else{motorB.now = -Encoder_count2;}

    Encoder_count1 = 0;
	Encoder_count2 = 0;
	
	// PID计算和电机控制
	pid_cal(&motorA);
	pid_cal(&motorB);
	
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);
}
void pid_cal(pid_t *pid)
{
	// 计算当前偏差
	pid->error[0] = pid->target - pid->now;

	// 计算输出
	if(pid->pid_mode == DELTA_PID)  // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // 位置式
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid)
{
	// 输出限幅
	if(pid->out>=MAX_DUTY)	
		pid->out=MAX_DUTY;
	if(pid->out<=0)	
		pid->out=0;
}


void angle_control(float target_angle)
{
    // 角度环
    // 1.设定目标角度
    angle.target = target_angle;
    // 2.获取当前角度
    angle.now = yaw_Kalman;
    // 3.PID控制器计算输出
    pid_cal(&angle);
    
    // 转弯速度限制（直接限制输出值）
    float max_turn_speed = 20.0f; // 转弯最大速度
    float turn_speed = angle.out;
    
    if(turn_speed > max_turn_speed) {
        turn_speed = max_turn_speed;
    } else if(turn_speed < -max_turn_speed) {
        turn_speed = -max_turn_speed;
    }

    // 速度环
    // IMU初始位置以左为正，以右为负
    // 当angle.out为正时，表示需要向左转（左轮后退，右轮前进）
    // 当angle.out为负时，表示需要向右转（左轮前进，右轮后退）
    motor_target_set(-turn_speed, turn_speed);
}


