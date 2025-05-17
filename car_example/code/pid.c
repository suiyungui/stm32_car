#include "headfile.h"

pid_t motorA;
pid_t motorB;
pid_t angle;
uint8_t turn_count = 0; // ת�����


void datavision_send()  // ��λ�����η��ͺ���
{
	// ���ݰ�ͷ
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);

	// ��������
	uart_sendbyte(UART_1, (uint8_t)motorA.target);  
	uart_sendbyte(UART_1, (uint8_t)motorA.now);
//	uart_sendbyte(UART_1, (uint8_t)motorB.target);  
//	uart_sendbyte(UART_1, (uint8_t)motorB.now);
	// ���ݰ�β
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
	// ת��״̬����
	if(track_state == 1)
	{
		angle_control(track_angle[turn_count]);
		track_state = 0;
		is_turning = 1;
	}
	
	// ���������ݸ���
	wheel_encoder_update();
	Encoder_count1 = Encoder_count1 * 10;
    Encoder_count2 = Encoder_count2 * 10;
	
	// ��ȡ��ǰ�ٶ�
	if(motorA_dir){motorA.now = Encoder_count1;}else{motorA.now = -Encoder_count1;}
	if(motorB_dir){motorB.now = Encoder_count2;}else{motorB.now = -Encoder_count2;}

    Encoder_count1 = 0;
	Encoder_count2 = 0;
	
	// PID����͵������
	pid_cal(&motorA);
	pid_cal(&motorB);
	
	pidout_limit(&motorA);
	pidout_limit(&motorB);
	
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);
}
void pid_cal(pid_t *pid)
{
	// ���㵱ǰƫ��
	pid->error[0] = pid->target - pid->now;

	// �������
	if(pid->pid_mode == DELTA_PID)  // ����ʽ
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // λ��ʽ
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// ��¼ǰ����ƫ��
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid)
{
	// ����޷�
	if(pid->out>=MAX_DUTY)	
		pid->out=MAX_DUTY;
	if(pid->out<=0)	
		pid->out=0;
}


void angle_control(float target_angle)
{
    // �ǶȻ�
    // 1.�趨Ŀ��Ƕ�
    angle.target = target_angle;
    // 2.��ȡ��ǰ�Ƕ�
    angle.now = yaw_Kalman;
    // 3.PID�������������
    pid_cal(&angle);
    
    // ת���ٶ����ƣ�ֱ���������ֵ��
    float max_turn_speed = 20.0f; // ת������ٶ�
    float turn_speed = angle.out;
    
    if(turn_speed > max_turn_speed) {
        turn_speed = max_turn_speed;
    } else if(turn_speed < -max_turn_speed) {
        turn_speed = -max_turn_speed;
    }

    // �ٶȻ�
    // IMU��ʼλ������Ϊ��������Ϊ��
    // ��angle.outΪ��ʱ����ʾ��Ҫ����ת�����ֺ��ˣ�����ǰ����
    // ��angle.outΪ��ʱ����ʾ��Ҫ����ת������ǰ�������ֺ��ˣ�
    motor_target_set(-turn_speed, turn_speed);
}


