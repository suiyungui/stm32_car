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
    // ��angle_control������ʼ��
    static int last_turn_count = -1;
    if(turn_count != last_turn_count) {
        angle.iout = 0; // ��ת�俪ʼ�����������
        last_turn_count = turn_count;
    }
    
    // �ǶȻ�
    // 1.�趨Ŀ��Ƕ�
    angle.target = target_angle;
    
    // 2.��ȡ��ǰ�Ƕ�
    angle.now = yaw_Kalman;
    
    // 3.����ǶȲ�������·��
    // ע�⣺������Ҫ��ʽ����ǶȲ����������pid_cal�еļ���
    // ԭ����������Ҫ���⴦��Ƕȵ����������-180/+180�ȱ߽磩
    float angle_diff = angle.target - angle.now;
    
    // �����һ��ת�䣬ǿ��ʹ����ת
    if(turn_count == 0 || turn_count == 1 || turn_count == 2) {
        // ǿ��ʹ����ת(��ֵ������ת)
        // ȷ��angle_diffΪ��ֵ����ת
        if(angle_diff > 0) {
            angle_diff -= 360.0f;
        }
    } 
    // ��������£�ѡ�����·��
    else {
        // ����ǶȲȷ��ѡ�����·��
        // ԭ����-180��+180�ȷ�Χ�ڣ�ȷ���ǶȲ����180��
        if(angle_diff > 180.0f) {
            angle_diff -= 360.0f;
        } else if(angle_diff < -180.0f) {
            angle_diff += 360.0f;
        }
    }
    
    // ֱ��ʹ�ü�����ĽǶȲ�����ǵ���pid_cal
    // ԭ��pid_cal�����¼��� error[0] = target - now
    // ��Ḳ�����Ǵ�����ĽǶȲ�������·��ѡ��ʧЧ
    // ��¼ǰ����ƫ��
    angle.error[2] = angle.error[1];
    angle.error[1] = angle.error[0];
    angle.error[0] = angle_diff; // ���õ�ǰ���Ϊ�����ĽǶȲ�
    
    // ����PID���
    // ע�⣺����ֱ��ʵ��PID���㣬�����ǵ���pid_cal����
    // �߼���pid_cal��ͬ�������������Ǵ�����ĽǶȲ�
    if(angle.pid_mode == POSITION_PID)  // λ��ʽ
    {
        angle.pout = angle.p * angle.error[0];
        angle.iout += angle.i * angle.error[0];
        angle.dout = angle.d * (angle.error[0] - angle.error[1]);
        angle.out = angle.pout + angle.iout + angle.dout;
    }
    else if(angle.pid_mode == DELTA_PID)  // ����ʽ
    {
        angle.pout = angle.p * (angle.error[0] - angle.error[1]);
        angle.iout = angle.i * angle.error[0];
        angle.dout = angle.d * (angle.error[0] - 2 * angle.error[1] + angle.error[2]);
        angle.out += angle.pout + angle.iout + angle.dout;
    }
    
    // ת���ٶ����ƣ�ֱ���������ֵ��
    float max_turn_speed = 15.0f; // ת������ٶ�
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


