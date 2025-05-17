#include "headfile.h"

int main(void)
{
	OLED_Init();
	
	motor_init();
	encoder_init();
	wheel_encoder_init(); // ��ʼ����������������

	uart_init(UART_1,115200,0);
	
	pid_init(&motorA, DELTA_PID, 3, 5, 1);
	pid_init(&motorB, DELTA_PID, 3, 5, 1);
	pid_init(&angle, POSITION_PID, 0.2, 0, 2);  // ����Pֵ��Dֵ��ʹת�������
	
	// ��ʼ��I2C��MPU6050
	I2C_Init();
	MPU6050_Init();   
    // ��ʼ��mcu_dmp�㷨��ֻʹ�ü��ٶȼƺ������ǵ���̬�ںϷ���
    imu_init();
	
	// ��ʼ���ⲿ�жϣ��ܽ�ΪPB7�������ش��������ȼ�Ϊ0
	// MPU6050��INT�������ӵ�PB7�����ݾ���ʱ�ᴥ���ж�
	exti_init(EXTI_PB7,RISING,0);
	
	// ��ʼ����ʱ���жϣ���������߼�
	tim_interrupt_ms_init(TIM_3,10,0);
    
//	motor_target_set(150, 150); // ��Χ0-150���� ��150ʱdutyΪ45000���Ƚϰ�ȫ
	
    track_init();  // ��ʼ���켣����
    servo_angle(10);// ��ʼ��
	while (1)
	{
		track_task_check();  // �������״̬������check_distance��check_turn_complete��
		printf("yaw:%.2f\r\n",yaw_Kalman);
//		printf("motorA:%d,%d\n", (uint8_t)motorA.target,(uint8_t)motorA.now);

//		// ��OLED����ʾƫ���ǣ�����mcu_dmp�㷨�Ľ��
		OLED_ShowFloat(1, 1, yaw_Kalman, 3, 2);
		
		delay_ms(20);
	} 
}
