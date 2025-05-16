#include "headfile.h"

int main(void)
{
	OLED_Init();
	
	motor_init();
	encoder_init();

	uart_init(UART_1,115200,0);
	
	pid_init(&motorA, DELTA_PID, 10, 10, 5);
	pid_init(&motorB, DELTA_PID, 10, 10, 5);
	pid_init(&angle, POSITION_PID, 2, 0, 0.5);
	
	// ��ʼ��I2C��MPU6050
	I2C_Init();
	MPU6050_Init();
	//HMC5883L_Init();
    
    // ��ʼ��mcu_dmp�㷨��ֻʹ�ü��ٶȼƺ������ǵ���̬���Ʒ�����
    imu_init();
	
	// ��ʼ���ⲿ�жϣ�����ΪPB7�����ش��������ȼ�Ϊ0
	// MPU6050��INT�������ӵ�PB7������������ʱ�����ж�
	exti_init(EXTI_PB7,RISING,0);
	
	// ��ʼ����ʱ���жϣ��������������߼�
	tim_interrupt_ms_init(TIM_3,10,0);
	
	while (1)
	{
//		printf("ax:%d, ay:%d az:%d gx:%d gy:%d gz:%d\r\n", ax, ay, az, gx, gy, gz);
//		printf("yaw:%.2f  pitch:%.2f roll:%.2f\r\n", yaw_Kalman, pitch_Kalman, roll_Kalman);
		
//		// ��OLED����ʾƫ���ǣ�����mcu_dmp�㷨�Ľ����
		OLED_ShowFloat(1, 1, yaw_Kalman, 3, 2);

//		// ��Ӹ����Ǻͺ���ǵ���ʾ
//		OLED_ShowFloat(2, 1, pitch_Kalman, 3, 2);
//		OLED_ShowFloat(3, 1, roll_Kalman, 3, 2);
		
		delay_ms(20);
	} 
}
