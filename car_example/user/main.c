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
	
	// 初始化I2C和MPU6050
	I2C_Init();
	MPU6050_Init();
	//HMC5883L_Init();
    
    // 初始化mcu_dmp算法（只使用加速度计和陀螺仪的姿态估计方案）
    imu_init();
	
	// 初始化外部中断，设置为PB7上升沿触发，优先级为0
	// MPU6050的INT引脚连接到PB7，当有新数据时产生中断
	exti_init(EXTI_PB7,RISING,0);
	
	// 初始化定时器中断，用于其他控制逻辑
	tim_interrupt_ms_init(TIM_3,10,0);
	
	while (1)
	{
//		printf("ax:%d, ay:%d az:%d gx:%d gy:%d gz:%d\r\n", ax, ay, az, gx, gy, gz);
//		printf("yaw:%.2f  pitch:%.2f roll:%.2f\r\n", yaw_Kalman, pitch_Kalman, roll_Kalman);
		
//		// 在OLED上显示偏航角（来自mcu_dmp算法的结果）
		OLED_ShowFloat(1, 1, yaw_Kalman, 3, 2);

//		// 添加俯仰角和横滚角的显示
//		OLED_ShowFloat(2, 1, pitch_Kalman, 3, 2);
//		OLED_ShowFloat(3, 1, roll_Kalman, 3, 2);
		
		delay_ms(20);
	} 
}
