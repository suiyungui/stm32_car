#include "headfile.h"

int main(void)
{
	OLED_Init();
	
	motor_init();
	encoder_init();
	wheel_encoder_init(); // 初始化编码器测量参数

	uart_init(UART_1,115200,0);
	
	pid_init(&motorA, DELTA_PID, 3, 5, 1);
	pid_init(&motorB, DELTA_PID, 3, 5, 1);
	pid_init(&angle, POSITION_PID, 0.2, 0, 2);  // 降低P值和D值，使转弯更缓和
	
	// 初始化I2C和MPU6050
	I2C_Init();
	MPU6050_Init();   
    // 初始化mcu_dmp算法，只使用加速度计和陀螺仪的姿态融合方法
    imu_init();
	
	// 初始化外部中断，管脚为PB7，上升沿触发，优先级为0
	// MPU6050的INT引脚连接到PB7，数据就绪时会触发中断
	exti_init(EXTI_PB7,RISING,0);
	
	// 初始化定时器中断，处理控制逻辑
	tim_interrupt_ms_init(TIM_3,10,0);
    
//	motor_target_set(150, 150); // 范围0-150左右 在150时duty为45000，比较安全
	
    track_init();  // 初始化轨迹控制
    servo_angle(10);// 初始化
	while (1)
	{
		track_task_check();  // 持续检查状态（包含check_distance和check_turn_complete）
		printf("yaw:%.2f\r\n",yaw_Kalman);
//		printf("motorA:%d,%d\n", (uint8_t)motorA.target,(uint8_t)motorA.now);

//		// 在OLED上显示偏航角，这是mcu_dmp算法的结果
		OLED_ShowFloat(1, 1, yaw_Kalman, 3, 2);
		
		delay_ms(20);
	} 
}
