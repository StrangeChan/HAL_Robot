#include "mpu6050.h"
#include "usart.h"
#include "delay.h"

/*******************************

为模块化的MPU6050所写，该模块自带卡尔曼滤波算法
使用前建议通过上位机对加速度、角速度做矫正
MPU_Init()函数为Z轴校准

********************************/

void MPU6_Init()
{
	//角度初始化指令，0XFF,0XAA,0X01,0X04,0X00
	u8 cmd[5] = {0XFF,0XAA,0X01,0X04,0X00};
	//55 53 3F 2B D6 DD 26 3F 64 
	HAL_UART_Transmit(&huart2,cmd,5,1000);
	
	delay_ms(10);
}


