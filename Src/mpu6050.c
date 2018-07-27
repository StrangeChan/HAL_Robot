#include "mpu6050.h"
#include "usart.h"
#include "delay.h"

/*******************************

Ϊģ�黯��MPU6050��д����ģ���Դ��������˲��㷨
ʹ��ǰ����ͨ����λ���Լ��ٶȡ����ٶ�������
MPU_Init()����ΪZ��У׼

********************************/

void IMU_Init()
{
	//�Ƕȳ�ʼ��ָ�0XFF,0XAA,0X01,0X04,0X00
	//u8 cmd[5] = {0XFF,0XAA,0X01,0X04,0X00};
	//�Ƕȳ�ʼ�� ʹ Z ��Ƕȹ���  0xFF 0xAA 0x52
	u8 cmd[3] = {0XFF,0XAA,0X52};
	//55 53 3F 2B D6 DD 26 3F 64 
	HAL_UART_Transmit(&huart2,cmd,3,1000);
	HAL_UART_Transmit(&huart1,cmd,3,1000);
	
	delay_ms(10);
}


