#include "interrupt_callback.h"


//��ʱ�����£�������жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM1)
	{
		myRemoteUpCallback();
		
	}
	if(htim->Instance==TIM5)
	{
		GetPosition();
		//printf("%f\n",BasketballRobot.ThetaD);
		
	}
}




//��ʱ�����벶���жϻص�����
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//�����жϷ���ʱִ��
{
	if(htim->Instance==TIM1)
	{
		myRemoteCcCallback();
	}
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)//����Ǵ���1
	{
//		uint8_t Res;
//		static uint8_t a ;
//		a=!a;
		LED0 = !LED0;
		LED1 = !LED0;
	}
	if(huart->Instance==USART2)//����Ǵ���1
	{
			
//		if(aRxBuffer2[0]==0x55&&aRxBuffer2[1]==0x53)
//		{
//			u8  sum,i;
//			sum =0;
//			for(i = 0;i <10;i++)
//			{
//				sum += aRxBuffer2[i];
//			}
//	
//			if(sum == aRxBuffer2[10])
//			{
//				HAL_UART_Transmit(&huart1,(uint8_t*)aRxBuffer2,11,1000);	//���ͽ��յ�������
//			}	
//		}
		GetYaw();
		printf("%f\n",BasketballRobot.ThetaD);
		LED0 = !LED0;
		LED1 = !LED0;
	}
	if(huart->Instance==USART3)//����Ǵ���1
	{
//		uint8_t Res;
//		static uint8_t a ;
//		a=!a;
		LED0 = !LED0;
		LED1 = !LED0;
	}
}
