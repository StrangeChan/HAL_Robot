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
//		if(huart2.RxState == HAL_UART_STATE_BUSY_RX && aRxBuffer2[0] != 0x55)
//		{
//			HAL_UART_AbortReceive(&huart2);
//			HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, USART2_REC_LEN);	
//			UartRxTimEnble &= ~0x02;
//		}
		//GetPosition();
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
		HAL_UART_Receive_IT(&huart1,(u8 *)aRxBuffer1, USART1_REC_LEN);
		
		GetVisionData();
		LED0 = !LED0;
		LED1 = !LED0;
	}
	if(huart->Instance==USART2)//����Ǵ���1
	{
		HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, USART2_REC_LEN);

		GetYaw();
		//printf("%f\n",BasketballRobot.ThetaD);
		
		LED0 = !LED0;
		LED1 = !LED0;
	}
	if(huart->Instance==USART3)//����Ǵ���1
	{
		HAL_UART_Receive_IT(&huart3,(u8 *)aRxBuffer3, USART3_REC_LEN);
		
		GetRadarData();
		LED0 = !LED0;
		LED1 = !LED0;
	}
}
