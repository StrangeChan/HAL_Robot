#include "interrupt_callback.h"
#include "control.h"
u8 count = 0;
//定时器更新（溢出）中断回调函数
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
		//printf(" 1  %d   \r\n",htim5.Instance->CNT);
//			printf(" 2  %d    \r\n",htim5.Instance->CNT);
		//GetPosition();
		//printf("  %f    %f \r\n",BasketballRobot.X,BasketballRobot.Y);
//		
		GetYaw();
		GetPosition();
		if(count++==1)
			LCD_Show_position();
		
		if(count %50== 2)
			LCD_Show_lcj();
		
//		if(count == 2)
//			printf("tim\r\n");
		if(count == 10)
			LCD_Show_v();
		
//		printf("1： %d\r\n",htim5.Instance->CNT);
//		MPU9250_Read();
//		Mahony_update(GYO[0] * Factor,GYO[1] * Factor,GYO[2] * Factor,
//								Acc[0],Acc[1],Acc[2],
//								Mag_[0],Mag_[1],Mag_[2]);
//		Mahony_computeAngles();
//		BasketballRobot.ThetaD = getYaw();
//		//GetPosition();
//		printf("2 ：%d  %f \r\n",htim5.Instance->CNT,getYaw());
		
		//delay_ms(1);
		//printf(" 3  %d    \r\n",htim5.Instance->CNT);
		//printf("%f\n",BasketballRobot.ThetaD);
		//SendToPc(1,BasketballRobot.X*1000+14000,BasketballRobot.Y*1000+14000,BasketballRobot.ThetaD);
		//count++;
	}
}




//定时器输入捕获中断回调函数
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)//捕获中断发生时执行
{
	if(htim->Instance==TIM1)
	{
		myRemoteCcCallback();
	}
	
}
//uint8_t sum = 0, i = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(huart->Instance==USART1)//如果是串口1
	{
		
		//GetVisionData();
		receiveRadarData();
		LED0 = !LED0;
		LED1 = !LED0;
	}
	if(huart->Instance==USART2)//如果是串口2
	{	
//		receiveIMUData();
//		GetYaw();	
		
	}
	if(huart->Instance==USART3)//如果是串口3
	{
		
		receiveRadarData();
		
		//GetRadarData();
	
		//LED0 = !LED0;
//		LED1 = !LED0;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
	
	HAL_UART_MspDeInit(huart);
	HAL_UART_MspInit(huart);
	
	//__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
	/*启动串口DMA接收*/
	HAL_UART_Receive_DMA(&huart2,(u8 *)aRxBuffer2,USART2_REC_LEN);
}

