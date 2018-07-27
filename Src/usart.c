/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
vu8 USART1_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
u16 USART1_RX_STA=0;       			//接收状态标记	

vu8 USART2_RX_BUF[USART_REC_LEN];	//接收缓冲,最大USART_REC_LEN个字节.
u16 USART2_RX_STA = 0;			//接收状态标记

vu8 USART3_RX_BUF[USART_REC_LEN];	//接收缓冲,最大USART_REC_LEN个字节.
u16 USART3_RX_STA = 0;			//接收状态标记

uint8_t receive = 0;
uint8_t receive2 = 0;		//接收完成标志
uint8_t receive3 = 0;

vu8 aRxBuffer1[USART1_REC_LEN];//HAL库使用的串口接收缓冲
vu8 aRxBuffer2[USART2_REC_LEN];//HAL库使用的串口接收缓冲
vu8 aRxBuffer3[USART3_REC_LEN ];//HAL库使用的串口接收缓冲

	

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */
	//HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();
  
    /**USART1 GPIO Configuration    
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();
  
    /**USART2 GPIO Configuration    
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}

//串口发送1个字符 
//c:要发送的字符
void usart_send_char(u8 c)
{
    while(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_TC)==RESET){}; 
    USART1->DR=c;  
} 
void SendToPc(u8 cmd,u16 data1,u16 data2,u16 data3)
{
	u8 sum =0;
	
	usart_send_char('@');
	//sum += '@';
	
	usart_send_char('^');
	//sum +=  '@';
	
	usart_send_char(cmd);
	///sum += cmd;
	
	usart_send_char((data1>>8)&0xff);
	usart_send_char((data1)&0xff);
	
	usart_send_char((data2>>8)&0xff);
	usart_send_char(data2&0xff);
	
	usart_send_char((data3>>8)&0xff);
	usart_send_char(data3&0xff);
	
	sum = '@'+  '^'+cmd+((data1>>8)&0xff)+((data1)&0xff)+((data2>>8)&0xff)+(data2&0xff)+((data3>>8)&0xff)+(data3&0xff);
	
	usart_send_char(sum);
		
}



//串口1中断服务程序，视觉数据
void myUSART1_IRQHandler(void)
{
	uint8_t Res;
	uint8_t End=0;
	uint8_t CheckSum_1 = 0;
	
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_RXNE) != RESET))  //接收中断
	{
		HAL_UART_Receive(&huart1,&Res,1,1000);//(USART1->DR);	//读取接收到的数据
		
		if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(Res!='z')
			{	
				if(End!='z')
				{
					USART1_RX_BUF[USART1_RX_STA&0X3FFF]=Res ;
					CheckSum_1=Res+ CheckSum_1;
					USART1_RX_STA++;
					if(USART1_RX_STA>(USART_REC_LEN-1))
						USART1_RX_STA=0;//接收数据错误,重新开始接收	 
				}		
				else if(End=='z')
				{
					if( CheckSum_1==Res)
					{	
						USART1_RX_STA|=0x8000;
						receive=1;
					}
					else
						USART1_RX_STA=0;
					
					End = 0;
					CheckSum_1=0;
				}
			}
			else if(Res=='z') 
			{ 
				End='z';

				if( CheckSum_1=='z')
					 CheckSum_1= CheckSum_1+'1';
			}
		} 
	} 
}

uint8_t End=0;
u8 receive_start2 = 0;
uint8_t CheckSum_2 = 0;
void myUSART2_IRQHandler(void)
{
	uint8_t Res;

	
	if((__HAL_UART_GET_FLAG(&huart2,UART_FLAG_RXNE) != RESET))  //接收中断
	{
		HAL_UART_Receive(&huart2,&Res,1,1000);//(USART1->DR);	//读取接收到的数据
		
		if((USART1_RX_STA&0x8000)==0)//接收未完成
		{
			if(Res == 0x55)
			{
				if(receive_start2 == 0)
				{
					USART2_RX_STA=0;
					CheckSum_2 = 0;
					receive_start2 = 1;
				}
			}	
			else if(Res == 0x53)
			{
				if(receive_start2 == 1)
				{
					receive_start2 = 2;
				}
			}
			if(receive_start2 != 0)
			{
				if(USART2_RX_STA < 10)
				{
					USART2_RX_BUF[USART2_RX_STA&0X3FFF]=Res;
					CheckSum_2 += Res;
					USART2_RX_STA++;
				}
				else
				{
					if(Res ==CheckSum_2)
					{
						
							//LED0 = !LED0;
							LED1 = !LED0;
						receive2 = 1;
						USART2_RX_STA|=0x8000;
			
						BasketballRobot.ThetaD = ((float)((USART2_RX_BUF[7]<<8)|aRxBuffer2[6]))/32768*180;
;
						
						BasketballRobot.ThetaR = BasketballRobot.ThetaD * PI / 180 + BasketballRobot.theta_offset;
						
						while(BasketballRobot.ThetaR < 0)
							BasketballRobot.ThetaR  = BasketballRobot.ThetaR + PI + PI;
						
						while (BasketballRobot.ThetaR > 2 * PI)
							BasketballRobot.ThetaR = BasketballRobot.ThetaR - PI - PI;
						
						while(BasketballRobot.ThetaD < 0)
							BasketballRobot.ThetaD  = BasketballRobot.ThetaD + 360;
						
						while (BasketballRobot.ThetaD >360)
							BasketballRobot.ThetaD = BasketballRobot.ThetaD - 360;
						printf("%.2f\n",BasketballRobot.ThetaD);
						receive2 = 0;
						USART2_RX_STA =000;
					}
					receive_start2 = 0;
				}
			}
		} 
	} 
}

void myUSART3_IRQHandler(void)
{
	uint8_t end=0;
	uint8_t  CheckSum_3 = 0;
	uint8_t Res;
	
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{	

		HAL_UART_Receive(&huart2,&Res,1,1000);//(USART1->DR);	//读取接收到的数据
		
		if((USART3_RX_STA&0x8000)==0)//接收未完成
		{
			if(Res!='z')
			{	
				if(end!='z')
				{
					USART3_RX_BUF[USART3_RX_STA&0X3FFF]=Res ;
					//USART_SendData(USART3, USART3_RX_BUF[USART3_RX_STA&0X3FFF]);
					// CheckSum_3=Res+ CheckSum_3;
					
					USART3_RX_STA++;
					if(USART3_RX_STA>(USART_REC_LEN-1))
						USART3_RX_STA=0;//接收数据错误,重新开始接收	 
				}		
				else if(end=='z')
				{
					USART3_RX_STA|=0x8000;
					receive3=1;
					
					end=0;
				}
			}
			else if(Res=='z') 
			{ 
				if( USART3_RX_STA ==7)
					end='z';
				else
					end=0;
				USART3_RX_STA=0;
			}
		} 
	}
	
}
	
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
