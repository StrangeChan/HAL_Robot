
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fsmc.h"

/* USER CODE BEGIN Includes */
#include "control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
	u8 zhongquan_case;
	u8 changdi;
	u8 chengxu;
	u8 sanfen_case;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	u8 key = 0;					//按键值
	//u8 chengxu = 0;				//程序选择
	u8 flag=0;
	u8 qiu = 0;				//找球
	
	u8 findballtime = 0;			//找球时调整角度次数
	u8 i;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
		
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	delay_init(168);  
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM9_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_FSMC_Init();
  MX_TIM12_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
	LCD_Show_Title();
	Control_Init();
	IMU_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	LED0 =1;
	LED1 =1;

  while (1)
  {
	  
	  
	    key = Remote_Scan();
		flag = 0;
		key = 0;
		chengxu = 0;
	    changdi = 0;
		//选择程序
		while(1)
		{
//			printf("%x  \r\n",huart2.RxState);
//			printf("errr%x  \n",huart2.ErrorCode);
//			delay_ms(100);
			LCD_ShowString(30+200,400,200,16,16,"chengxu:");
			key = Remote_Scan();
			//key = KEY_Scan(0);
			switch(key)
			{
				case 0:		//没有按键按下
					
					break;
				case KEY_RIGHT:		//右
					LCD_ShowString(30+200,400,200,16,16,"qiu:    ");
					flag = 1;
					break;
				case KEY_DOWN:		//下
					LCD_ShowString(30+200,400,200,16,16,"chengxu-");
					if(chengxu != 0)
						chengxu--;
					break;
				case KEY_LEFT:		//左
					LCD_ShowString(30+200,400,200,16,16,"clear   ");
					chengxu = 0;
					break;
				case KEY_UP:		//上
					LCD_ShowString(30+200,400,200,16,16,"chengxu+");
					chengxu++;
					break;
			}
			
			LCD_ShowNum(30+200+48+8+10,320,chengxu,4,16);
			
			if(flag)
				break;
		}
		
		flag = 0;
		key = 0;
		
		//选择球
		while(1)
		{
			key = Remote_Scan();
			//key = KEY_Scan(0);
			switch(key)
			{
				case 0:		//没有按键按下
					
					break;
				case KEY_RIGHT:		//右
					LCD_ShowString(30+200,400,200,16,16,"changdi   ");
					flag = 1;
					break;
				case KEY_DOWN:		//下
					LCD_ShowString(30+200,400,200,16,16,"qiu-");
					if(qiu != 0)
						qiu--;
					break;
				case KEY_LEFT:		//左
					LCD_ShowString(30+200,400,200,16,16,"clear   ");
					qiu = 0;
					break;
				case KEY_UP:		//上
					LCD_ShowString(30+200,400,200,16,16,"qiu+");
					qiu++;
					break;

			}
			
			LCD_ShowNum(30+200+48+8+10,340,qiu,4,16);
			
			if(flag)
				break;
		}
		
		flag = 0;
		key = 0;
		while(1)
		{
			key = Remote_Scan();
			switch(key)
			{
				case 0:		//没有按键按下
					break;
				case KEY_RIGHT:		//右
					LCD_ShowString(30+200,400,200,16,16,"start   ");
					flag = 1;
					break;
				case KEY_DOWN:		//下
					LCD_ShowString(30+200,400,200,16,16,"changdi-");
					if(changdi != 0)
						changdi--;
					break;
				case KEY_LEFT:		//左
					LCD_ShowString(30+200,400,200,16,16,"clear   ");
					changdi=0;
					break;
				case KEY_UP:		//上
					LCD_ShowString(30+200,400,200,16,16,"changdi+");
					changdi++;
					break;
				case KEY_POWER:
					flag = 1;
					chengxu = 99;
					break;
			}
			
			LCD_ShowNum(30+200+48+8+10,360,changdi,4,16);
			
			if(flag)
				break;
		}

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		while(1){
		switch(chengxu)
		{
			case 0:	    	//测试程序
				switch(qiu)
				{
//					case 0:
//						RobotRotate(180);
//						//printf("err\r\n");
//						//顺时针180°
//						break;
//					case 1:
//						//机械臂下降
//						Robot_armDown();
////						2高电平往下，接红线，正转
////						__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,300);
////						__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,4000);
//						LED0 = !LED0;
//						break;
//					case 2:
//						//机械臂上升
//						Robot_armUp();
//						LED0 = !LED0;
//						break;
//					case 3:
//						//红外测试
//						GetInfraredState();
//						LED0 = !LED0;
//						break;
//					case 4:
//						//限位开关测试6
//						if(LimitSwitchDowm == 1)
//							LED1 = 0;
//						else
//							LED1 = 1;
//						if(LimitSwitchUp == 1)
//							LED0 = 0;
//						else
//							LED0 = 1;
//						break;
//					case 5:
//						//避障测试
//						RobotGoAvoidance();
//						LED0 = !LED0;
//						break;
//					case 6:
//						//视觉测试
//						FindBall_vision(qiu);
//						LED0 = !LED0;
//						break;
					case 1:
						RobotGoTo(0,2,0);
//						RobotRotate(0);
//					  RobotGoTo(3,2,0);
//						RobotGoTo(3,3.5,0);
//						RobotGoTo(0,2,-110);
//						RobotGoTo(0,0,0);
						break;
				}
				break;
			case 1:			//传球第一回合				
				switch(changdi)
				{
					case 0:			//上场
						//延时10s
						for(i = 0 ;i <10;i++){
							SetPWM(0,0,0);
							delay_ms(1000);
						}
			
						RobotGoTo(0.0-Correction_X,2.0+Correction_Y,-110);
						if(!DownShotUp())
							break;
						RobotRotate(0.0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有
						if(findballtime==0){			//没找到球
							RobotGoTo(-3.0-Correction_X,2.0+Correction_Y,0);
							FindBall_VandR(qiu);
						}				
				
						RobotGoTo(0-Correction_X,2+Correction_Y,-110);
					
						if(!DownShotUp())
							break;
				
						RobotGoTo(0,1+Correction_Y,-180);
				
						//加视觉找框回位
						break;
					case 1:			//下场
						//延时10s
						for(i = 0 ;i <10;i++){
							SetPWM(0,0,0);
							delay_ms(1000);
						}
			
						RobotGoTo(0+Correction_X,2+Correction_Y,110);
						if(!DownShotUp())
							break;
						RobotRotate(0);
						FindBall_VandR(qiu);
						
			
						//判断是否找到球，如果没有
						if(findballtime==0){			//没找到球
							RobotGoTo(3+Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}				
				
						RobotGoTo(0+Correction_X,2+Correction_Y,110);
					
						if(!DownShotUp())
							break;
				
						RobotGoTo(0,1+Correction_Y,180);
				
						//加视觉找框回位
						break;
				}				
				break;
			case 2:			//传球第二回合
				switch(changdi)
				{
					case 0:			//上场
						RobotGoTo(0-Correction_X,2+Correction_Y,0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有
						if(findballtime==0){		//没找到球
							RobotGoTo(-3-Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}
				
						RobotGoTo(-6-Correction_X,2+Correction_Y,0);
						RobotGoTo(-6-Correction_X,4+Correction_Y,-210);
				
						if(!DownShotUp())
							break;
				
						RobotGoTo(-5-Correction_X,2+Correction_Y,-45);
						FindBall_VandR(qiu);
						RobotGoTo(-6-Correction_X,4+Correction_Y,-210);
				
						if(!DownShotUp())
							break;
				
						RobotGoTo(0,1+Correction_Y,-180);
				
						//加视觉找框回位
						break;
					case 1:			//下场
						RobotGoTo(0+Correction_X,2+Correction_Y,0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有
						if(findballtime==0){		//没找到球
							RobotGoTo(3+Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}
				
						RobotGoTo(6+Correction_X,2+Correction_Y,0);
						RobotGoTo(6+Correction_X,4+Correction_Y,210);
				
						if(!DownShotUp())
							break;
				
						RobotGoTo(5+Correction_X,2+Correction_Y,45);
						FindBall_VandR(qiu);
						RobotGoTo(6+Correction_X,4+Correction_Y,210);
				
						if(!DownShotUp())
							break;
				
						RobotGoTo(0+Correction_X,1+Correction_Y,180);
				
						//加视觉找框回位
						break;
				}				
				break;
			case 3:			//传球第三回合
				switch(changdi)
				{
					case 0:			//上场
						RobotGoTo(-5-Correction_X,2+Correction_Y,-45);
						FindBall_VandR(qiu);
				
						//记录铲球点A的位置
						BasketballRobot.PX=BasketballRobot.X;
						BasketballRobot.PY=BasketballRobot.Y;
				
						RobotGoTo(-5-Correction_X,2+Correction_Y,-45);
						RobotGoTo(-6-Correction_X,4+Correction_Y,-210);
						if(!DownShotUp())
							break;
				
						//原路返回
						RobotGoTo(-5-Correction_X,2+Correction_Y,-45);
						RobotGoTo(BasketballRobot.PX-Correction_X,BasketballRobot.PY+Correction_Y,-210);
			
						RobotGoTo(-6.75-Correction_X,2+Correction_Y,-90);
						//FindBall_VandR(qiu);
						FindBall_radar();
			
						//原路返回
						RobotGoTo(BasketballRobot.PX-Correction_X,BasketballRobot.PY+Correction_Y,-210);
						RobotGoTo(-5-Correction_X,2+Correction_Y,-45);
				
						RobotGoTo(-6-Correction_X,4+Correction_Y,-210);
						if(!DownShotUp())
							break;
				
						RobotGoTo(0-Correction_X,1+Correction_Y,-180);
			
						//加视觉找框回位
						break;
					case 1:			//下场
						RobotGoTo(5+Correction_X,2+Correction_Y,45);
						FindBall_VandR(qiu);
				
						//记录铲球点A的位置
						BasketballRobot.PX=BasketballRobot.X;
						BasketballRobot.PY=BasketballRobot.Y;
				
						RobotGoTo(5+Correction_X,2+Correction_Y,45);
						RobotGoTo(6+Correction_X,4+Correction_Y,210);
						if(!DownShotUp())
							break;
				
						//原路返回
						RobotGoTo(5+Correction_X,2+Correction_Y,45);
						RobotGoTo(BasketballRobot.PX+Correction_X,BasketballRobot.PY+Correction_Y,210);
			
						RobotGoTo(6.75+Correction_X,2+Correction_Y,90);
						//FindBall_VandR(qiu);
						FindBall_radar();
			
						//原路返回
						RobotGoTo(BasketballRobot.PX+Correction_X,BasketballRobot.PY+Correction_Y,210);
						RobotGoTo(5+Correction_X,2+Correction_Y,45);
				
						RobotGoTo(6+Correction_X,4+Correction_Y,210);
						if(!DownShotUp())
							break;
				
						RobotGoTo(0+Correction_X,1+Correction_Y,180);
			
						//加视觉找框回位
						break;
				}					
				break;
			case 4:			//投篮第一回合
				switch(changdi)
				{
					case 0:			//上场
						RobotGoTo(-4-Correction_X,3.7+Correction_Y,-90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
			
						RobotGoTo(0-Correction_X,2+Correction_Y,0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有			
						if(findballtime==0){		//没找到球
							RobotGoTo(-3-Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}
				
						RobotGoTo(-9-Correction_X,3.7+Correction_Y,-90);
				
						//雷达找框
						FindBasketry();
				
						if(!DownShotUp())
							break;
						break;
					case 1:			//下场
						RobotGoTo(4+Correction_X,3.7+Correction_Y,90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
			
						RobotGoTo(0+Correction_X,2+Correction_Y,0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有			
						if(findballtime==0){		//没找到球
							RobotGoTo(3+Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}
				
						RobotGoTo(9+Correction_X,3.7+Correction_Y,90);
				
						//雷达找框
						FindBasketry();
				
						if(!DownShotUp())
							break;
						break;
				}
				break;
			case 5:			//投篮第二回合
				switch(changdi)
				{
					case 0:			//上场
						RobotGoTo(-5-Correction_X,2+Correction_Y,-45);
						FindBall_VandR(qiu);
			
						//记录铲球点A的位置
						BasketballRobot.PX=BasketballRobot.X;
						BasketballRobot.PY=BasketballRobot.Y;
			
						RobotGoTo(-9-Correction_X,3.7+Correction_Y,-90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
			
						//RobotGoTo(A);
						RobotGoTo(BasketballRobot.PX-Correction_X,BasketballRobot.PY+Correction_Y,-90);
			
						RobotGoTo(0-Correction_X,2+Correction_Y,0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有			
						if(findballtime==0){		//没找到球
							RobotGoTo(-3-Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}
				
						RobotGoTo(BasketballRobot.PX-Correction_X,BasketballRobot.PY+Correction_Y,0);
						RobotGoTo(-9-Correction_X,3.7+Correction_Y,-90);
				
						//雷达找框
						FindBasketry();
				
						if(!DownShotUp())
							break;
						break;
					case 1:			//下场
						RobotGoTo(5+Correction_X,2+Correction_Y,45);
						FindBall_VandR(qiu);
			
						//记录铲球点A的位置
						BasketballRobot.PX=BasketballRobot.X;
						BasketballRobot.PY=BasketballRobot.Y;
			
						RobotGoTo(9+Correction_X,3.7+Correction_Y,90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
			
						//RobotGoTo(A);
						RobotGoTo(BasketballRobot.PX+Correction_X,BasketballRobot.PY+Correction_Y,90);
			
						RobotGoTo(0+Correction_X,2+Correction_Y,0);
						FindBall_VandR(qiu);
			
						//判断是否找到球，如果没有			
						if(findballtime==0){		//没找到球
							RobotGoTo(3+Correction_X,2+Correction_Y,0);
							FindBall_VandR(qiu);
						}
				
						RobotGoTo(BasketballRobot.PX+Correction_X,BasketballRobot.PY+Correction_Y,0);
						RobotGoTo(9+Correction_X,3.7+Correction_Y,90);
				
						//雷达找框
						FindBasketry();
				
						if(!DownShotUp())
							break;
						break;
				}					
				break;
			case 6:			//投篮第三回合
				switch(changdi)
				{
					case 0:			//上场
						RobotGoTo(-5-Correction_X,2-Correction_Y,-30);
						FindBall_VandR(qiu);
			
						RobotRotate(-90);
						RobotGoTo(-9-Correction_X,3.7+Correction_Y,-90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
			
						RobotRotate(-240);
						FindBall_VandR(qiu);
						RobotGoTo(-9-Correction_X,3.7+Correction_Y,-90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
						break;
					case 1:			//下场
						RobotGoTo(5+Correction_X,2+Correction_Y,30);
						FindBall_VandR(qiu);
			
						RobotRotate(90);
						RobotGoTo(9.0f+Correction_X,3.7+Correction_Y,90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
			
						RobotRotate(240);
						FindBall_VandR(qiu);
						RobotGoTo(9+Correction_X,3.7+Correction_Y,90);
			
						//雷达找框
						FindBasketry();
			
						if(!DownShotUp())
							break;
						break;
				}				
				break;
				
		}
					
	}
	  
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
