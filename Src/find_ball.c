#include "find_ball.h"

u8 ball;
void FindBasketball(void)
{
	ball = 1;
	HAL_UART_Transmit(&huart1,&ball,1,1000);
}


void FindVolleyball(void)
{
	ball = 3;
	HAL_UART_Transmit(&huart1,&ball,1,1000);
}


//�Ӿ�����
//����4������
void FindBall_vision(u8 ball)
{
	float w=200;
	
	u8 time = 1;
	
	float theta = BasketballRobot.ThetaR,D_theta = 0;
	
	switch(ball)
	{
		case 1:
			FindBasketball();
			delay_ms(10000);
			FindBasketball();
			delay_ms(10000);
			FindBasketball();
			break;
		case 3:
			FindVolleyball();
			delay_ms(10000);
			FindVolleyball();
			delay_ms(10000);
			FindVolleyball();
			break;
	}
	
	SetPWM(0,0,0);
	
	LCD_Show_pwm();
	
	//while(receive != 1);
	USART1_RX_STA=0;
	receive=0;
	do{
		//�ȴ����ݽ������
		while(receive != 1);
		
		//����������Ч
		if(!GetVisionData())
		{	
			if(time == 0);
				
			//�������
			else if(time++ <5)
			{				
				//SetPWM(0,0,0);
				continue;
			}
			//������κ���δ�ҵ���
			else if(time !=0)
				time = 0;
		}
		//������Ч
		else
			time = 1;
		
		//û�ҵ���,��ת
		if(time == 0)
		{
			D_theta = BasketballRobot.ThetaR - theta;
			if((D_theta > PI/6.0f && D_theta < PI) || (D_theta < -PI && D_theta > -PI*11.0f/6.0f))
			{
				w = -200;
			}
			if((D_theta < -PI/6.0f && D_theta > -PI) || (D_theta > PI && D_theta < PI*11.0f/6.0f))
			{
				w = 200;
			}
			GetMotorVelocity(0,0,w);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			
			LCD_Show_pwm();
		}
		
		//��Ч����
		else if(Vision.Depth > 4000)
		{
			GetMotorVelocity(0,0,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		
		else if((Vision.X< VISION_MID-30) && Vision.Depth>1300)
		{
			GetMotorVelocity_Self(-5,10,0);  //ԭ�� -50 10 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if((Vision.X > VISION_MID+30) && Vision.Depth > 1300)
		{
			GetMotorVelocity_Self(5,10,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Vision.Depth > 1300)
		{
			GetMotorVelocity_Self(0,20,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if((Vision.X< VISION_MID-20) && Vision.Depth > 700)
		{
			GetMotorVelocity_Self(-4,0,0);  //ԭ��-40 0 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if((Vision.X > VISION_MID+20) && (Vision.Depth >700))
		{
			GetMotorVelocity_Self(4,0,0); //ԭ��4 0 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Vision.Depth>700)
		{
			GetMotorVelocity_Self(0,12,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Vision.X< VISION_MID-30)
		{
			GetMotorVelocity_Self(-3,0,0); //ԭ��-30 0 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);;
		}
		else if(Vision.X > VISION_MID+30)
		{
			GetMotorVelocity_Self(3,0,0); //ԭ��30 0 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Vision.X <= VISION_MID+30 && Vision.X > VISION_MID+10)
		{
			GetMotorVelocity_Self(1.5,0,0); //ԭ��15 0 0 
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Vision.X >= VISION_MID-30 && Vision.X < VISION_MID -10)
		{
			GetMotorVelocity_Self(-1.5,0,0); //ԭ��-15 0 0 
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else 
		{
				SetPWM(0,0,0);
			
				Robot_armDown();
				GetMotorVelocity_Self(0,7,0);
				SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
				break;
			
		}
	}while(1);
	
	GetInfraredState();

	SetPWM(0,0,0);
	
	Robot_armUp();
	LCD_Show_pwm();
	
}

//���ü�������
//����������ɫ
//����3m����
void FindBall_radar(void)
{
	float w=300;
	float theta = BasketballRobot.ThetaD,D_theta = 0;	
	
	SetPWM(0,0,0);
	LCD_Show_pwm();
	do{
		while(receive3 != 1);
		
		if(!GetRadarData())
		{	
			
			SetPWM(0,0,0);
		//	continue;
		}
		LED0 = !LED0;

		if(Radar.Distance < 10)
			continue;
		if(Radar.Distance > 3000)
		{			
			D_theta = BasketballRobot.ThetaR - theta;
			if((D_theta > PI/6.0f && D_theta < PI) || (D_theta < -PI && D_theta > -PI*11.0f/6.0f))
			{
				w = -300;
			}
			if((D_theta < -PI/6.0f && D_theta > -PI) || (D_theta > PI && D_theta < PI*11.0f/6.0f))
			{
				w = 300;
			}
			GetMotorVelocity(0,0,w);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Angle< RADAR_MID-15)
		{
			GetMotorVelocity_Self(0,0,200);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Angle > RADAR_MID+15)
		{
			GetMotorVelocity_Self(0,0,-200);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Distance > 1000)
		{
			GetMotorVelocity_Self(0,30,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if((Radar.Angle< RADAR_MID-10) && Radar.Distance >700)
		{
			GetMotorVelocity_Self(0,0,150);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if((Radar.Angle > RADAR_MID+10) && Radar.Distance >700)
		{
			GetMotorVelocity_Self(0,0,-150);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Distance>700)
		{
			GetMotorVelocity_Self(0,20,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Angle< RADAR_MID-5)
		{
			GetMotorVelocity_Self(-8,0,0); //ԭ��-80 0 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Angle > RADAR_MID+5)
		{
			GetMotorVelocity_Self(8,0,0);//ԭ��80 0 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else 
		{
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			Robot_armDown();
			GetMotorVelocity_Self(0,7,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			if(Radar.Distance < 350)
				break;
		}
	}while(1);
	
	GetInfraredState();	
	
	SetPWM(0,0,0);
	
	Robot_armUp();
	LCD_Show_pwm();
	
}

//�Ӿ����״�������
//����4m����
void FindBall_VandR(u8 ball)
{
	
	float w=200;
	u8 time = 1;
	float theta = BasketballRobot.ThetaR,D_theta = 0;
	switch(ball)
	{
		case 1:
			FindBasketball();
			delay_ms(10000);
			FindBasketball();
			delay_ms(10000);
			FindBasketball();
			break;
		case 3:
			FindVolleyball();
		
			delay_ms(10000);
		
			FindVolleyball();
			delay_ms(10000);
			FindVolleyball();
		
			break;


	}
	SetPWM(0,0,0);
	LCD_Show_pwm();
	
	//����Ӿ���������
	while(receive != 1);
	USART1_RX_STA=0;
	receive=0;
	
	//��մ��ڽ������ݻ���   
	receive3 = 0;
	USART3_RX_STA = 0;
	
	SetPWM(0,0,0);
	LCD_Show_pwm();
	
	//��ֹ��Ч����
	
	while(receive3 != 1);
	receive3 = 0;
	USART3_RX_STA = 0;

	
	do{
		while(receive != 1);
		while(receive3 != 1);
		
		if(!GetVisionData())
		{	
			if(time == 0)
			{
			}
			else if(time++ <5)
			{
				SetPWM(0,0,0);
				continue;
			}
			else if(time !=0)
				time = 0;
		}
		else
			time = 1;
		
		GetRadarData();
		
		if(time == 0)
		{
			D_theta = BasketballRobot.ThetaR - theta;
			if((D_theta > PI/6.0f && D_theta < PI) || (D_theta < -PI && D_theta > -PI*11.0f/6.0f))
			{
				w = -200;
			}
			if((D_theta < -PI/6.0f && D_theta > -PI) || (D_theta > PI && D_theta < PI*11.0f/6.0f))
			{
				w = 200;
			}
			GetMotorVelocity(0,0,w);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			LCD_Show_pwm();
		}
		else if(Radar.Distance>4000)
		{
		
			SetPWM(0,0,0);
		}
		else if((Radar.Angle< VISION_MID-30) && Radar.Distance>1000)
		{
			GetMotorVelocity_Self(-8,16,0); //ԭ��-5 10 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if((Radar.Angle > VISION_MID+30) && Radar.Distance > 1000)
		{
			GetMotorVelocity_Self(8,16,0);//ԭ�� 5 10 0
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else if(Radar.Distance > 1000)
		{
			GetMotorVelocity_Self(0,30,0);
			SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		}
		else
			break;
	}while(1);
	
	if(Radar.Distance>800)
	{
		LCD_ShowString(30+200,500,200,16,16,"View!");
		//�״����ݲ�ͬʱ�������Ӿ�����Ѱ��
		while(1)
		{
			while(receive != 1);
			
			GetVisionData();
			GetRadarData();
			
			if(Radar.Distance<800)
				break;

			if(!GetVisionData())
				SetPWM(0,0,0);
					
			if(Radar.Angle< VISION_MID-20 )
			{
				GetMotorVelocity_Self(-6,0,0); //ԭ��-4 0 0
				SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			}
			else if(Radar.Angle > VISION_MID+20)
			{
				GetMotorVelocity_Self(6,0,0);//ԭ��4 0 0
				SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			}
					
			else if(Radar.Distance>700)
			{
				GetMotorVelocity_Self(0,20,0);
				SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			}
					
			else if(Radar.Angle< VISION_MID-30)
			{
				GetMotorVelocity_Self(-6,0,0); //ԭ��-3 0 0
				SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
			}
					else if(Radar.Angle > VISION_MID+30)
					{
						GetMotorVelocity_Self(6,0,0);//ԭ�� 3 0 0
						SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
						 
						 
					}
					else if(Radar.Angle <= VISION_MID+30 && Radar.Angle > VISION_MID+10)
					{
						GetMotorVelocity_Self(3,0,0); //ԭ�� 1.5 0 0 
						SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
						 
						 
					}
					else if(Radar.Angle >= VISION_MID-30 && Radar.Angle < VISION_MID -10)
					{
						GetMotorVelocity_Self(-3,0,0);//ԭ��-1.5 0 0
						SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
						 
						 
					}
					else 
					{
							SetPWM(0,0,0);
							Robot_armDown();
							GetMotorVelocity_Self(0,7,0);
							SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
							 
							 
							break;
					}
				}
			}
			
		

	if(Radar.Distance<=800)
			{	
				LCD_ShowString(30+200,500,200,16,16,"Radar!");
					//�״�������ͬ�������״�����Ѱ��
				while(1)
				{
					while(receive3 != 1);
					GetRadarData();
					GetVisionData();

					if(!GetRadarData())
					{	
					GetMotorVelocity_Self(0,0,0);
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					}
					
					else if((Radar.Angle< RADAR_MID-10) && Radar.Distance >700)
					{
					GetMotorVelocity_Self(-10,0,0);//ԭ��-10 0 0
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					}
					else if((Radar.Angle > RADAR_MID+10) && Radar.Distance >700)
					{
					GetMotorVelocity_Self(10,0,0); //ԭ��10 0 0
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					}
					else if(Radar.Distance>700)
					{
					GetMotorVelocity_Self(0,20,0); //ԭ�� 0 14 0
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					}
					else if(Radar.Angle< RADAR_MID-5)
					{
					GetMotorVelocity_Self(-10,0,0);//ԭ��-10 0 0
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					}
					else if(Radar.Angle > RADAR_MID+5)
					{
					GetMotorVelocity_Self(10,0,0);//ԭ��10 0 0
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					}
				/*	else if(Radar.Angle <= RADAR_MID+5 && Radar.Angle > RADAR_MID+2)
					{
						GetMotorVelocity_Self(1,0,0);
						SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
						 
						 
					}
					else if(Radar.Angle >= RADAR_MID-5 && Radar.Angle < RADAR_MID -2)
					{
						GetMotorVelocity_Self(-1,0,0);
						SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
						 
						 
					}
					*/
					else 
					{
					SetPWM(0,0,0);
					Robot_armDown();
					GetMotorVelocity_Self(0,7,0);//ԭ��0 7 0 
					SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
					 
					 
					if(Radar.Distance < 350)
						break;
					}	
				
				}
			}

						
	GetInfraredState();
	SetPWM(0,0,0);
			
	Robot_armUp();

			
	LCD_Show_pwm();
	LCD_ShowString(30+200,460,200,16,16,"       ");
}