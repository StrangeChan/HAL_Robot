#include "control.h"

struct ROBOT BasketballRobot;

struct RADAR Radar;
struct VISION Vision;

void Control_Init(void)
{
	BasketballRobot.X = 0;		//机器人在坐标系中x坐标
	BasketballRobot.Y = 0;		//机器人在坐标系中y坐标
	BasketballRobot.ThetaR = 0;	//机器人正方向和y轴夹角
	BasketballRobot.ThetaR = 0;	//机器人正方向和y轴夹角
	BasketballRobot.Vx = 0;		//机器人在坐标系x方向速度
	BasketballRobot.Vy = 0;		//机器人在坐标系y方向速度
	BasketballRobot.W = 0;		//机器人角速度，顺时针正方向
	
	BasketballRobot.w[1] = 0;		//第一个编码器实际计数
	BasketballRobot.w[2] = 0;		//第二个编码器实际计数
	BasketballRobot.w[0] = 0;		//第三个编码器实际计数
	
	BasketballRobot.v[1] = 0;		//第一个编码器所得速度
	BasketballRobot.v[2] = 0;		//第二个编码器所得速度
	BasketballRobot.v[0] = 0;		//第三个编码器所得速度
	
	BasketballRobot.LastTheta = 0;	//上一时刻，机器人theta角
	BasketballRobot.theta_offset = 0;	//角度偏差矫正
	
	//雷达、视觉数据清空
	Radar.Angle = 0;
	Radar.Distance = 0;
	
	Vision.Depth = 0;
	Vision.X = 0;
	
	//开启外设
	HAL_UART_Receive_IT(&huart1,(u8 *)aRxBuffer1, USART1_REC_LEN);
	HAL_UART_Receive_IT(&huart2,(u8 *)aRxBuffer2, USART2_REC_LEN);
	HAL_UART_Receive_IT(&huart3,(u8 *)aRxBuffer3, 1);	
	
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);		//开始捕获 TIM1 的通道 1，红外遥控
	HAL_TIM_Base_Start_IT(&htim1);					//使能更新中断，红外遥控
	HAL_TIM_Base_Start_IT(&htim5);					//主定时器，获取姿态信息
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);	//开启解码器通道
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim8,TIM_CHANNEL_ALL);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);		//开启 底盘电机PWM 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);		//开启铲球电机PWM 
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_2);
	
	HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
	
	
	SetPWM(0,0,0);
	
//	MPU_Init();			//MPU6050初始化
//	MPU_Init();
//	MPU_Init();	
	
}



//电机速度转换成PWM数值，原理看电机驱动板手册
//计算公式： V = Vmax *（占空比*100 – 50） /50
static void Velocity2PWM(float *V)
{
	*V=1000 - *V;//*V+=1000;
	if(*V>=1900)
		*V=1900;
	if(*V<=100)
		*V=100;
}

//设置三个轮子PWM
//V1:	电机1速度
//V2:	电机2速度
//V3;	电机3速度
void SetPWM(float V1,float V2,float V3)
{
	BasketballRobot.Velocity[0] = V1;
	BasketballRobot.Velocity[1] = V2;
	BasketballRobot.Velocity[2] = V3;
	
	//转换
	Velocity2PWM(&V1);
	Velocity2PWM(&V2);
	Velocity2PWM(&V3);
	
	//向CCR1寄存器写值   发出PWM波
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,(uint32_t)V1);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(uint32_t)V2);
	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,(uint32_t)V3);
	
}

//给定球场坐标速度求得轮子的速度
//vx：球场坐标的x轴速度
//vy：球场坐标的y轴速度
//w:机器人原地旋转的角速度
//注意最大速度！
void GetMotorVelocity(float vx,float vy,float w)
{
	u8 i,j,k;
	float L[3][3];
	float theta[3][3];
	float V[3];
	float tem[3][3];
					
	//v(PWM)=L*Theta*V
	//   cos(60)	sin(60)	-MOTOR_L
	//L= cos(180) 	sin(180)	-MOTOR_L
	//   cos(-60)	sin(-60)	-MOTOR_L
	L[0][0] =  0.5;					L[0][1] =  0.8660254037844386;		L[0][2] = -MOTOR_L;
	L[1][0] = -1;						L[1][1] =  0;						L[1][2] = -MOTOR_L;
	L[2][0] =  0.5;					L[2][1] = -0.8660254037844386;		L[2][2] = -MOTOR_L;
	//		cos(theta)	sin(theta)	0
	//theta= -sin(theta)	cos(theta) 	0
	//		 	0			0		1
	theta[0][0]= cos(BasketballRobot.ThetaR);	theta[0][1] = sin(BasketballRobot.ThetaR);	theta[0][2] = 0;
	theta[1][0]= -theta[0][1];				theta[1][1] = theta[0][0];				theta[1][2] = 0;
	theta[2][0]= 0;						theta[2][1] = 0;						theta[2][2] = 1;
	//V
	V[0] = -vx*10;
	V[1] = -vy;
	V[2] = -w;
	
	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			tem[i][j] = 0;
			for(k=0;k<3;k++)
				tem[i][j] += L[i][k] * theta[k][j];			
		}
	}
	
	for(i=0;i<3;i++)
	{
		BasketballRobot.Velocity[i] = 0;
		for(j=0;j<3;j++)
			BasketballRobot.Velocity[i] += tem[i][j]*V[j];
	}
	
	LCD_Show_pwm();
}

//给自身坐标系速度求得轮子的速度
//vx：自身坐标标的x轴速度
//vy：自身坐标的y轴速度
//w:  机器人原地旋转的角速度
//注意最大速度！
void GetMotorVelocity_Self(float vx,float vy,float w)
{
	u8 i,j;
	float L[3][3];
	float V[3];
					
	//v(PWM)=L*Theta*V
	//   cos(60)	sin(60)	-MOTOR_L
	//L= cos(180) 	sin(180)	-MOTOR_L
	//   cos(-60)	sin(-60)	-MOTOR_L
	L[0][0] =  0.5;					L[0][1] =  0.8660254037844386;		L[0][2] = -MOTOR_L;
	L[1][0] = -1;						L[1][1] =  0;						L[1][2] = -MOTOR_L;
	L[2][0] =  0.5;					L[2][1] = -0.8660254037844386;		L[2][2] = -MOTOR_L;

	//V
	V[0] = -vx*10;
	V[1] = -vy;
	V[2] = -w;
	
	
	for(i=0;i<3;i++)
	{
		BasketballRobot.Velocity[i] = 0;
		for(j=0;j<3;j++)
			BasketballRobot.Velocity[i] += L[i][j]*V[j];
	}
	
	LCD_Show_pwm();
}


//获取红外开关状态
void GetInfraredState(void)
{	// return GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_9);
	while(1)
	{
		if(!INFRARED)
			break;
	}
}


//机械臂下降
void Robot_armDown(void)
{
	//原来板子：3000
	//V1.0：1750
	u16 i,t;
	u16 W=2700;
	u16 nms=2000;

	
	if(LimitSwitchDowm==1)
	{
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MOTOR_STATIC_1);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);

		return;
	}
	//EXTIX_Enable(1);
	#ifdef ZQD_DEBUG
	BEEP = 1;
	#endif
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MOTOR_STATIC_1);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,W);
	
	LED1 = 1;
	for(i=0;i<nms;i++)
	{	  
		if(LimitSwitchDowm == 1)
		{	
			for(t=0;t<0xff;t++);
			if(LimitSwitchDowm==1)
			{
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,3970);
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);
				break;
			}
		}
		for(t=0;t<0x4fff;t++)
			if(LimitSwitchDowm == 1)
				break;
	}
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MOTOR_STATIC_1);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);

	#ifdef ZQD_DEBUG
	BEEP = 0;
	#endif
}

//机械臂上升
void Robot_armUp(void)
{
	//原来板子：1960
	//V1.0:550
	u16 i,t;
	u16 W=2700;
	u16 nms=2000;

	
	if(LimitSwitchUp==1)
	{
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MOTOR_STATIC_1);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);
		return ;
	}
	//EXTIX_Enable(0);
	#ifdef ZQD_DEBUG
	BEEP = 1;
	#endif
	
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,W);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);
	for(i=0;i<nms;i++)
	{
		if(LimitSwitchUp == 1)
		{
			for(t=0;t<0xff;t++);
			if(LimitSwitchUp == 1)
			{
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MOTOR_STATIC_1);
				__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);
				break;
			}
		}
		for(t=0;t<0x4fff;t++)
			if(LimitSwitchUp == 1)
				break;
	}
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1,MOTOR_STATIC_1);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2,MOTOR_STATIC_2);	

	#ifdef ZQD_DEBUG
	BEEP = 0;
	#endif
}



//根据偏差大小调整角速度
static float AdjustAngleV(float D_Theta)
{
	float Vw = 0;
	
	//大于30°线性控制
	if(D_Theta>0&&(D_Theta<180))  
	{
		Vw=D_Theta;
	}
	else if(D_Theta>0&&(D_Theta>=180)) 
	{
		D_Theta = 360-D_Theta;
		Vw=-D_Theta;
	}
	else if(D_Theta<0&&(D_Theta>=-180)) 
	{
		D_Theta = -D_Theta;
		Vw=-D_Theta;
	}
	else if(D_Theta<0&&(D_Theta<-180)) 
	{
		D_Theta = 360+D_Theta;
		Vw=D_Theta;
	}
	else 
		//Vw=Vw;
	
	//小于60°大于30°匀速
	//实际PWM为100
	if(D_Theta < 60)
	{
		if(Vw>0)
			Vw = 1000;
		else
			Vw = -1000;
	}
	
	//小于30°大于5°
	if(D_Theta < 30)
	{
		if(Vw>30)
			Vw = 200;
		else
			Vw = -200;
	}
	//小于5°
	if(D_Theta < 5)
	{
		if(Vw>0)
			Vw = 40;
		else
			Vw = -40;
	}
	if(D_Theta == 0)
		Vw = 0;
	
	return Vw;
}


//根据偏差大小调整Y轴速度
static float AdjustVy(float D_Y)
{
	float sy;
	
	if(D_Y > 0.05f)
	{
			
		if(D_Y >= 1.5f)
		{
			sy = 8;
			if(BasketballRobot.Vy < 0.3f)
				sy = 2;
			else if(BasketballRobot.Vy < 0.5f)
				sy = 4;
			else if(BasketballRobot.Vy < 0.9f)
				sy = 6;
		}
		if(D_Y < 1.5f&&D_Y > 0.2f)  
			sy = D_Y*10/2;
			
		if(D_Y < 0.2f)	
			sy = 0.25;			
		}
	else if(D_Y < -0.05f)
	{
		if(D_Y < -1.5f)
		{
			sy = -8;
			if(BasketballRobot.Vy > -0.3f)
				sy = -2;
			else if(BasketballRobot.Vy > -0.5f)
				sy = -4;
			else if(BasketballRobot.Vy > -0.9f)
				sy = -6;
		}
		
		if(D_Y > -1.5f&&D_Y < -0.2f)  
			sy = D_Y*10/2;
			
		if(D_Y > -0.2f)
			sy = -0.25;
		}
	else 
		sy = 0;
	
	return sy;
	
}
	

//根据偏差大小调整X轴速度
static float AdjustVx(float D_X)
{
	float sx;
	
	if(D_X > 0.05f)
	{
		if(D_X > 1.5f)
		{
			sx = 8;
			if(BasketballRobot.Vx < 0.1f)
				sx = 0.5;
			else if(BasketballRobot.Vx < 0.2f)
				sx = 1;
			else if(BasketballRobot.Vx < 0.4f)
				sx = 2;
			else if(BasketballRobot.Vx < 0.58f)
				sx = 4;
			else if(BasketballRobot.Vx < 7.5f)
				sx = 6;
		}
		else if(D_X < 1.5f)
		{
			if(D_X > 0.2f)
				sx = 2;
				
			else if(D_X > 0.15f)
			sx = 1;
				
			else if(D_X > 0.1f)
				sx = 0.5f;
			
			else
				sx = 0.25f;	
		}
	}
	else if(D_X < -0.05f)
	{
		if(D_X < -1.5f)
		{
			sx = -8;
			if(BasketballRobot.Vx > -0.1f)
				sx = -0.5;
			else if(BasketballRobot.Vx > -0.2f)
				sx = -1;
			else if(BasketballRobot.Vx > -0.4f)
				sx = -2;
			else if(BasketballRobot.Vx > -0.58f)
				sx = -4;
			else if(BasketballRobot.Vx > -7.5f)
				sx = -6;
		}
		else if(D_X > -1.5f)
		{
			if(D_X < -0.2f)
				sx = -2;
				
			else if(D_X < -0.15f)
				sx = -1;

			else if(D_X < -0.1f)
				sx = -0.5f;
			else
				sx = -0.25f;
		}
	}
	else 
		sx = 0;
	
	return sx;
	
}


//自旋运动，根据误差角度，自动调节
void RobotRotate(float theta)
{
	float D_Theta;
	float Vw=0;        //W大于0 逆时针

	//D_Theta = theta-BasketballRobot.ThetaD;
	D_Theta = theta-0;
	Vw = AdjustAngleV(D_Theta);
	
	
	while(D_Theta>1||D_Theta < -1)
	{
		GetMotorVelocity(0,0,Vw);	
		
		SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		
		D_Theta = theta-BasketballRobot.ThetaD;
		
		Vw = AdjustAngleV(D_Theta);
	}
	SetPWM(0,0,0);

	while(BasketballRobot.W);
}


//行至指定坐标
//X_I:目标坐标的X
//Y_I:目标坐标的Y
//Theta_I:目标坐标的角度
void RobotGoTo(float X_I,float Y_I,float Theta_I)
{
	float D_Theta,D_X,D_Y,Vw=0,sx,sy=0;
	
	D_Theta =  Theta_I - BasketballRobot.ThetaD;	//角度差
	D_X = X_I - BasketballRobot.X;				
	D_Y = Y_I - BasketballRobot.Y;
	
	while(fabs(D_Y) > 0.05f || fabs(D_X) > 0.05f)
	{
		sy = AdjustVy(D_Y);		
		
		sx = AdjustVx(D_X);
		
		Vw = AdjustAngleV(D_Theta)/2;
		
		GetMotorVelocity(sx*12,sy*100,Vw);
		
		SetPWM(BasketballRobot.Velocity[0],BasketballRobot.Velocity[1],BasketballRobot.Velocity[2]);
		
		D_Theta =  Theta_I - BasketballRobot.ThetaD;
		D_X = X_I - BasketballRobot.X;
		D_Y = Y_I - BasketballRobot.Y;
	}
	SetPWM(0,0,0);
	delay_ms(1000);
	RobotRotate(Theta_I);
}









