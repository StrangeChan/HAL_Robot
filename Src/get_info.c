#include "get_info.h"



void ReadEncoder(void)
{
	int16_t nEncoder1,nEncoder2,nEncoder3;
	//读取CNT数值后清零
	nEncoder1 = __HAL_TIM_GET_COUNTER(&htim2);
	__HAL_TIM_SET_COUNTER(&htim2,0);
	
	nEncoder2 = __HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,0);
	
	nEncoder3 = __HAL_TIM_GET_COUNTER(&htim8);
	__HAL_TIM_SET_COUNTER(&htim8,0);
	
	BasketballRobot.w[2] += nEncoder3;
	BasketballRobot.w[1] += nEncoder2;
	BasketballRobot.w[0] += nEncoder1;  
}

void GetYaw(void)
{
	uint16_t temp;
	if(aRxBuffer2[0]==0x55&&aRxBuffer2[1]==0x53)
	{
		u8  sum,i;
		sum =0;
		for(i = 0;i <10;i++)
			sum += aRxBuffer2[i];
		
		
		//偏航角（z 轴） Yaw=((YawH<<8)|YawL)/32768*180(°)
		if(sum == aRxBuffer2[10])
		{
			temp = aRxBuffer2[7];
			BasketballRobot.ThetaD = ((float)((temp<<8)|aRxBuffer2[6]))/32768*180;
//			receive2 = 0;
//			USART2_RX_STA=0;
			
			BasketballRobot.ThetaR = BasketballRobot.ThetaD * PI / 180 + BasketballRobot.theta_offset;
			
			while(BasketballRobot.ThetaR < 0)
				BasketballRobot.ThetaR  = BasketballRobot.ThetaR + PI + PI;
			
			while (BasketballRobot.ThetaR > 2 * PI)
				BasketballRobot.ThetaR = BasketballRobot.ThetaR - PI - PI;
			
			while(BasketballRobot.ThetaD < 0)
				BasketballRobot.ThetaD  = BasketballRobot.ThetaD + 360;
			
			while (BasketballRobot.ThetaD >360)
				BasketballRobot.ThetaD = BasketballRobot.ThetaD - 360;
		}
	}
}


//坐标转换,里程计定位
void GetPosition(void)
{
	//根据速度运算球场坐标

	float nW,nX,nY;
	

	float l1,l2,l3;	//里程计减去自旋偏差后数值
	
	float theta_inv[2][2]; //角度矩阵
	
	GetYaw();
	ReadEncoder();
	
	BasketballRobot.LastTheta = BasketballRobot.ThetaR;
	
	
	//theta_inv
	theta_inv[0][0]= cos(BasketballRobot.ThetaR);	theta_inv[0][1] = -theta_inv[1][0];		
	theta_inv[1][0]= sin(BasketballRobot.ThetaR);	theta_inv[1][1] = theta_inv[0][0];	
	
	nW = (BasketballRobot.w[0]+BasketballRobot.w[1]+BasketballRobot.w[2])/3.0f;	
	//除去自传偏差
	l1 = BasketballRobot.w[0] - nW;	
	l2 = BasketballRobot.w[1] - nW;
	l3 = BasketballRobot.w[2] - nW;
	
	nX = (l1-l2-l3)/2.0f;
	nY = (-l2 + l3)/1.7320508f;
	
	BasketballRobot.X += nX*theta_inv[0][0]+nY*theta_inv[1][0];
	BasketballRobot.Y += nX*theta_inv[0][1]+nY*theta_inv[1][1];
	

}

//视觉数据处理
u8 GetVisionData(void)
{	
	u8 check_sum = 0,i;
	u32 x,d;

	if(aRxBuffer1[7] == 'z')
	{
		for(i = 0;i <7;i++)
			check_sum += aRxBuffer1[i];
		
		if(check_sum == 'z')
			check_sum += '1';
		
		if(check_sum == aRxBuffer1[8])
		{					   
			//坐标位置信息
			if(aRxBuffer1[0]!=' ')
				x = (aRxBuffer1[0]-'0')*100;
			else 
				x = 0;
			aRxBuffer1[0] = ' ';
			
			if(aRxBuffer1[1]!=' ')
				x += (aRxBuffer1[1]-'0')*10;
			aRxBuffer1[1] = ' ';
			
			if(aRxBuffer1[2]!=' ')
				x += (aRxBuffer1[2]-'0');
			aRxBuffer1[2] = ' ';
			
			//深度信息
			if(aRxBuffer1[3]!=' ')
				d=(aRxBuffer1[3]-'0')*1000;
			else 
				d=0;
			aRxBuffer1[3] = ' ';
			
			if(aRxBuffer1[4]!=' ')
				d+=(aRxBuffer1[4]-'0')*100;
			aRxBuffer1[4] = ' ';
			
			if(aRxBuffer1[5]!=' ')
				d+=(aRxBuffer1[5]-'0')*10;
			aRxBuffer1[5] = ' ';
			
			if(aRxBuffer1[6]!=' ')
				d+=(aRxBuffer1[6]-'0');
			aRxBuffer1[6] = ' ';
			
			receive=0;
		}	

	}
	
	if(x > 10 ||x < 630)
		Vision.X = x;
	if(d>500)
		Vision.Depth = d;
	
	LCD_ShowString(30+200,420,200,16,16,"View :pix");	
	LCD_ShowNum(30+200+48+8+45,420,Vision.X,4,16);		
	LCD_ShowString(30+200,440,200,16,16,"View :length");	
	LCD_ShowNum(30+200+48+8+45,440,Vision.Depth,4,16);	
}

//激光处理数据
u8 GetRadarData(void)
{
	
	if(USART3_RX_STA&0x8000)
	{					   
		//得到此次接收到的数据长度
		//len=USART1_RX_STA&0x3fff;

		
		//距离信息
		if(USART3_RX_BUF[0]!=' ')
			Radar.Distance=(USART3_RX_BUF[0]-'0')*1000;
		else 
			Vision.Depth=0;
		USART3_RX_BUF[0] = ' ';
		
		if(USART3_RX_BUF[1]!=' ')
			Vision.Depth+=(USART3_RX_BUF[1]-'0')*100;
		USART3_RX_BUF[1] = ' ';
		
		if(USART3_RX_BUF[2]!=' ')
			Vision.Depth+=(USART3_RX_BUF[2]-'0')*10;
		USART3_RX_BUF[2] = ' ';
		
		if(USART3_RX_BUF[3]!=' ')
			Vision.Depth +=(USART3_RX_BUF[3]-'0');
		USART3_RX_BUF[3] = ' ';
		
		//角度信息
		if(USART3_RX_BUF[4]!=' ')
			Radar.Angle=(USART3_RX_BUF[4]-'0')*100;
		else 
			Radar.Angle=0;
		USART3_RX_BUF[4] = ' ';
		if(USART3_RX_BUF[5]!=' ')
			Radar.Angle+=(USART3_RX_BUF[5]-'0')*10;
		USART3_RX_BUF[5] = ' ';
		if(USART3_RX_BUF[6]!=' ')
			Radar.Angle+=(USART3_RX_BUF[6]-'0');
		USART3_RX_BUF[6] = ' ';

		LCD_ShowString(30+200,460,200,16,16,"Radar:rad");	
		LCD_ShowNum(30+200+48+8+45,460,Radar.Angle,4,16);		
		LCD_ShowString(30+200,480,200,16,16,"Radar:length");	
		LCD_ShowNum(30+200+48+8+45,480,Vision.Depth,4,16);	
		
		USART3_RX_STA=0;
		receive3=0;
	}
	
	if(Radar.Angle<240 || Radar.Angle >300) //原来&&
		return 0;
	if(Vision.Depth>4000)
		return 0;
	
	return 1;
}

