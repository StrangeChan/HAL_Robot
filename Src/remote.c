#include "remote.h"




void Remote_Init(void)
{  
	MX_TIM1_Init();//CubleMX���ɴ���
}



//ң��������״̬
//[7]:�յ����������־
//[6]:�õ���һ��������������Ϣ
//[5]:����	
//[4]:����������Ƿ��Ѿ�������								   
//[3:0]:�����ʱ��
u8 	RmtSta=0;	  	  
u16 Dval;		//�½���ʱ��������ֵ
u32 RmtRec=0;	//������յ�������	   		    
u8  RmtCnt=0;	//�������µĴ���	 



//��ʱ�����£������
void myRemoteUpCallback(void)
{
	if(RmtSta&0x80)//�ϴ������ݱ����յ���
	{	
		RmtSta&=~0X10;						//ȡ���������Ѿ���������
		if((RmtSta&0X0F)==0X00)RmtSta|=1<<6;//����Ѿ����һ�ΰ����ļ�ֵ��Ϣ�ɼ�
		if((RmtSta&0X0F)<14)RmtSta++;
		else
		{
			RmtSta&=~(1<<7);//���������ʶ
			RmtSta&=0XF0;	//��ռ�����	
		}						 	   	
	}	
}

//�����жϷ���ʱִ��
void myRemoteCcCallback(void)
{
 	if(RDATA)//�����ز���
	{
		TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
		TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_ICPOLARITY_FALLING);//CC1P=1 ����Ϊ�½��ز���
		__HAL_TIM_SET_COUNTER(&htim1,0);  //��ն�ʱ��ֵ   	  
		RmtSta|=0X10;					//����������Ѿ�������
	}
	else //�½��ز���
	{
		Dval=HAL_TIM_ReadCapturedValue(&htim1,TIM_CHANNEL_1);//��ȡCCR1Ҳ������CC1IF��־λ
		TIM_RESET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1);   //һ��Ҫ�����ԭ�������ã���
		TIM_SET_CAPTUREPOLARITY(&htim1,TIM_CHANNEL_1,TIM_ICPOLARITY_RISING);//����TIM5ͨ��1�����ز���
		if(RmtSta&0X10)					//���һ�θߵ�ƽ���� 
		{
			if(RmtSta&0X80)//���յ���������
			{
						
				if(Dval>300&&Dval<800)			//560Ϊ��׼ֵ,560us
				{
					RmtRec<<=1;	//����һλ.
					RmtRec|=0;	//���յ�0	   
				}else if(Dval>1400&&Dval<1800)	//1680Ϊ��׼ֵ,1680us
				{
					RmtRec<<=1;	//����һλ.
					RmtRec|=1;	//���յ�1
				}else if(Dval>2200&&Dval<2600)	//�õ�������ֵ���ӵ���Ϣ 2500Ϊ��׼ֵ2.5ms
				{
					RmtCnt++; 		//������������1��
					RmtSta&=0XF0;	//��ռ�ʱ��		
				}
			}
			else if(Dval>4200&&Dval<4700)		//4500Ϊ��׼ֵ4.5ms
			{
				RmtSta|=1<<7;	//��ǳɹ����յ���������
				RmtCnt=0;		//�����������������
			}						 
		}
		RmtSta&=~(1<<4);
	}				 		     	    
}

//����������
//����ֵ:
//	 0,û���κΰ�������
//����,���µİ�����ֵ.
u8 Remote_Scan(void)
{        
	u8 sta=0;       
	u8 t1,t2;  
	if(RmtSta&(1<<6))//�õ�һ��������������Ϣ��
	{ 
	    t1=RmtRec>>24;			//�õ���ַ��
	    t2=(RmtRec>>16)&0xff;	//�õ���ַ���� 
 	    if((t1==(u8)~t2)&&t1==REMOTE_ID)//����ң��ʶ����(ID)����ַ 
	    { 
	        t1=RmtRec>>8;
	        t2=RmtRec; 	
	        if(t1==(u8)~t2)sta=t1;//��ֵ��ȷ	 
		}   
		if((sta==0)||((RmtSta&0X80)==0))//�������ݴ���/ң���Ѿ�û�а�����
		{
		 	RmtSta&=~(1<<6);//������յ���Ч������ʶ
			RmtCnt=0;		//�����������������
		}
	}  
	if(sta)
	{
		delay_ms(500);
		RmtSta = 0;
	}
	return sta;
}
