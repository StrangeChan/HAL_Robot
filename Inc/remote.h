#ifndef __REMOTE_H
#define __REMOTE_H
#include "sys.h"
#include "delay.h"
#include "tim.h"

#define RDATA   PAin(8)//�������������

//����ң��ʶ����(ID),ÿ��ң�����ĸ�ֵ��������һ��,��Ҳ��һ����.
//����ѡ�õ�ң����ʶ����Ϊ0
#define REMOTE_ID 0   

#define KEY_UP 98
#define KEY_DOWN 168
#define KEY_LEFT 34
#define KEY_RIGHT 194
#define KEY_POWER 162
#define KEY_PLAY 2
#define KEY_1 104
#define KEY_2 152
#define KEY_3 176
#define KEY_4 48
#define KEY_5 24
#define KEY_6 122
#define KEY_7 16
#define KEY_8 56
#define KEY_9 90
#define KEY_0 66
#define KEY_VOL_A 144
#define KEY_VOL_D 224

extern u8 RmtCnt;	        //�������µĴ���
extern u8 RmtSta;	  	  
extern u16 Dval;		//�½���ʱ��������ֵ
extern u32 RmtRec;	//������յ�������	   		    



void Remote_Init(void);     //���⴫������ʼ��

void myRemoteUpCallback(void);	//��ʱ�����£������ִ��
void myRemoteCcCallback(void);	//�����жϷ���ʱִ��

u8 Remote_Scan(void);
#endif
