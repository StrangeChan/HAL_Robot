#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "sys.h"
#include "usart.h"
#include "delay.h"
#include "lcd.h"
#include "gpio.h"
#include "tim.h"
#include "mpu6050.h"
#include "remote.h"
#include "get_info.h"
#include "find_ball.h"
//#include "timer.h"
#include <math.h>


#define PI 		3.141592654f

#define ENCODER_FD 	4.0f			//��������Ƶ
#define ENCODER_MAX 500.0f		//������������
#define ENCODER_T 	0.01f   		//���������㵱ǰ�ٶ�ʱ��
#define ENCODER_L 	0.2006f		//�������ֵ����������ĵľ���
#define ENCODER_R 	0.0275f		//���������Ӱ뾶
#define RAD	 0.1570796327f		//������һ�������Ӧ�ĽǶ� pi/500/4/0.01

#define MOTOR_L 0.2013f		//�ֵ����������ĵľ���
#define MOTOR_R 0.0508f		//���ӵİ뾶

#define MOTOR_STATIC_1 4000		//TIM9 CH1 PE5
#define MOTOR_STATIC_2 4000  		//TIM9 CH2 PE6

#define RADAR_MID 268	//�״ﶨλ����
#define VISION_MID 320	//�Ӿ���λ����
#define DIS_RADAR 2500	//�����״ﶨλ����
#define DIS_VISION 280	//�����Ӿ���λ����


struct ROBOT
{
	float X;		//������������ϵ��x����
	float Y;		//������������ϵ��y����
	//float x;		//������������ϵ��x����
	//float y;		//������������ϵ��y����
	float ThetaR;	//�������������y��н� ����
	float ThetaD;	//�������������y��н� �Ƕ�

	float Vx;		//������������ϵx�����ٶ�
	float Vy;		//������������ϵy�����ٶ�	
	float W;		//�����˽��ٶȣ�˳ʱ��������
	
	float w[3];		//��������ʵ�ʼ���
	float v[3];		//�����������ٶ�
	
	float Velocity[3];	//���ӵ��ٶ�
	float LastTheta;	//��һʱ�̣�������theta��
	float theta_offset;	//�Ƕ�ƫ�����
};

//�����״����ݣ�������
struct RADAR
{
	uint32_t Distance;  //����
	
	uint32_t Angle;	//�Ƕ�
};	

//�����Ӿ�����
struct VISION
{
	uint32_t Depth;	//��ȣ�����
	
	uint32_t X;		//Xλ�ã�����
};


extern struct ROBOT BasketballRobot;

extern struct RADAR Radar;
extern struct VISION Vision;


void Control_Init(void);		//�����˳�ʼ��


static void Velocity2PWM(float *V);		//����ٶ�ת����PWM��ֵ��ԭ������������ֲ�
void SetPWM(float V1,float V2,float V3); 	//������������PWM

void GetMotorVelocity(float vx,float vy,float w);		//�����������ٶ�������ӵ��ٶ�
void GetMotorVelocity_Self(float vx,float vy,float w);	//����������ϵ�ٶ�������ӵ��ٶ�


void GetInfraredState(void);	//��ȡ���⿪��״̬
void Robot_armDown(void);	//��е���½�
void Robot_armUp(void);		//��е������

uint8_t GetVisionData(void);		//�Ӿ����ݴ���
uint8_t GetRadarData(void);		//���⴦������


static float AdjustAngleV(float D_Theta);		//����ƫ���С�������ٶ�
static float AdjustVy(float D_Y);			//����ƫ���С����Y���ٶ�
static float AdjustVx(float D_X);			//����ƫ���С����X���ٶ�


void RobotRotate(float theta);	//�����˶����������Ƕȣ��Զ�����

void RobotGoTo(float X_I,float Y_I,float Theta_I);	//����ָ������
#endif
