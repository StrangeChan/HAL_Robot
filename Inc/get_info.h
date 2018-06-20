#ifndef __GETPOSITION_H
#define __GETPOSITION_H


#include "stm32f4xx_hal.h"
#include "tim.h"
#include "control.h"
#include "sys.h"



void ReadEncoder(void);
void GetYaw(void);
void GetPosition(void);		//����ת��

void GetVisionData(void);		//�Ӿ����ݴ���
void GetRadarData(void);		//���⴦������

#endif

